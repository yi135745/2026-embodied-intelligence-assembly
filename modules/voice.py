"""语音模块：AI盒子HTTP服务；可显式切回省赛本地FunASR/pyttsx3。

对外接口：
    Voice.wake(wake_word)  -> bool   阻塞监听，命中唤醒词返回 True，听到退出指令返回 False
    Voice.is_exit(text)    -> bool   判断是否为全局退出指令
    Voice.listen()         -> str    一次「录音 + 识别 + 热词纠错」
    Voice.speak(text)      -> None   语音播报
"""

import difflib
import logging
import math
import os
import re
import struct
import sys
import threading
import time
import warnings
import wave
from pathlib import Path

import config
from drivers.ai_box import AiBoxClient, AiBoxRequestError


class VoiceServiceError(RuntimeError):
    """盒子连接、协议或设备失败；不能当成静音继续运行。"""


def _configure_logs() -> None:
    """降低 FunASR/ModelScope 的日志噪声，保留关键输出。"""
    warnings.filterwarnings("ignore")
    logging.getLogger("modelscope").setLevel(logging.ERROR)
    logging.getLogger("funasr").setLevel(logging.ERROR)
    logging.getLogger().setLevel(logging.ERROR)
    os.environ["MODELSCOPE_LOG_LEVEL"] = "40"


def _log_asr_status(message: str) -> None:
    print("[ASR] " + message, file=sys.stderr, flush=True)


class Voice:
    """保持wake/listen/speak/is_exit接口，业务层无需区分语音后端。"""

    def __init__(self, backend=None):
        self.backend = backend or config.VOICE_BACKEND
        if self.backend not in ("ai_box", "local"):
            raise ValueError("VOICE_BACKEND只能是ai_box或local")
        # pyttsx3 引擎按需初始化，播报时用锁避免通道冲突。
        self._tts_lock = threading.Lock()
        if self.backend == "local":
            _configure_logs()
            self._load_asr_model()
        else:
            self._box_client = AiBoxClient(config.AI_BOX_URL)
            self.health = self.check_health()
            native_word = self.health.get("models", {}).get("wakeup_keyword", "未知")
            _log_asr_status("AI盒子已连接；原生唤醒词=%s，当前使用ASR文字匹配唤醒=%s"
                            % (native_word, config.WAKE_WORD))

    def _box_request(self, path, payload=None, timeout=5.0):
        try:
            return self._box_client.request(path, payload, timeout)
        except AiBoxRequestError as exc:
            # 请求超时不代表盒子已停止录音/播报，禁止自动重发造成重入。
            raise VoiceServiceError(str(exc)) from exc

    def check_health(self):
        """只检查资源是否就绪；不代表麦克风/扬声器已经通过实测。"""
        if self.backend != "ai_box":
            raise ValueError("health仅适用于ai_box后端")
        result = self._box_request("/health", timeout=config.AI_BOX_HEALTH_TIMEOUT)
        if result.get("ready") is not True:
            raise VoiceServiceError("AI盒子未就绪：%s" % result.get("message", result))
        models = result.get("models")
        if not isinstance(models, dict) or models.get("piper_exists") is not True:
            raise VoiceServiceError("AI盒子缺少离线Piper资源，不能确认播报可用")
        return result

    @staticmethod
    def _box_result(result, operation):
        # HTTP 200也可能失败；必须同时检查业务状态和摘要结构。
        if result.get("ok") is not True or result.get("error_code", "OK") != "OK":
            raise VoiceServiceError("AI盒子%s失败 [%s]：%s" % (
                operation, result.get("error_code", "UNKNOWN"), result.get("message", "无详情")))
        digest = result.get("result_digest")
        if not isinstance(digest, dict):
            raise VoiceServiceError("AI盒子%s缺少有效result_digest" % operation)
        return digest

    def _box_listen(self):
        result = self._box_request("/v1/asr", {
            "mode": "live_capture", "language": "zh-CN",
            # 不等待原生“小E同学”；由wake()匹配比赛用“小具同学”。
            "wakeup_required": False, "prewoken": True,
            "start_timeout_s": config.AI_BOX_START_TIMEOUT,
            "max_record_seconds": config.AI_BOX_MAX_RECORD_SECONDS,
            "vad_threshold": config.AI_BOX_VAD_THRESHOLD,
        }, timeout=config.AI_BOX_ASR_TIMEOUT)
        if (result.get("ok") is False
                and result.get("error_code") == "NO_SPEECH_DETECTED"
                and result.get("retryable") is not False):
            return ""
        digest = self._box_result(result, "识别")
        text = digest.get("instruction")
        if not isinstance(text, str):
            raise VoiceServiceError("AI盒子识别结果instruction不是字符串")
        return self._normalize_text(text)

    def _box_speak(self, text):
        result = self._box_request("/v1/tts", {
            "text": text, "backend": "piper", "voice": "default", "context": {},
        }, timeout=config.AI_BOX_TTS_TIMEOUT)
        digest = self._box_result(result, "播报")
        if digest.get("tts_skipped") or digest.get("interrupted"):
            raise VoiceServiceError("AI盒子播报被跳过或打断，不能视为已播完：%s" % digest)

    # --------------------------------------------------------------
    # 模型加载
    # --------------------------------------------------------------
    def _load_asr_model(self) -> None:
        from funasr import AutoModel

        model_dir = Path(config.ASR_MODEL_DIR)
        if not model_dir.exists():
            raise SystemExit("找不到语音识别模型目录：" + str(model_dir))
        _log_asr_status("正在加载语音识别模型...")
        self.asr_model = AutoModel(
            model=str(model_dir),
            trust_remote_code=True,
            device="cpu",
            disable_update=True,
        )
        _log_asr_status("语音识别模型加载完成。")

    # --------------------------------------------------------------
    # 对外：唤醒 / 识别 / 播报
    # --------------------------------------------------------------
    def wake(self, wake_word: str) -> bool:
        """阻塞监听，直到命中唤醒词（返回 True）或听到退出指令（返回 False）。"""
        while True:
            text = self.listen()
            if not text:
                continue
            print("听到：" + text)
            if self._is_exit_command(text):
                return False
            # 盒子分支不使用省赛宽松相似度：否则“小E同学”也可能误中。
            matched = (wake_word in text if self.backend == "ai_box"
                       else self._has_wakeup_word(text, wake_word))
            if matched:
                return True

    def is_exit(self, text: str) -> bool:
        """判断是否为全局退出指令（「退出系统」），供命令阶段识别退出。"""
        return self._is_exit_command(text)

    def listen(self) -> str:
        """完成一次「VAD 录音 -> ASR 识别 -> 热词纠错」，返回标准命令文本。"""
        if self.backend == "ai_box":
            return self._box_listen()
        wav_file = self._record_audio(Path(config.TEMP_WAV_FILE))
        return self._recognize_audio(wav_file)

    def speak(self, text: str) -> None:
        """同步播完后返回；盒子失败时抛错，不静默切回电脑扬声器。"""
        print("正在播报：" + text)
        if self.backend == "ai_box":
            with self._tts_lock:
                self._box_speak(text)
            return
        import pyttsx3

        try:
            with self._tts_lock:
                engine = pyttsx3.init()
                engine.setProperty("rate", config.TTS_RATE)
                for voice in engine.getProperty("voices"):
                    if "zh-cn" in voice.id.lower() or "chinese" in voice.name.lower():
                        engine.setProperty("voice", voice.id)
                        break
                engine.say(text)
                engine.runAndWait()
        except Exception as exc:
            print("语音播报失败：" + str(exc))

    # --------------------------------------------------------------
    # 录音（VAD）
    # --------------------------------------------------------------
    @staticmethod
    def _get_rms(block: bytes) -> float:
        """计算音频块音量（RMS），用于判断是否有人开始说话。"""
        count = len(block) // 2
        fmt = "%dh" % count
        shorts = struct.unpack(fmt, block)
        sum_squares = 0.0
        for sample in shorts:
            normalized = sample / 32768.0
            sum_squares += normalized * normalized
        return math.sqrt(sum_squares / count) * 32768

    def _record_audio(self, output_file: Path) -> Path:
        """VAD 录音：先等声音超过阈值，再在连续静音后自动停止。"""
        import pyaudio

        audio_format = pyaudio.paInt16
        start_time = time.monotonic()
        _log_asr_status("开始打开麦克风")
        audio = pyaudio.PyAudio()
        sample_width = audio.get_sample_size(audio_format)
        stream = audio.open(
            format=audio_format,
            channels=config.CHANNELS,
            rate=config.RATE,
            input=True,
            frames_per_buffer=config.CHUNK,
        )
        _log_asr_status("麦克风已打开，开始录音")

        # 丢弃刚打开麦克风时的前 0.5 秒数据，减少设备初始化噪声和上轮播报尾音影响。
        for _ in range(int(config.RATE / config.CHUNK * 0.5)):
            stream.read(config.CHUNK, exception_on_overflow=False)

        frames = []
        silent_chunks = 0
        total_chunks = 0
        limit_chunks = int(config.SILENCE_LIMIT * config.RATE / config.CHUNK)
        max_chunks = int(config.MAX_DURATION * config.RATE / config.CHUNK)
        has_started = False

        try:
            while True:
                data = stream.read(config.CHUNK, exception_on_overflow=False)
                frames.append(data)
                total_chunks += 1
                rms = self._get_rms(data)

                if rms > config.THRESHOLD:
                    if not has_started:
                        _log_asr_status("检测到声音，RMS=%.1f，阈值=%d" % (rms, config.THRESHOLD))
                    has_started = True
                    silent_chunks = 0
                elif has_started:
                    silent_chunks += 1

                if (has_started and silent_chunks > limit_chunks) or total_chunks > max_chunks:
                    break
        finally:
            stream.stop_stream()
            stream.close()
            audio.terminate()

        output_file.parent.mkdir(parents=True, exist_ok=True)
        with wave.open(str(output_file), "wb") as wav_file:
            wav_file.setnchannels(config.CHANNELS)
            wav_file.setsampwidth(sample_width)
            wav_file.setframerate(config.RATE)
            wav_file.writeframes(b"".join(frames))
        _log_asr_status(
            "录音结束，用时 %.2f 秒，已检测到声音=%s，音频块=%d"
            % (time.monotonic() - start_time, "是" if has_started else "否", total_chunks)
        )
        return output_file

    # --------------------------------------------------------------
    # 语音识别
    # --------------------------------------------------------------
    def _recognize_audio(self, wav_file_path: Path) -> str:
        """调用 FunASR 转文字，并做热词纠错。"""
        try:
            start_time = time.monotonic()
            _log_asr_status("开始 FunASR 识别：" + str(wav_file_path))
            result = self.asr_model.generate(
                input=str(wav_file_path), cache={}, language="zh", use_itn=True
            )
            raw_text = re.sub(r"<\|.*?\|>", "", result[0]["text"]).strip()
            clean_text = self._normalize_text(raw_text)
            if raw_text and raw_text != clean_text:
                print("语音纠错：%s -> %s" % (raw_text, clean_text))
            _log_asr_status("FunASR 识别完成，用时 %.2f 秒，结果：%s" % (time.monotonic() - start_time, clean_text))
            return clean_text
        except Exception as exc:
            print("语音识别失败：" + str(exc))
            return ""

    @staticmethod
    def _normalize_text(text: str) -> str:
        """清理标点和空白，再应用热词纠错。"""
        text = re.sub(r"[，。！？、,!?\s]", "", text).strip()
        for wrong, right in sorted(config.ASR_CORRECTION_DICT.items(), key=lambda item: len(item[0]), reverse=True):
            if right in text:
                continue
            text = text.replace(wrong, right)
        return text

    # --------------------------------------------------------------
    # 唤醒词 / 退出指令判断
    # --------------------------------------------------------------
    @staticmethod
    def _has_wakeup_word(text: str, wake_word: str) -> bool:
        """优先精确匹配；失败时用相似度兜底，适应普通话不标准的情况。"""
        if wake_word in text:
            return True
        word_len = len(wake_word)
        for length in range(max(2, word_len - 1), word_len + 2):
            for start in range(0, max(len(text) - length + 1, 0)):
                fragment = text[start:start + length]
                if difflib.SequenceMatcher(None, fragment, wake_word).ratio() >= 0.65:
                    return True
        return False

    @staticmethod
    def _is_exit_command(text: str) -> bool:
        """「退出系统」作为全局语音指令。"""
        if "退出系统" in text:
            return True
        exit_phrases = ["退出", "关闭", "结束", "停止", "退岀", "推出系统", "退出西统", "关闭西统"]
        if any(phrase in text for phrase in exit_phrases):
            return True
        return difflib.SequenceMatcher(None, text, "退出系统").ratio() >= 0.6
