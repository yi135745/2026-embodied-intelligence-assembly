"""语音协议离线回归：不接盒子、不录音、不加载本地模型。"""
import io
import json
import sys
import unittest
from http.client import IncompleteRead
from pathlib import Path
from unittest.mock import Mock, patch
from urllib.error import HTTPError, URLError

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import config
from modules.voice import Voice, VoiceServiceError


HEALTH = {"ready": True, "capture_ready": "lazy", "models": {
    "piper_exists": True, "wakeup_keyword": "小E同学"}}


def reply(digest):
    return {"ok": True, "error_code": "OK", "result_digest": digest}


def response(value):
    return io.BytesIO(json.dumps(value, ensure_ascii=False).encode("utf-8"))


class BoxVoiceTests(unittest.TestCase):
    def setUp(self):
        self.opener = Mock()
        self.opener.open.return_value = response(HEALTH)
        factory = patch("drivers.ai_box.build_opener", return_value=self.opener)
        self.factory = factory.start()
        self.addCleanup(factory.stop)
        self.voice = Voice(backend="ai_box")

    def request_data(self):
        request = self.opener.open.call_args.args[0]
        return request, json.loads(request.data.decode("utf-8"))

    def test_health_is_get_and_disables_proxies(self):
        request = self.opener.open.call_args.args[0]
        self.assertEqual(request.full_url, config.AI_BOX_URL.rstrip("/") + "/health")
        self.assertEqual(request.method, "GET")
        self.assertIsNone(request.data)
        self.assertEqual(self.factory.call_args.args[0].proxies, {})
        self.assertEqual(self.voice.health, HEALTH)

    def test_asr_payload_and_normalization(self):
        self.opener.open.return_value = response(reply({"instruction": "任务1。"}))
        self.assertEqual(self.voice.listen(), "任务一")
        request, payload = self.request_data()
        self.assertTrue(request.full_url.endswith("/v1/asr"))
        self.assertEqual(request.method, "POST")
        self.assertFalse(payload["wakeup_required"])
        self.assertTrue(payload["prewoken"])
        self.assertEqual(payload["max_record_seconds"], config.AI_BOX_MAX_RECORD_SECONDS)
        self.assertIn("request_id", payload)
        self.assertEqual(self.opener.open.call_args.kwargs["timeout"], config.AI_BOX_ASR_TIMEOUT)

    def test_silence_returns_empty(self):
        self.opener.open.return_value = response({
            "ok": False, "error_code": "NO_SPEECH_DETECTED", "retryable": True})
        self.assertEqual(self.voice.listen(), "")

    def test_failures_are_not_silence_even_if_http_200(self):
        for code in ("ASR_TIMEOUT", "WAKEUP_TIMEOUT", "SPEECH_REAL_CAPTURE_FAILED",
                     "SPEECH_SERVICE_NOT_READY", "ASR_MODEL_MISSING", "UNKNOWN"):
            with self.subTest(code=code):
                self.opener.open.return_value = response({
                    "ok": False, "error_code": code, "retryable": True, "message": "failed"})
                with self.assertRaisesRegex(VoiceServiceError, code):
                    self.voice.listen()

    def test_nonretryable_silence_raises(self):
        self.opener.open.return_value = response({
            "ok": False, "error_code": "NO_SPEECH_DETECTED", "retryable": False})
        with self.assertRaises(VoiceServiceError):
            self.voice.listen()

    def test_invalid_digest_or_instruction_raises(self):
        for value in (reply(None), reply([]), reply({}), reply({"instruction": 1}),
                      {"ok": "true", "result_digest": {"instruction": "任务一"}},
                      {"ok": True, "error_code": "FAILED", "result_digest": {}}, {}):
            with self.subTest(value=value):
                self.opener.open.return_value = response(value)
                with self.assertRaises(VoiceServiceError):
                    self.voice.listen()

    def test_invalid_json_or_nonobject_raises(self):
        for data in (b"<html>bad gateway</html>", b"[]", b"null", b"\xff"):
            with self.subTest(data=data):
                self.opener.open.return_value = io.BytesIO(data)
                with self.assertRaises(VoiceServiceError):
                    self.voice.listen()

    def test_network_errors_do_not_retry(self):
        for exc in (URLError("offline"), TimeoutError("timeout"), IncompleteRead(b"partial"),
                    HTTPError(config.AI_BOX_URL, 503, "unavailable", {}, None)):
            with self.subTest(exc=exc):
                self.opener.open.reset_mock()
                self.opener.open.side_effect = exc
                with self.assertRaisesRegex(VoiceServiceError, "未自动重试"):
                    self.voice.listen()
                self.assertEqual(self.opener.open.call_count, 1)

    def test_health_ready_does_not_imply_piper_ready(self):
        for health in ({"ready": False, "message": "missing asr"},
                       {"ready": True, "models": {"piper_exists": False}},
                       {"ready": True}, {"ready": "true", "models": HEALTH["models"]}):
            with self.subTest(health=health):
                self.opener.open.return_value = response(health)
                with self.assertRaises(VoiceServiceError):
                    self.voice.check_health()

    def test_tts_uses_offline_piper_and_utf8(self):
        self.opener.open.return_value = response(reply({"interrupted": False}))
        self.voice.speak("我已就绪，请下达指令")
        request, payload = self.request_data()
        self.assertTrue(request.full_url.endswith("/v1/tts"))
        self.assertEqual(payload["text"], "我已就绪，请下达指令")
        self.assertEqual(payload["backend"], "piper")
        self.assertEqual(payload["voice"], "default")
        self.assertEqual(self.opener.open.call_args.kwargs["timeout"], config.AI_BOX_TTS_TIMEOUT)

    def test_tts_skipped_interrupted_or_failed_raises(self):
        for result in (reply({"tts_skipped": True}), reply({"interrupted": True}),
                       {"ok": False, "error_code": "TTS_FAILED"}, reply(None)):
            with self.subTest(result=result):
                self.opener.open.return_value = response(result)
                with self.assertRaises(VoiceServiceError):
                    self.voice.speak("测试")

    def test_wake_requires_contest_word_then_next_listen_gets_command(self):
        self.opener.open.side_effect = [response(reply({"instruction": text})) for text in (
            "任务一", "小E同学", "小菊同学", "任务2")]
        self.assertTrue(self.voice.wake(config.WAKE_WORD))
        self.assertEqual(self.voice.listen(), "任务二")
        # 初始health一次 + 三次等待唤醒 + 一次命令。
        self.assertEqual(self.opener.open.call_count, 5)

    def test_exit_during_wake(self):
        self.opener.open.return_value = response(reply({"instruction": "退出系统"}))
        self.assertFalse(self.voice.wake(config.WAKE_WORD))
        self.assertTrue(self.voice.is_exit("退出系统"))

    def test_box_does_not_initialize_local_asr(self):
        with patch.object(Voice, "_load_asr_model") as loader:
            self.opener.open.return_value = response(HEALTH)
            Voice(backend="ai_box")
        loader.assert_not_called()

    def test_invalid_backend_rejected(self):
        with self.assertRaises(ValueError):
            Voice(backend="unknown")


class LocalVoiceTests(unittest.TestCase):
    def test_local_is_explicit_and_never_contacts_box(self):
        with patch.object(Voice, "_load_asr_model") as loader, \
                patch("drivers.ai_box.build_opener") as factory:
            voice = Voice(backend="local")
        loader.assert_called_once()
        factory.assert_not_called()
        with patch.object(voice, "_record_audio", return_value=Path("unused.wav")) as record, \
                patch.object(voice, "_recognize_audio", return_value="任务一"):
            self.assertEqual(voice.listen(), "任务一")
        record.assert_called_once()


if __name__ == "__main__":
    unittest.main()
