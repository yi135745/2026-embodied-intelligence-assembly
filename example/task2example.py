# v12版本，新增机器人运动前语音确认机制
# 区分方块使用模拟坐标，和实际拍照识别坐标的不同流程
import logging
import warnings
import os
import sys
import re
import json
import subprocess
import wave
import struct
import math
import time
import threading
import socket
from datetime import datetime
import pyaudio
import pyttsx3
import difflib  # 🤖 V6 新增：用于字符串相似度模糊匹配
from funasr import AutoModel
from openai import OpenAI
from ocr_engine import recognize_card_text

# 🤖 V8 新增：引入遨博 SDK
from pyaubo_sdk import RpcClient

# ==========================================
# 0. 运行环境配置与降噪
# ==========================================
# 屏蔽标准的 Python 警告
warnings.filterwarnings("ignore")

# 屏蔽 ModelScope 和 FunASR 的日志输出
logging.getLogger('modelscope').setLevel(logging.ERROR)
logging.getLogger('funasr').setLevel(logging.ERROR)
logging.getLogger().setLevel(logging.ERROR)

# 环境变量设置（进一步禁止更新检查提示）
os.environ["MODELSCOPE_LOG_LEVEL"] = "40"

# ==========================================
# 1. 全局配置参数
# ==========================================
PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))

# --- 1.1 比赛现场配置：现场调试优先修改这里 ---
# 海康 GigE 工业相机 IP 地址。需要和实际相机IP地址一致
HIK_CAMERA_IP = "192.168.1.10"

# AUBO 机器人控制器连接参数。需要和实际机器人参数一致，特别是IP地址必须一致
ROBOT_IP = "192.168.193.129"
ROBOT_PORT = 30004
ROBOT_NAME = "rob1"
ROBOT_USER = "aubo"
ROBOT_PASSWORD = "123456"

# AUBO 运动参数。XYZ 单位为 mm，RX/RY/RZ 单位为 rad，速度/加速度沿用 pyaubo_sdk moveLine 参数单位。
ROBOT_SPEED = 0.1                  # 机器人速度
ROBOT_ACCELERATION = 0.1           # 机器人加速度
ROBOT_REQUEST_TIMEOUT_MS = 5000    # 机器人Request超时ms
ROBOT_WAIT_TIMEOUT = 30.0          # 机器人超时s
ROBOT_POSITION_TOLERANCE = 1.0     # 机器人位置容忍误差mm
ROBOT_ORIENTATION_TOLERANCE = 0.01 # 机器人姿态容忍误差rad

# 任务卡2拍照位：[x, y, z, rx, ry, rz]。机器人会先移动到这里再拍照或读取任务卡图片。
# 实际比赛前，需要使用示教器标定并替换下面的示例位姿。
TASK_CARD2_VIEW_POSITION = [200.0, 150.0, 100.0, 0.0, 0.0, 0.0]

# 方块识别拍照位：[x, y, z, rx, ry, rz]。该位置与任务卡2拍照位相互独立。
# 实机模式下，机器人先移动到这里，再发送 ActionBlock 触发 VisionMaster 拍摄方块区域。
# 实际比赛前，需要使用示教器标定并替换下面的示例位姿。
BLOCK_VIEW_POSITION = [200.0, 250.0, 300.0, 0.0, 0.0, 0.0]

# 托盘识别拍照位：[x, y, z, rx, ry, rz]。识别完方块后，机器人移动到这里再次触发 VisionMaster。
# 实际比赛前，需要使用示教器标定并替换下面的示例位姿。
TRAY_VIEW_POSITION = [500.0, 250.0, 300.0, 0.0, 0.0, 0.0]

# 语音播报速度。指令复述较长，单独提高语速以减少比赛耗时。
TTS_RATE_NORMAL = 150        # 正常语速
TTS_RATE_TASK_REVIEW = 220   # 加快语速

# 录音参数
CHUNK = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000
TEMP_WAV_FILE = "temp_record.wav"
THRESHOLD = 300  # 音量阈值 (VAD)，将此值改为800或1000，能降低麦克风对环境底噪的灵敏度，唤醒小具同学时大声一点，能大幅提升比赛现场的抗干扰能力
SILENCE_LIMIT = 2  # 静音时长限制 (VAD)
MAX_DURATION = 15  # 最大录音时长
MAIN_COMMAND_MIN_DURATION = 6  # 主指令至少录满 6 秒，确保完整录到长句指令

# --- 1.2 全局视觉感知数据 ---
# 这里必须保留这个变量，因为后续安全匹配和机器人坐标查找都依赖它。
# v12 不在代码里写死坐标：运行时由 vision_data_sim.json 或 VisionMaster 网络数据填充。
GLOBAL_VISION_DATA = []

# --- 1.3 ASR 工业级热词纠错字典 ---
ASR_CORRECTION_DICT = {
    "托盘儿": "托盘",
    "方快": "方块",
    "黄涩": "黄色",
    "蓝涩": "蓝色",
    # 🤖 V10 新增：唤醒词同音字防错
    "小菊同学": "小具同学",
    "小桔同学": "小具同学",
    "小剧同学": "小具同学",
    "小局同学": "小具同学",
    "小聚同学": "小具同学",
    "小巨同学": "小具同学",
    "小据同学": "小具同学",
    "小橘同学": "小具同学",
    "玩具同学": "小具同学",
    "小俊同学": "小具同学",
    "小旭同学": "小具同学",
    "具同学": "小具同学",
    "小志同学":"小具同学"
}

# --- 1.4 任务卡 OCR 配置 ---
TASK_CARD2_IMAGE = "任务卡2.png"
TASK_CARD2_OCR_TEXT = "任务卡2_ocr.txt"
TASK_CARD_LLM_LOG_DIR = "llm_parse_logs"
TASK_CARD_CAPTURE_IMAGE = os.getenv("TASK_CARD_CAPTURE_IMAGE", "task_card2_capture.png").strip()
VISION_DATA_SIM_JSON = "vision_data_sim.json"
VISION_DATA_REAL_JSON = "vision_data_real.json"
VISION_MASTER_HOST = "192.168.1.20"
VISION_MASTER_PORT = 18080
VISION_MASTER_TIMEOUT = 30
VISION_MASTER_BLOCK_TRIGGER_TEXT = "ActionBlock"
VISION_MASTER_TRAY_TRIGGER_TEXT = "ActionTray"
HIK_CAMERA_CAPTURE_SCRIPT = os.path.join(PROJECT_DIR, "hik_mvs_capture.py")
DEFAULT_HIK_CAMERA_CAPTURE_CMD = (
    f'"{sys.executable}" "{HIK_CAMERA_CAPTURE_SCRIPT}" --camera-ip {HIK_CAMERA_IP} --output {{image_quoted}}'
)
HIK_CAMERA_CAPTURE_CMD = os.getenv("HIK_CAMERA_CAPTURE_CMD", DEFAULT_HIK_CAMERA_CAPTURE_CMD).strip()
HIK_CAMERA_CAPTURE_TIMEOUT = 120
VISION_MASTER_OCR_CMD = os.getenv("VISION_MASTER_OCR_CMD", "").strip()
VISION_MASTER_CAPTURE_OCR_CMD = os.getenv("VISION_MASTER_CAPTURE_OCR_CMD", "").strip()
VISION_MASTER_TIMEOUT = 120

VALID_COLORS = ["红色", "橙色", "黄色", "绿色", "蓝色", "紫色"]
COLOR_ALIAS = {
    "红": "红色",
    "红色": "红色",
    "橙": "橙色",
    "橙色": "橙色",
    "黄": "黄色",
    "黄色": "黄色",
    "绿": "绿色",
    "绿色": "绿色",
    "蓝": "蓝色",
    "蓝色": "蓝色",
    "紫": "紫色",
    "紫色": "紫色",
}

# ==========================================
# 2. 核心系统初始化
# ==========================================
print("🔄 正在初始化系统组件...")


def get_model_path():
    """智能寻找资源目录，兼容 PyCharm 源码运行和 EXE 打包运行"""
    if getattr(sys, 'frozen', False):
        exe_dir = os.path.dirname(sys.executable)
        project_root = os.path.dirname(exe_dir)
    else:
        project_root = os.path.dirname(os.path.abspath(__file__))

    target_model_path = os.path.join(
        project_root,
        "resources",
        "modelscope",
        "hub",
        "models",
        "iic",
        "SenseVoiceSmall"
    )
    return target_model_path


model_dir = get_model_path()
if not os.path.exists(model_dir):
    print(f"\n❌ 严重错误: 找不到语音模型文件夹！")
    print(f"预期路径: {model_dir}")
    print("请确认你已将 modelscope 文件夹拷贝至项目根目录下的 resources 文件夹中。")
    sys.exit(1)

# [耳朵] 初始化 FunASR
asr_model = AutoModel(model=model_dir, trust_remote_code=True, device="cpu", disable_update=True)

# [大脑] 初始化 LM Studio 客户端
llm_client = OpenAI(base_url="http://localhost:1234/v1", api_key="lm-studio")
LLM_MODEL = os.getenv("LM_STUDIO_MODEL", "qwen2.5").strip() or "qwen2.5"

# 🤖 [身体] V8.1 机器人实联初始化
print(f"⏳ 正在握手 AUBO ARCS 控制器 ({ROBOT_IP}:{ROBOT_PORT})...")  # noqa: spelling
rpc = RpcClient()
rpc.setRequestTimeout(ROBOT_REQUEST_TIMEOUT_MS)
robot_interface = None

try:
    rpc.connect(ROBOT_IP, ROBOT_PORT)
    try:
        rpc.login(ROBOT_USER, ROBOT_PASSWORD)  # 即使报错也会静默通过  # noqa: spelling
    except rpc.LoginError:
        pass

    robot_interface = rpc.getRobotInterface(ROBOT_NAME)

    # 移除了 setRobotModeToReal，因为 ARCS 仿真器默认就绪
    print(f"✅ 物理链路建立完毕，机械臂实例 [{ROBOT_NAME}] 已就绪。")
except Exception as e:
    print(f"❌ 机械臂通信初始化失败: {e} (将以无臂模式运行)")

print("✅ 所有系统组件初始化完成！\n")


# ==========================================
# 3. 核心功能组件库
# ==========================================
TTS_LOCK = threading.Lock()


def speak(text, rate=TTS_RATE_NORMAL):
    """嘴巴：局部唤醒，将文本转换为语音播报，避免通道冲突"""
    print(f"🔊 正在播报: {text}")
    try:
        with TTS_LOCK:
            engine = pyttsx3.init()
            engine.setProperty('rate', rate)

            # 显式声明 voices 是一个列表，消除 PyCharm 的类型警告
            voices: list = engine.getProperty('voices')
            for voice in voices:
                if 'zh-cn' in voice.id.lower() or 'chinese' in voice.name.lower():
                    engine.setProperty('voice', voice.id)
                    break

            engine.say(text)
            engine.runAndWait()
    except Exception as exc:
        print(f"❌ 语音播报组件发生异常: {exc}")


def speak_async(text, rate=TTS_RATE_NORMAL):
    """后台语音播报：不阻塞机器人运动下发，用于竞赛搬运流程抢时间。"""
    thread = threading.Thread(target=speak, args=(text, rate), daemon=True)
    thread.start()
    return thread


def get_rms(block):
    """计算音频块的音量(RMS)，用于 VAD 判断"""
    count = len(block) / 2
    fmt = "%dh" % count
    shorts = struct.unpack(fmt, block)
    sum_squares = 0.0
    for sample in shorts:
        n = sample / 32768.0
        sum_squares += n * n
    return math.sqrt(sum_squares / count) * 32768


def record_audio(output_file=TEMP_WAV_FILE, min_duration=0, max_duration=MAX_DURATION):
    """VAD 智能录音：检测到人声开始录音，连续静音自动停止"""
    p = pyaudio.PyAudio()
    stream = p.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, frames_per_buffer=CHUNK)

    # 👇 [修复1：强行排空系统的音频残留缓冲区]
    # 丢弃刚打开麦克风时的前 0.5 秒数据，防止吃进上一轮播报的尾音或键盘声
    for _ in range(int(RATE / CHUNK * 0.5)):
        stream.read(CHUNK, exception_on_overflow=False)
    # 👆 -------------------------------------------

    frames = []
    silent_chunks = 0
    total_chunks = 0
    limit_chunks = int(SILENCE_LIMIT * RATE / CHUNK)
    min_chunks = int(min_duration * RATE / CHUNK)
    max_chunks = int(max_duration * RATE / CHUNK)
    has_started = False

    try:
        while True:
            # 增加 exception_on_overflow=False，防止音频缓冲区溢出卡死，并允许信号打断
            data = stream.read(CHUNK, exception_on_overflow=False)
            frames.append(data)
            total_chunks += 1
            rms = get_rms(data)

            if rms > THRESHOLD:
                if not has_started:
                    has_started = True
                silent_chunks = 0
            else:
                if has_started:
                    silent_chunks += 1

            can_stop_on_silence = total_chunks >= min_chunks
            if (has_started and can_stop_on_silence and silent_chunks > limit_chunks) or (total_chunks > max_chunks):
                break

    except KeyboardInterrupt:
        # 如果在录音阻塞时按下了 Ctrl+C，在此处安全释放麦克风
        stream.stop_stream()
        stream.close()
        p.terminate()
        # 将中断信号继续向上抛给主程序的 try...except 捕捉
        raise

        # 正常结束录音时的清理
    stream.stop_stream()
    stream.close()
    p.terminate()

    wf = wave.open(output_file, 'wb')
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(p.get_sample_size(FORMAT))
    wf.setframerate(RATE)
    wf.writeframes(b''.join(frames))
    wf.close()
    return output_file


def recognize_audio(wav_file_path):
    """耳朵部分 B：语音转文字 + 工业热词纠错"""
    try:
        res = asr_model.generate(input=wav_file_path, cache={}, language="zh", use_itn=True)
        clean_text = re.sub(r'<\|.*?\|>', '', res[0]["text"]).strip()

        # ASR 工业语音纠错逻辑
        original_text = clean_text
        for wrong, right in ASR_CORRECTION_DICT.items():
            if wrong in clean_text:
                clean_text = clean_text.replace(wrong, right)

        if original_text != clean_text:
            print(f"🔧 [语音纠错] [{original_text}] -> [{clean_text}]")

        return clean_text
    except Exception as exc:
        # 使用 exc 变量打印具体的错误原因
        print(f"⚠️ [底层警告] FunASR 识别抛出异常: {exc}")
        return ""


def normalize_confirm_text(text):
    """清理确认指令文本，降低标点和空格对确认识别的影响"""
    return re.sub(r'[，。！？、,.!?？\s]', '', text or '').strip()


def is_voice_move_confirmed(text):
    """识别机器人运动前的二次语音确认，只接受明确的确认移动类指令"""
    clean_text = normalize_confirm_text(text)
    if not clean_text:
        return False

    cancel_words = ["取消", "不要", "停止", "不确认", "别动", "不要动", "取消移动"]
    if any(word in clean_text for word in cancel_words):
        return False

    confirm_words = ["确认移动", "确认执行", "确认运动", "开始移动", "可以移动", "确认"]
    if any(word in clean_text for word in confirm_words):
        return True

    return difflib.SequenceMatcher(None, clean_text, "确认移动").ratio() >= 0.75


def is_voice_transport_confirmed(text):
    """识别任务卡整套搬运前的语音确认；确认阶段允许“搬运”作为确认口令。"""
    clean_text = normalize_confirm_text(text)
    if not clean_text:
        return False

    cancel_words = ["取消", "不要", "停止", "不确认", "别搬", "不要搬", "取消搬运"]
    if any(word in clean_text for word in cancel_words):
        return False

    confirm_words = ["确认搬运", "开始搬运", "可以搬运", "执行搬运", "确认执行搬运", "搬运"]
    if any(word in clean_text for word in confirm_words):
        return True

    return difflib.SequenceMatcher(None, clean_text, "确认搬运").ratio() >= 0.75


def is_voice_transport_cancelled(text):
    """识别任务卡搬运确认阶段的明确取消指令"""
    clean_text = normalize_confirm_text(text)
    if not clean_text:
        return False
    cancel_words = ["取消", "不要", "停止", "不确认", "别搬", "不要搬", "取消搬运", "退出搬运"]
    return any(word in clean_text for word in cancel_words)


def wait_voice_move_confirmation(real_coordinates, action, target):
    """机器人运动前等待一次语音确认，替代 v10 的键盘 e 确认"""
    print(f"👉 确认发送坐标 {real_coordinates} 给机器人吗？请说“确认移动”执行，其他内容取消。")
    speak(f"准备执行{action}，目标{target}。请说确认移动，确认后机器人开始运动。")

    confirm_file = record_audio("temp_confirm.wav")
    confirm_text = recognize_audio(confirm_file)
    if os.path.exists(confirm_file):
        os.remove(confirm_file)

    print(f"🎧 [语音确认] 识别到：{confirm_text}")
    return is_voice_move_confirmed(confirm_text)


def wait_voice_transport_confirmation():
    print("👉 请确认搬运方块")
    speak("请确认搬运方块")

    while True:
        confirm_file = record_audio("temp_transport_confirm.wav")
        confirm_text = recognize_audio(confirm_file)
        if os.path.exists(confirm_file):
            os.remove(confirm_file)

        print(f"🎧 [搬运确认] 识别到：{confirm_text}")
        if is_voice_transport_confirmed(confirm_text):
            return True
        if is_voice_transport_cancelled(confirm_text):
            return False

        print("⏳ 未听到“确认搬运”，继续等待。若要取消，请说“取消搬运”。")


def normalize_robot_target_pose(real_coordinates):
    """校验机器人六维目标位姿，XYZ 使用 mm，RX/RY/RZ 使用 rad。"""
    if not isinstance(real_coordinates, (list, tuple)) or len(real_coordinates) != 6:
        raise ValueError(f"机器人目标位姿必须是 [x, y, z, rx, ry, rz] 六个值: {real_coordinates}")

    pose = [float(value) for value in real_coordinates]
    if not all(math.isfinite(value) for value in pose):
        raise ValueError(f"机器人目标位姿包含无效数值: {real_coordinates}")
    return pose


def format_pose_mm_rad(pose):
    """格式化 pyaubo_sdk 返回的位姿：XYZ 从 m 转换为 mm，RX/RY/RZ 保持 rad。"""
    return "[%.2f, %.2f, %.2f, %.5f, %.5f, %.5f]" % (
        pose[0] * 1000.0,
        pose[1] * 1000.0,
        pose[2] * 1000.0,
        pose[3],
        pose[4],
        pose[5],
    )


def calculate_position_error_mm(current_pose, target_pose):
    dx = (current_pose[0] - target_pose[0]) * 1000.0
    dy = (current_pose[1] - target_pose[1]) * 1000.0
    dz = (current_pose[2] - target_pose[2]) * 1000.0
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def calculate_orientation_error_rad(current_pose, target_pose):
    drx = current_pose[3] - target_pose[3]
    dry = current_pose[4] - target_pose[4]
    drz = current_pose[5] - target_pose[5]
    return math.sqrt(drx * drx + dry * dry + drz * drz)


def get_action_label(action):
    if action == "pick":
        return "抓取"
    if action == "place":
        return "放置"
    if action == "move":
        return "移动"
    return str(action)


def format_step_prefix(step):
    if step is None:
        return ""
    return f"第 {step} 步"


def format_motion_label(action, target, step=None):
    step_prefix = format_step_prefix(step)
    action_label = get_action_label(action)
    if step_prefix:
        return f"{step_prefix} {action_label} -> {target}"
    return f"{action_label} -> {target}"


def wait_until_robot_arrives(target_pose, action, target, step=None):
    start_time = time.monotonic()
    last_report_time = -1.0
    motion_label = format_motion_label(action, target, step)

    while True:
        current_pose = robot_interface.getRobotState().getTcpPose()
        error_mm = calculate_position_error_mm(current_pose, target_pose)
        orientation_error_rad = calculate_orientation_error_rad(current_pose, target_pose)
        elapsed = time.monotonic() - start_time

        if last_report_time < 0 or elapsed - last_report_time >= 1.0:
            print(
                "📏 等待机器人到位 [%s]，当前位姿 %s，目标位姿 %s，位置误差 %.2f mm，姿态误差 %.5f rad"
                % (motion_label, format_pose_mm_rad(current_pose), format_pose_mm_rad(target_pose), error_mm, orientation_error_rad)
            )
            last_report_time = elapsed

        if error_mm <= ROBOT_POSITION_TOLERANCE and orientation_error_rad <= ROBOT_ORIENTATION_TOLERANCE:
            print(
                "✅ 机器人到位 [%s]，最终位姿 %s，目标位姿 %s，位置误差 %.2f mm，姿态误差 %.5f rad"
                % (motion_label, format_pose_mm_rad(current_pose), format_pose_mm_rad(target_pose), error_mm, orientation_error_rad)
            )
            return True

        if elapsed >= ROBOT_WAIT_TIMEOUT:
            print(
                "⏱️ 等待机器人到位超时 [%s]，当前位姿 %s，目标位姿 %s，位置误差 %.2f mm，姿态误差 %.5f rad"
                % (motion_label, format_pose_mm_rad(current_pose), format_pose_mm_rad(target_pose), error_mm, orientation_error_rad)
            )
            return False

        time.sleep(0.2)


def execute_robot_move(real_coordinates, action, target, step=None):
    try:
        target_pose_mm_rad = normalize_robot_target_pose(real_coordinates)
    except (TypeError, ValueError) as pose_err:
        print(f"❌ 运动目标位姿格式错误: {pose_err}")
        speak("机器人目标位姿格式错误，请检查坐标数据。")
        return False

    if not robot_interface:
        print(f"⚠️ 物理层未连接，仿真跳过运动执行 [{format_motion_label(action, target, step)}]: {target_pose_mm_rad}")
        return True

    try:
        print(f"⚙️ 正在向 ARCS 下发 MoveL 运动指令 [{format_motion_label(action, target, step)}]...")
        tx, ty, tz, rx, ry, rz = target_pose_mm_rad
        target_pose = [tx / 1000.0, ty / 1000.0, tz / 1000.0, rx, ry, rz]

        robot_interface.getMotionControl().moveLine(
            target_pose,
            ROBOT_SPEED,
            ROBOT_ACCELERATION,
            0.0,
            True,
        )
        print(f"🎯 运动指令已下发 [{format_motion_label(action, target, step)}]: 目标位姿 {target_pose_mm_rad}")
        return wait_until_robot_arrives(target_pose, action, target, step)
    except Exception as move_err:
        print(f"❌ 运动执行被拦截: {move_err}")
        speak("机械臂运动异常，请检查控制台输出。")
        return False


def move_robot_to_task_card2_position():
    task_card2_position = TASK_CARD2_VIEW_POSITION
    print(f"📍 准备移动机器人到任务卡2位置: {task_card2_position}")
    speak_async("正在移动机器人到任务卡2位置")
    return execute_robot_move(task_card2_position, "move", "任务卡2位置")


def move_robot_to_block_view_position():
    print(f"📍 准备移动机器人到方块识别拍照位: {BLOCK_VIEW_POSITION}")
    speak_async("正在移动机器人到方块识别位置")
    return execute_robot_move(BLOCK_VIEW_POSITION, "move", "方块识别拍照位")


def move_robot_to_tray_view_position():
    print(f"📍 准备移动机器人到托盘识别拍照位: {TRAY_VIEW_POSITION}")
    speak_async("正在移动机器人到托盘识别位置")
    return execute_robot_move(TRAY_VIEW_POSITION, "move", "托盘识别拍照位")


def project_file_path(file_name):
    return os.path.join(PROJECT_DIR, file_name)


def current_timestamp_text():
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def current_timestamp_file():
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def get_all_vision_target_types():
    return {
        "红色的方块", "橙色的方块", "黄色的方块", "绿色的方块", "蓝色的方块", "紫色的方块",
        "红色的托盘", "橙色的托盘", "黄色的托盘", "绿色的托盘", "蓝色的托盘", "紫色的托盘",
    }


def get_vision_target_types(object_kind):
    return {f"{color}的{object_kind}" for color in VALID_COLORS}


def normalize_vision_item_type(item, object_kind=None):
    """兼容 VisionMaster 返回完整类型或仅返回颜色，按拍照阶段补充“方块/托盘”。"""
    item_type = item.get("type") or item.get("color")
    if not isinstance(item_type, str):
        return item_type

    item_type = item_type.strip()
    if item_type in get_all_vision_target_types() or not object_kind:
        return item_type

    normalized_color = COLOR_ALIAS.get(item_type)
    if normalized_color:
        return f"{normalized_color}的{object_kind}"
    return item_type


def normalize_vision_data(data, required_types=None, object_kind=None):
    """校验 VisionMaster 坐标数据并转换成 GLOBAL_VISION_DATA 标准格式。"""
    required = set(required_types or get_all_vision_target_types())

    if not isinstance(data, list):
        raise ValueError("VisionMaster 坐标文件必须是 JSON 数组。")

    normalized = []
    seen = set()
    for index, item in enumerate(data, start=1):
        if not isinstance(item, dict):
            raise ValueError(f"VisionMaster 第 {index} 项不是对象: {item}")

        item_type = normalize_vision_item_type(item, object_kind)
        pos = item.get("pos")

        if item_type not in required:
            raise ValueError(f"VisionMaster 输出了未知目标: {item_type}")
        if item_type in seen:
            raise ValueError(f"VisionMaster 输出重复目标: {item_type}")
        if not isinstance(pos, list) or len(pos) != 6:
            raise ValueError(f"{item_type} 位姿格式错误，必须为 [x, y, z, rx, ry, rz]: {pos}")

        normalized.append({
            "id": index,
            "type": item_type,
            "pos": normalize_robot_target_pose(pos),
        })
        seen.add(item_type)

    missing = required - seen
    if missing:
        raise ValueError(f"VisionMaster 缺少目标: {sorted(missing)}")

    return normalized


def apply_global_vision_data(normalized_data, source_name):
    """覆盖 GLOBAL_VISION_DATA 并打印坐标明细。"""
    global GLOBAL_VISION_DATA
    GLOBAL_VISION_DATA = normalized_data
    print(f"✅ 已从 {source_name} 更新 GLOBAL_VISION_DATA")
    for item in GLOBAL_VISION_DATA:
        print(f"  {item['type']}: {item['pos']}")


def update_global_vision_data_from_json(json_path):
    """读取坐标 JSON，校验后覆盖 GLOBAL_VISION_DATA。"""
    with open(json_path, "r", encoding="utf-8") as file_obj:
        data = json.load(file_obj)
    apply_global_vision_data(normalize_vision_data(data), f"坐标文件: {json_path}")


def save_real_vision_data_to_json(normalized_data):
    """保存网络收到的真实视觉坐标，便于追踪和复用。"""
    real_json_path = project_file_path(VISION_DATA_REAL_JSON)
    payload = [
        {
            "type": item["type"],
            "pos": item["pos"],
        }
        for item in normalized_data
    ]
    with open(real_json_path, "w", encoding="utf-8") as file_obj:
        json.dump(payload, file_obj, ensure_ascii=False, indent=2)
    print(f"💾 已保存 VisionMaster 真实坐标到: {real_json_path}")


def extract_json_from_network_payload(payload):
    """从 VisionMaster TCP 响应中提取 JSON。"""
    text = payload.decode("utf-8", errors="replace").strip("\ufeff\r\n \t")
    if not text:
        raise ValueError("网络数据为空。")
    json_match = re.search(r'(\[.*]|\{.*})', text, re.DOTALL)
    if not json_match:
        raise ValueError(f"VisionMaster 返回内容不包含 JSON: {text}")
    return json.loads(json_match.group(1))


def request_vision_data_from_network(trigger_text, required_types, object_kind, scene_name):
    """连接 VisionMaster，触发一次拍照，在同一 TCP 连接中读取当前区域的坐标 JSON。"""
    print(
        f"🌐 准备触发 VisionMaster 拍摄{scene_name}。\n"
        f"   Python 连接 VisionMaster: {VISION_MASTER_HOST}:{VISION_MASTER_PORT}\n"
        f"   发送触发字符串: {trigger_text}"
    )
    print(f"⏳ VisionMaster 响应超时: {VISION_MASTER_TIMEOUT} 秒")

    try:
        with socket.create_connection((VISION_MASTER_HOST, VISION_MASTER_PORT), timeout=VISION_MASTER_TIMEOUT) as conn:
            conn.settimeout(VISION_MASTER_TIMEOUT)
            conn.sendall((trigger_text + "\n").encode("utf-8"))
            chunks = []
            while True:
                try:
                    chunk = conn.recv(65536)
                except socket.timeout:
                    break
                if not chunk:
                    break
                chunks.append(chunk)
                if b"\n" in chunk:
                    break
    except OSError as exc:
        raise RuntimeError(f"连接 VisionMaster 失败: {exc}") from exc

    if not chunks:
        raise RuntimeError(f"VisionMaster 未返回{scene_name}坐标 JSON。")

    data = extract_json_from_network_payload(b"".join(chunks))
    normalized_data = normalize_vision_data(data, required_types=required_types, object_kind=object_kind)
    print(f"✅ VisionMaster 已返回{scene_name} 6 项位姿")
    for item in normalized_data:
        print(f"  {item['type']}: {item['pos']}")
    return normalized_data


def update_global_vision_data_before_transport():
    """优先本地模拟坐标；实机模式下依次拍摄方块区和托盘区，再合并真实坐标。"""
    sim_json_path = project_file_path(VISION_DATA_SIM_JSON)
    if os.path.exists(sim_json_path):
        print(f"🧪 检测到模拟坐标文件，使用模拟坐标: {sim_json_path}")
        update_global_vision_data_from_json(sim_json_path)
        return True

    print("📷 未检测到模拟坐标文件，当前应连接实际机器人和相机。")
    if not move_robot_to_block_view_position():
        raise RuntimeError("移动到方块识别拍照位失败。")

    block_data = request_vision_data_from_network(
        VISION_MASTER_BLOCK_TRIGGER_TEXT,
        get_vision_target_types("方块"),
        "方块",
        "方块区域",
    )

    if not move_robot_to_tray_view_position():
        raise RuntimeError("移动到托盘识别拍照位失败。")

    tray_data = request_vision_data_from_network(
        VISION_MASTER_TRAY_TRIGGER_TEXT,
        get_vision_target_types("托盘"),
        "托盘",
        "托盘区域",
    )

    normalized_data = normalize_vision_data(block_data + tray_data)
    source_name = f"VisionMaster 两次 TCP 响应 {VISION_MASTER_HOST}:{VISION_MASTER_PORT}"
    save_real_vision_data_to_json(normalized_data)
    apply_global_vision_data(normalized_data, source_name)
    return True


def extract_ocr_text(command_output):
    """兼容 Vision Master 输出纯文本或 JSON OCR 结果"""
    output = (command_output or "").strip()
    if not output:
        return ""

    def collect_text(value, texts):
        if isinstance(value, dict):
            for key in ["text", "Text", "ocr", "OCR", "result", "Result", "content", "Content", "value", "Value"]:
                item = value.get(key)
                if isinstance(item, str) and item.strip():
                    texts.append(item.strip())
            for item in value.values():
                collect_text(item, texts)
        elif isinstance(value, list):
            for item in value:
                collect_text(item, texts)

    try:
        payload = json.loads(output)
        text_parts = []
        collect_text(payload, text_parts)
        if text_parts:
            deduped_parts = []
            for part in text_parts:
                if part not in deduped_parts:
                    deduped_parts.append(part)
            return "".join(deduped_parts)
    except json.JSONDecodeError:
        pass

    return output


def run_vision_master_command(command_template, image_path=None):
    """调用外部 Vision Master OCR 命令，命令需把 OCR 文本输出到 stdout"""
    if not command_template:
        raise RuntimeError("未配置 Vision Master 外部命令。")

    command = command_template
    if image_path:
        quoted_image_path = '"' + image_path + '"'
        if "{image_quoted}" in command:
            command = command.replace("{image_quoted}", quoted_image_path)
        elif "{image}" in command:
            command = command.replace("{image}", image_path)
        else:
            command = f"{command} {quoted_image_path}"

    print(f"🧾 [Vision Master] 正在执行 OCR 命令: {command}")
    completed = subprocess.run(
        command,
        shell=True,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        timeout=VISION_MASTER_TIMEOUT,
        check=False,
    )

    if completed.returncode != 0:
        detail = completed.stderr.strip() or completed.stdout.strip()
        raise RuntimeError(f"Vision Master OCR 命令执行失败: {detail}")

    ocr_text = extract_ocr_text(completed.stdout.strip() or completed.stderr.strip())
    if not ocr_text:
        raise RuntimeError("Vision Master 未返回有效 OCR 文本。")
    return ocr_text


def run_hik_camera_capture_command(command_template, output_path):
    """调用外部拍照命令获取任务卡图片；命令可使用 {image}/{output} 占位符。"""
    if not command_template:
        raise RuntimeError("未配置 HIK_CAMERA_CAPTURE_CMD，无法调用海康相机拍照。")

    quoted_output_path = '"' + output_path + '"'
    command = command_template
    if "{image_quoted}" in command:
        command = command.replace("{image_quoted}", quoted_output_path)
    elif "{output_quoted}" in command:
        command = command.replace("{output_quoted}", quoted_output_path)
    elif "{image}" in command:
        command = command.replace("{image}", output_path)
    elif "{output}" in command:
        command = command.replace("{output}", output_path)
    else:
        command = f"{command} {quoted_output_path}"

    print(f"📷 [海康相机] 正在执行拍照命令: {command}")
    completed = subprocess.run(
        command,
        shell=True,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        timeout=HIK_CAMERA_CAPTURE_TIMEOUT,
        check=False,
    )

    if completed.returncode != 0:
        detail = completed.stderr.strip() or completed.stdout.strip()
        raise RuntimeError(f"海康相机拍照命令执行失败: {detail}")

    candidate_path = (completed.stdout or "").strip().strip('"')
    if candidate_path and os.path.exists(candidate_path):
        return candidate_path

    if os.path.exists(output_path):
        return output_path

    raise RuntimeError(f"海康相机拍照命令执行完成，但未生成图片: {output_path}")


def normalize_task_card_ocr_lines(text_lines):
    """合并 PaddleOCR 多行结果，保留中文任务指令的自然顺序。"""
    cleaned_lines = []
    for line in text_lines:
        clean_line = re.sub(r"\s+", "", str(line or "").strip())
        if clean_line:
            cleaned_lines.append(clean_line)
    return "".join(cleaned_lines)


def read_task_card2_text():
    """读取任务卡2：优先本地图片 PaddleOCR，没有图片时调用海康相机拍照后 OCR。"""
    image_path = project_file_path(TASK_CARD2_IMAGE)
    ocr_text_path = project_file_path(TASK_CARD2_OCR_TEXT)

    if os.path.exists(image_path):
        print(f"🧾 [任务卡2] 检测到本地图片，使用 PaddleOCR 识别: {image_path}")
        ocr_image_path = image_path
    else:
        capture_path = project_file_path(TASK_CARD_CAPTURE_IMAGE)
        print("🧾 [任务卡2] 未找到本地图片，准备调用海康相机拍照。")
        ocr_image_path = run_hik_camera_capture_command(HIK_CAMERA_CAPTURE_CMD, capture_path)

    text_lines = recognize_card_text(ocr_image_path)
    task_text = normalize_task_card_ocr_lines(text_lines)
    if not task_text:
        raise RuntimeError("PaddleOCR 未返回有效任务卡文本。")

    print(f"🧾 [PaddleOCR] 原始行结果: {text_lines}")
    print(f"🧾 [PaddleOCR] 合并文本: {task_text}")
    with open(ocr_text_path, "w", encoding="utf-8") as file_obj:
        file_obj.write(task_text)
    return task_text


def is_task_card2_command(text):
    clean_text = re.sub(r'[，。！？、,.!?？\s]', '', text or "")
    keywords = ["任务卡2", "任务卡二", "第二张任务卡", "读取任务卡", "识别任务卡", "执行任务卡", "开始装配"]
    return any(keyword in clean_text for keyword in keywords)


def ask_brain_task_card(task_text):
    """让 Qwen 2.5 7B Instruct 只负责解析 OCR 文本中的装配顺序"""
    print("🧠 大脑正在解析任务卡2的六步装配顺序...")
    system_prompt = """你是工业机器人任务卡解析器，不是问答助手。

【任务】
从 OCR 文本中解析装配顺序。每一步都表示“把某颜色方块放到某颜色托盘上”。

【硬性要求】
1. 只输出 JSON 数组，不要输出解释、Markdown 或多余文本。
2. JSON 数组必须恰好 6 个元素，顺序必须严格保持 OCR 文本中的先后顺序。
3. 每个元素字段固定为：step、block_color、tray_color。
4. block_color 和 tray_color 只能是：红色、橙色、黄色、绿色、蓝色、紫色。
5. 不允许合并、跳过、改色、调换顺序。
6. 不允许追问，不允许说无法判断；OCR 文本中出现的“先、再、接着、然后、最后”就是顺序标记。
7. 不要解释 JSON 是什么，不要举例，不要输出英文说明。

【唯一允许的输出格式】
直接输出一个 JSON 数组。数组元素只能使用这种对象结构：
{"step": 1, "block_color": "红色", "tray_color": "黄色"}
其中 step 从 1 到 6 递增，block_color/tray_color 必须来自 OCR 文本里的对应颜色。
"""
    from openai.types.chat import (
        ChatCompletionSystemMessageParam,
        ChatCompletionUserMessageParam,
    )

    system_msg: ChatCompletionSystemMessageParam = {
        "role": "system",
        "content": system_prompt,
    }
    user_msg: ChatCompletionUserMessageParam = {
        "role": "user",
        "content": (
            "请解析下面 OCR 文本，严格按要求只输出 JSON 数组：\n"
            f"{task_text}"
        ),
    }
    completion = llm_client.chat.completions.create(
        model=LLM_MODEL,
        messages=[system_msg, user_msg],
        temperature=0.0,
        max_tokens=800,
    )
    return completion.choices[0].message.content.strip()


def normalize_color(color_text):
    clean_text = re.sub(r'[，。！？、,.!?？\s的方块托盘上放到把]', '', str(color_text or ""))
    for alias, color in sorted(COLOR_ALIAS.items(), key=lambda item: len(item[0]), reverse=True):
        if alias in clean_text:
            return color
    return None


def extract_json_payload(text):
    json_match = re.search(r'(\[.*]|\{.*})', text or "", re.DOTALL)
    if not json_match:
        raise ValueError("大脑未返回可解析的 JSON 结构。")
    payload = json.loads(json_match.group(1))
    if isinstance(payload, dict):
        payload = payload.get("steps") or payload.get("actions") or [payload]
    return payload


def validate_assembly_steps(steps):
    if not isinstance(steps, list):
        raise ValueError("任务卡解析结果不是数组。")

    normalized_steps = []
    for index, step in enumerate(steps, start=1):
        if not isinstance(step, dict):
            raise ValueError("任务卡步骤格式错误。")

        block_color = normalize_color(
            step.get("block_color") or step.get("block") or step.get("cube_color") or step.get("方块颜色")
        )
        tray_color = normalize_color(
            step.get("tray_color") or step.get("tray") or step.get("托盘颜色") or step.get("target")
        )
        if block_color not in VALID_COLORS or tray_color not in VALID_COLORS:
            raise ValueError(f"第 {index} 步颜色无效: {step}")

        normalized_steps.append({
            "step": index,
            "block_color": block_color,
            "tray_color": tray_color,
        })

    if len(normalized_steps) != 6:
        raise ValueError(f"任务卡必须解析出 6 步，当前为 {len(normalized_steps)} 步。")

    block_colors = [step["block_color"] for step in normalized_steps]
    tray_colors = [step["tray_color"] for step in normalized_steps]
    if set(block_colors) != set(VALID_COLORS) or len(set(block_colors)) != 6:
        raise ValueError(f"方块颜色必须覆盖且仅覆盖六色一次，当前为: {block_colors}")
    if set(tray_colors) != set(VALID_COLORS) or len(set(tray_colors)) != 6:
        raise ValueError(f"托盘颜色必须覆盖且仅覆盖六色一次，当前为: {tray_colors}")

    return normalized_steps


def parse_task_card_by_rules(task_text):
    """LLM 输出异常时，用规则兜底提取“X色方块 -> Y色托盘”的六组顺序"""
    clean_text = re.sub(r'\s+', '', task_text or "")
    color_pattern = "(红色|橙色|黄色|绿色|蓝色|紫色|红|橙|黄|绿|蓝|紫)"
    pair_pattern = re.compile(color_pattern + r"(?:的)?方块.*?" + color_pattern + r"(?:的)?托盘")

    steps = []
    for index, match in enumerate(pair_pattern.finditer(clean_text), start=1):
        steps.append({
            "step": index,
            "block_color": normalize_color(match.group(1)),
            "tray_color": normalize_color(match.group(2)),
        })
    return validate_assembly_steps(steps)


def format_steps_for_speech(steps):
    return "，".join(
        f"第{step['step']}步，{step['block_color']}方块到{step['tray_color']}托盘"
        for step in steps
    )


def save_task_card_llm_parse_log(task_text, raw_reply, steps=None, parse_source="", error_text=""):
    """保存大模型解析过程，满足竞赛对文本留痕和时间戳的要求。"""
    log_dir = project_file_path(TASK_CARD_LLM_LOG_DIR)
    os.makedirs(log_dir, exist_ok=True)
    log_path = os.path.join(log_dir, f"task_card2_llm_parse_{current_timestamp_file()}.txt")

    lines = [
        f"timestamp: {current_timestamp_text()}",
        f"model: {LLM_MODEL}",
        f"parse_source: {parse_source}",
        "",
        "[OCR_TEXT]",
        task_text or "",
        "",
        "[QWEN_RAW_OUTPUT]",
        raw_reply or "",
    ]
    if steps is not None:
        lines.extend([
            "",
            "[VALIDATED_STEPS]",
            json.dumps(steps, ensure_ascii=False, indent=2),
        ])
    if error_text:
        lines.extend([
            "",
            "[ERROR]",
            error_text,
        ])

    with open(log_path, "w", encoding="utf-8") as file_obj:
        file_obj.write("\n".join(lines))
    print(f"📝 大模型解析过程已保存: {log_path}")


def build_task_card2_actions(task_text):
    #print(f"🧾 [任务卡2 OCR] {task_text}")
    raw_reply = ""
    try:
        raw_reply = ask_brain_task_card(task_text)
        print(f"🤖 任务卡解析原始输出: {raw_reply}")
        steps = validate_assembly_steps(extract_json_payload(raw_reply))
        parse_source = "Qwen 2.5 校验通过"
        parse_error = ""
    except Exception as exc:
        print(f"⚠️ Qwen 解析任务卡失败，改用规则兜底解析: {exc}")
        parse_error = f"Qwen parse failed: {exc}"
        try:
            steps = parse_task_card_by_rules(task_text)
            parse_source = "Qwen 2.5 校验失败，规则兜底通过"
        except Exception as fallback_exc:
            save_task_card_llm_parse_log(
                task_text,
                raw_reply,
                steps=None,
                parse_source="任务卡识别失败",
                error_text=f"{parse_error}\nRule fallback failed: {fallback_exc}",
            )
            raise RuntimeError("任务卡识别失败,请确认任务卡位置") from fallback_exc

    save_task_card_llm_parse_log(
        task_text,
        raw_reply,
        steps=steps,
        parse_source=parse_source,
        error_text=parse_error,
    )

    print("📋 [装配计划] 已锁定 6 步顺序:")
    for step in steps:
        print(f"  第 {step['step']} 步: {step['block_color']}方块 -> {step['tray_color']}托盘")

    actions = []
    for step in steps:
        step_summary = f"第 {step['step']} 步，{step['block_color']}方块到{step['tray_color']}托盘"
        actions.append({
            "action": "pick",
            "target": f"{step['block_color']}的方块",
            "step": step["step"],
            "step_summary": step_summary,
        })
        actions.append({
            "action": "place",
            "target": f"{step['tray_color']}的托盘",
            "step": step["step"],
            "step_summary": step_summary,
        })
    return actions


def ask_brain(user_text):  # pylint: disable=redefined-outer-name
    """大脑：彻底剥夺视觉数据，仅作纯粹的 NLP 语义翻译机"""
    print("🧠 大脑正在进行纯语义解析（盲测模式）...")

    # --- 彻底致盲版 Prompt：绝不给它看 GLOBAL_VISION_DATA ---
    system_prompt = """你是一个纯粹的自然语言翻译机。

    【你的唯一任务】
    提取用户原话中的“动作”和“物品名词”，将其转换为 JSON 数组。

    【绝对铁律 - 否则系统崩溃】
    1. 只能使用两种 action： "pick" 或 "place"。
    2. target 必须【一字不差】地提取用户原话中的物品，绝对不准联想、纠错或改变颜色！哪怕用户说“拿取红方块儿”，你也要老老实实输出 "target": "红方块儿"。
    3. 只输出 JSON 数组，严禁其他废话和 Markdown 标记。

    【示例】
    用户：“把黑色的手机放到白桌子上”
    输出：
    [
      {"action": "pick", "target": "黑色的手机"},
      {"action": "place", "target": "白桌子"}
    ]
    """
    from openai.types.chat import (
        ChatCompletionSystemMessageParam,
        ChatCompletionUserMessageParam,
    )

    try:
        system_msg: ChatCompletionSystemMessageParam = {
            "role": "system",
            "content": system_prompt,
        }
        user_msg: ChatCompletionUserMessageParam = {
            "role": "user",
            "content": user_text,
        }
        completion = llm_client.chat.completions.create(
            model=LLM_MODEL,
            messages=[system_msg, user_msg],
            temperature=0.0,
        )
        return completion.choices[0].message.content.strip()
    except Exception as exc:
        return json.dumps([{"action": "none", "reason": f"通信异常: {str(exc)}"}])


# ==========================================
# 4. 主流程：语音唤醒与全双工动作解析 (V10 更新)
# ==========================================
if __name__ == "__main__":
    speak("智能体v12已上线，请呼叫小具同学。")

    try:
        while True:
            # 🤖 V10 修改：移除手动敲击回车，改为持续监听环境音
            print("\n💤 监听中... (等待唤醒词: '小具同学')")
            print("🗣️ 可语音输入：小具同学，移动机器人到任务卡2位置；如果退出直接说：退出系统")

            # 1. 录音与听写 (VAD 智能过滤，只有人声才会触发录制)
            saved_file = record_audio(min_duration=MAIN_COMMAND_MIN_DURATION)
            spoken_text = recognize_audio(saved_file)

            # 👇 [修复2：白噪音与纯标点过滤]
            # 剔除所有的标点符号，看看还剩不剩汉字
            clean_check = re.sub(r'[，。！？、,!?\s]', '', spoken_text)
            if not clean_check:
                # 如果是空的（比如只听到一个 "。"），当作纯噪音静默丢弃
                continue

            print(f"🎧 [底层捕获] ASR 实际听到的是: 【{spoken_text}】")

            global_command_text = re.sub(r'[，。！？、,!?]', '', spoken_text).strip()
            if "退出系统" in global_command_text:
                print("\n🛑 [语音指令] 接收到退出系统指令。")
                speak("收到，退出系统")
                break

            # --- 🚀 V10 核心：唤醒词拦截逻辑 ---
            if "小具同学" not in spoken_text:
                if is_task_card2_command(global_command_text):
                    command_text = global_command_text
                    print(f"⚡ [任务卡指令] 直接识别到: 【 {command_text} 】")
                else:
                    # 没听到唤醒词，当作环境噪音或别人在聊天，直接忽略
                    continue
            else:
                print(f"⚡ [系统唤醒] 听到声音: 【 {spoken_text} 】")

                # 提取真实指令（剔除唤醒词本身，支持一句话连着说）
                command_text = spoken_text.replace("小具同学", "").strip()
                # 剔除常见的标点符号干扰
                command_text = re.sub(r'[，。！？、,!?]', '', command_text).strip()

                # 支持两轮对话模式（如果用户只喊了“小具同学”四个字）
                if len(command_text) < 2:
                    speak("我在，请指示！")
                    print("\n🎤 请开始下达装配指令...")
                    cmd_file = record_audio(min_duration=MAIN_COMMAND_MIN_DURATION)
                    command_text = recognize_audio(cmd_file)
                    command_text = re.sub(r'[，。！？、,!?]', '', command_text).strip()

                    if not command_text:
                        speak("未检测到有效指令，系统休眠。")
                        continue

            print(f"👤 有效装配指令: 【 {command_text} 】")
            # 全局退出已在唤醒词判断前处理；这里保留带唤醒词场景的二次保护。
            if "退出系统" in command_text:
                print("\n🛑 [语音指令] 接收到退出系统指令。")
                speak("收到，退出系统")
                break  # 直接跳出主循环，程序会优雅地结束并断开机械臂连接

            try:
                # 2. 大脑意图提取。任务卡2走 PaddleOCR，其余语音指令保持原 v11 流程。
                task_card2_flow = False
                if is_task_card2_command(command_text):
                    task_card2_flow = True
                    if not move_robot_to_task_card2_position():
                        speak("移动到任务卡2位置失败，已停止任务卡流程。")
                        continue
                    try:
                        task_card_text = read_task_card2_text()
                        print(f"🧾 任务卡2识别文本: {task_card_text}")
                        actions = build_task_card2_actions(task_card_text)
                    except Exception as task_card_exc:
                        print(f"❌ 任务卡识别失败: {task_card_exc}")
                        speak("任务卡识别失败,请确认任务卡位置")
                        continue

                    step_summaries = []
                    seen_steps_for_summary = set()
                    for action_item in actions:
                        step_no = action_item.get("step")
                        if step_no not in seen_steps_for_summary:
                            step_summaries.append(action_item.get("step_summary") or "")
                            seen_steps_for_summary.add(step_no)
                    command_review_text = "识别到装配任务：" + "，".join([text for text in step_summaries if text])
                    print(f"🗣️ [指令复述] {command_review_text}")
                    speak(command_review_text, rate=TTS_RATE_TASK_REVIEW)

                    update_global_vision_data_before_transport()
                    raw_reply = json.dumps(actions, ensure_ascii=False)
                    print(f"🤖 任务卡动作序列: {raw_reply}")
                    if not wait_voice_transport_confirmation():
                        print("🚫 [语音拦截] 未确认搬运，任务卡2搬运流程取消。")
                        speak("未确认搬运，任务已取消。")
                        continue
                else:
                    raw_reply = ask_brain(command_text)
                    print(f"🤖 大脑原始输出: {raw_reply}")

                    # 3. JSON 格式强制提取
                    actions = extract_json_payload(raw_reply)

                    if isinstance(actions, dict):
                        actions = [actions]

                # 4. --- 【核心：柔性物理级安全护栏】 ---
                announced_task_steps = set()
                for task in actions:
                    action = task.get("action")
                    target = task.get("target") or task.get("location")

                    if action == "none":
                        reason = task.get("reason", "未知原因")
                        print(f"ℹ️ [大脑返回空指令]: {reason}")
                        speak(f"无法执行任务。原因：{reason}")
                        continue

                        # 动作同义词强行纠偏
                    if action in ["put", "drop", "set", "放入", "放置"]:
                        action = "place"
                    elif action in ["get", "grab", "take", "拿起", "抓取", "拿取"]:
                        action = "pick"

                    if action == "alert":
                        reason = task.get("reason", "未知原因")
                        print(f"⚠️ [AI 主动中断] {reason}: 缺少 {target}")
                        speak(f"警报！{reason}，请人工检查。")
                        break

                    # --- 模糊泛化匹配与颜色绝对阻断 ---
                    is_valid = False
                    real_coordinates = None

                    valid_items = [item["type"] for item in GLOBAL_VISION_DATA]
                    matches = difflib.get_close_matches(target, valid_items, n=1, cutoff=0.4)

                    if matches:
                        matched_name = matches[0]

                        # 🛡️ 致命颜色防波堤
                        color_conflict = False
                        for color in ["红", "蓝", "黄", "绿", "紫", "橙", "黑", "白"]:
                            if color in target and color not in matched_name:
                                color_conflict = True
                                break

                        if color_conflict:
                            print(f"🛑 [系统强行拦截] 颜色冲突！提取的 '{target}' 无法安全映射到 '{matched_name}'")
                        else:
                            is_valid = True
                            print(f"✨ [柔性吸附] 成功将 '{target}' 泛化识别为真实目标 -> '{matched_name}'")
                            target = matched_name

                            # 找到对应的真实坐标
                            for item in GLOBAL_VISION_DATA:
                                if item["type"] == matched_name:
                                    real_coordinates = item["pos"]
                                    break

                    if not is_valid:
                        print(f"🛑 [系统强行拦截] 视野中不存在与 '{target}' 匹配的物品！")
                        speak(f"严重警报！指令被强行拦截，系统中未检测到 {target}。")
                        break

                    # ==========================================
                    # 🤖 底层真实位姿控制
                    # ==========================================
                    step = task.get("step")
                    step_prefix = format_step_prefix(step)
                    if step_prefix:
                        print(f"\n⚠️  [安全等待] {step_prefix}，准备执行：【{get_action_label(action)}】 -> 【{target}】")
                    else:
                        print(f"\n⚠️  [安全等待] 准备执行：【{action}】 -> 【{target}】")

                    if task_card2_flow or wait_voice_move_confirmation(real_coordinates, action, target):

                        if task_card2_flow:
                            if step not in announced_task_steps:
                                step_summary = task.get("step_summary") or format_motion_label(action, target, step)
                                print(f"🔊 [步骤播报] {step_summary}")
                                speak_async(step_summary)
                                announced_task_steps.add(step)
                        else:
                            if action == "pick":
                                msg = f"{step_prefix}，正在执行抓取，目标：{target}。" if step_prefix else f"正在执行抓取，目标：{target}。"
                            elif action == "place":
                                msg = f"{step_prefix}，正在执行放置，位置：{target}。" if step_prefix else f"正在执行放置，位置：{target}。"
                            else:
                                msg = f"{step_prefix}，正在执行 {action} 动作。" if step_prefix else f"正在执行 {action} 动作。"
                            speak(msg)

                        if not execute_robot_move(real_coordinates, action, target, step):
                            break

                    else:
                        print(f"🚫 [语音拦截] 已取消对 '{target}' 的操作。")
                        speak("操作已取消。")
                        break
                else:
                    if task_card2_flow:
                        print("✅ 任务卡2全部搬运动作已完成。")
                        speak("任务已完成")
                        speak("两个任务均已完成")
            except Exception as e:
                print(f"❌ 指令解析失败: {e}")
                speak("指令解析协议格式错误。")

            # 5. 清理缓存
            if os.path.exists(saved_file):
                os.remove(saved_file)

    except KeyboardInterrupt:
        # 🤖 V10 修改：通过 Ctrl+C 安全退出程序
        print("\n🛑 检测到退出信号 (Ctrl+C)")
        speak("系统正在关机，再见。")
