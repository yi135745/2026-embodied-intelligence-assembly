# v5版本相对于v4版本改进，机器人到位后不再等待学生额外说「拍摄照片」，从而缩短任务执行时间。
# 任务一，语音输入“小具同学”，控制机器人移动到指定位置。
# 如果Project1目录下有任务卡1.png图片文件，用大模型识别图片中物体
# 如果没有该文件，调用工业相机实时拍照任务卡一，识别卡片中的物体，并通过语音播报。
import argparse
import base64
import difflib
import io
import importlib
import json
import logging
import math
import mimetypes
import os
import re
import struct
import subprocess
import sys
import threading
import time
import warnings
import wave
import queue
from ctypes import POINTER, byref, cast, memset, sizeof
from datetime import datetime
from pathlib import Path
from socket import timeout as SOCKET_TIMEOUT
from typing import Any
from urllib import error, request


import pyaudio
from funasr import AutoModel

try:
    from pyaubo_sdk import RpcClient
except ImportError:
    RpcClient = None


# ==============================
# 1. 基础配置
# ==============================
# 录音参数：FunASR/SenseVoiceSmall 通常使用 16 kHz、单声道、16 位 PCM。
CHUNK = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000
TEMP_WAV_FILE = "temp_voice_command.wav"
ASR_READY_TIMEOUT_SECONDS = 60.0
ASR_LISTEN_TIMEOUT_SECONDS = 45.0

# VAD 录音参数：
# THRESHOLD 越大，越不容易被环境噪声触发；现场噪声大时可适当调高。
# SILENCE_LIMIT 表示检测到人声后，连续静音多少秒自动结束录音。
# MAX_DURATION 防止录音一直不结束。
THRESHOLD = 300
SILENCE_LIMIT = 2
MAX_DURATION = 15

# 相机、模型、接口默认配置。常用现场参数一般只需要改相机 IP。
DEFAULT_CAMERA_IP = "192.168.1.20"   # 实际摄像头IP地址，需根据实际修改
DEFAULT_MODEL = "qwen3.6-plus"       # 千问大模型名称
DEFAULT_API_URL = "https://dashscope.aliyuncs.com/compatible-mode/v1/chat/completions"  # 千问大模型api网址
DEFAULT_API_TIMEOUT = 60.0           # 大模型识别超时（秒）
DEFAULT_MODEL_IMAGE_MAX_EDGE = 1280  # 发给大模型前，图片最长边压缩到该尺寸以内，降低超时概率
DEFAULT_MODEL_IMAGE_JPEG_QUALITY = 85
DEFAULT_MVS_SDK_DIR = "C:\Program Files (x86)\MVS\Development\Samples\Python"  # 海康MVSPython例程地址
DEFAULT_MVS_CAPTURE_NAME = "hik_mvs_capture.jpg"   # 实际相机拍摄保持文件名
DEFAULT_LOCAL_TEST_IMAGE_NAME = "任务卡2-1.png"     # 测试用图片文件名，如果目录下没有该图片，则实机拍摄
DEFAULT_ROBOT_IP = "192.168.1.10"                # 实际机器人IP地址，接入实机需改成实际机器人IP
# DEFAULT_ROBOT_IP = "192.168.193.129"               # AUBO虚拟机器人IP地址
DEFAULT_ROBOT_PORT = 30004                         # 机器人网络端口号，需和实际一致
DEFAULT_ROBOT_NAME = "rob1"                        # 机器人名，需和实际一致
DEFAULT_ROBOT_USER = "aubo"                        # 机器人用户名，需和实际一致
DEFAULT_ROBOT_PASSWORD = "123456"                  # 机器人用户密码，需和实际一致
DEFAULT_ROBOT_TARGET = [23.97, -219.16, 413.11, 1.652, -0.092, 96.964]  # 机器人运动目标地址，需和实际调整一致
DEFAULT_ROBOT_SPEED = 0.1                 # 机器人速度
DEFAULT_ROBOT_ACCELERATION = 0.1           # 机器人加速度
DEFAULT_ROBOT_WAIT_TIMEOUT = 30.0          # 机器人超时
DEFAULT_ROBOT_POSITION_TOLERANCE = 2.0     # 机器人位置容忍误差mm
DEFAULT_ROBOT_ORIENTATION_TOLERANCE = 1.0  # 机器人姿态容忍误差，单位度

# 这里复用 Project2 的 FunASR 本地模型目录，但不修改 Project2 中任何文件。
DEFAULT_ASR_MODEL_DIR = (
    r"E:\ProgramAndModels\Python\Aubo\Project2\resources\modelscope\hub\models\iic\SenseVoiceSmall"
)

# 发给 Qwen 视觉模型的提示词。要求模型只输出一行，便于直接语音播报。
PROMPT_TEXT = (
    "识别工业相机现场拍摄图片中主要可见的物体。"
    "只输出一行，格式严格为：任务卡1中有A、B、C。"
    "A、B、C替换为中文物体名称，用顿号分隔，不要解释，不要换行。"
)

# 语音识别热词纠错表：
# 左边是 ASR 可能识别错的文本，右边是程序真正使用的标准命令。
# 普通话不标准、现场噪声大时，优先在这里补充近音词。
ASR_CORRECTION_DICT = {
    "小菊同学": "小具同学",
    "小桔同学": "小具同学",
    "小剧同学": "小具同学",
    "小局同学": "小具同学",
    "小聚同学": "小具同学",
    "小巨同学": "小具同学",
    "小据同学": "小具同学",
    "小橘同学": "小具同学",
    "玩具同学": "小具同学",
    "小具同": "小具同学",
    "具同学": "小具同学",
    "小俊同学": "小具同学",
    "小君同学": "小具同学",
    "小军同学": "小具同学",
    "小均同学": "小具同学",
    "小钧同学": "小具同学",
    "小郡同学": "小具同学",
    "小旭同学": "小具同学",
    "小句同学": "小具同学",
    "小据同": "小具同学",
    "小聚同": "小具同学",
    "小俊同": "小具同学",
    "小君同": "小具同学",
    "小军同": "小具同学",
    "小均同": "小具同学",
    "拍摄照": "拍摄照片",
    "拍照片": "拍摄照片",
    "拍张照片": "拍摄照片",
    "拍一张照片": "拍摄照片",
    "拍照": "拍摄照片",
    "移动机器人": "移动机器人到指定位置",
    "动机器人到指定位置": "移动机器人到指定位置",
    "动机器人": "移动机器人到指定位置",
    "移动机器": "移动机器人到指定位置",
    "移动机械人": "移动机器人到指定位置",
    "机器人移动": "移动机器人到指定位置",
    "移动到指定位置": "移动机器人到指定位置",
    "移动机器人到位置": "移动机器人到指定位置",
    "确认": "确认移动",
    "确认移动机器人": "确认移动",
    "确定移动": "确认移动",
    "确认一动": "确认移动",
    "确认移到": "确认移动",
    "退出": "退出系统",
    "退出程序": "退出系统",
    "关闭程序": "退出系统",
    "关闭系统": "退出系统",
    "系统退出": "退出系统",
    "系统关闭": "退出系统",
}


# ==============================
# 2. 初始化和通用工具
# ==============================
def configure_logs() -> None:
    # 降低 FunASR/ModelScope 的日志噪声，保留本程序自己的关键输出。
    warnings.filterwarnings("ignore")
    logging.getLogger("modelscope").setLevel(logging.ERROR)
    logging.getLogger("funasr").setLevel(logging.ERROR)
    logging.getLogger().setLevel(logging.ERROR)
    os.environ["MODELSCOPE_LOG_LEVEL"] = "40"


def add_timestamp_prefix(message: str) -> str:
    return "[%s] %s" % (datetime.now().strftime("%Y-%m-%d %H:%M:%S"), message)


def install_timestamped_print() -> None:
    original_print = print

    def timestamped_print(*args, **kwargs) -> None:
        if not args:
            original_print(*args, **kwargs)
            return
        first_text = str(args[0])
        if first_text.startswith("语音识别状态："):
            original_print(*args, **kwargs)
            return
        first = add_timestamp_prefix(first_text)
        original_print(first, *args[1:], **kwargs)

    globals()["print"] = timestamped_print


def log_asr_status(message: str) -> None:
    print("[ASR] " + message, file=sys.stderr, flush=True)


def load_asr_model(model_dir: Path):
    # 加载离线语音识别模型。模型目录不存在时直接退出，避免后面录音后才报错。
    if not model_dir.exists():
        raise SystemExit("找不到语音识别模型目录：" + str(model_dir))
    return AutoModel(model=str(model_dir), trust_remote_code=True, device="cpu", disable_update=True)


# ==============================
# 3. 图片编码、大模型识别、语音播报
# ==============================
def guess_image_mime(image_bytes: bytes, image_name: str = "") -> str:
    # 先根据文件头判断图片类型；不确定时再根据文件名猜测。
    if image_bytes.startswith(b"\xff\xd8\xff"):
        return "image/jpeg"
    if image_bytes.startswith(b"\x89PNG\r\n\x1a\n"):
        return "image/png"
    if image_bytes.startswith(b"BM"):
        return "image/bmp"
    return mimetypes.guess_type(image_name)[0] or "image/jpeg"


def image_file_to_data_url(image_path: Path, max_edge: int, jpeg_quality: int) -> str:
    # Qwen 接口需要 data URL，所以这里把本地图片转成 base64 内嵌数据。
    # 工业相机原图通常分辨率较高，先缩放再上传可以明显减少接口耗时。
    if not image_path.exists():
        raise SystemExit("图片不存在：" + str(image_path))
    image_bytes = image_path.read_bytes()
    original_bytes = image_bytes
    mime_type = guess_image_mime(image_bytes, image_path.name)
    original_size_text = "未知尺寸"
    final_size_text = "未知尺寸"

    try:
        from PIL import Image

        with Image.open(io.BytesIO(image_bytes)) as image:
            original_size_text = f"{image.width}x{image.height}"
            final_image = image
            longest_edge = max(image.width, image.height)
            if max_edge > 0 and longest_edge > max_edge:
                scale = max_edge / float(longest_edge)
                new_size = (max(1, int(image.width * scale)), max(1, int(image.height * scale)))
                final_image = image.resize(new_size, Image.LANCZOS)

            if final_image.mode not in ("RGB", "L"):
                final_image = final_image.convert("RGB")

            buffer = io.BytesIO()
            final_image.save(
                buffer,
                format="JPEG",
                quality=max(1, min(95, jpeg_quality)),
                optimize=True,
            )
            image_bytes = buffer.getvalue()
            mime_type = "image/jpeg"
            final_size_text = f"{final_image.width}x{final_image.height}"
    except ImportError:
        print("提示：未安装 Pillow，跳过图片压缩。建议执行 pip install pillow 后重试。")
    except Exception as exc:
        print("提示：图片压缩失败，改用原图上传：" + str(exc))

    print(
        "模型输入图片：原图 %s，%.1f KB；上传 %s，%.1f KB"
        % (original_size_text, len(original_bytes) / 1024.0, final_size_text, len(image_bytes) / 1024.0)
    )
    image_base64 = base64.b64encode(image_bytes).decode("utf-8")
    return f"data:{mime_type};base64,{image_base64}"


# noinspection DuplicatedCode
def identify_image_file(
    image_path: Path,
    prompt: str,
    model: str,
    api_url: str,
    timeout: float,
    image_max_edge: int,
    image_jpeg_quality: int,
) -> str:
    # API Key 从环境变量读取，避免把密钥写进源码。
    api_key = os.getenv("DASHSCOPE_API_KEY") or os.getenv("QWEN_API_KEY")
    if not api_key:
        raise SystemExit("请先设置环境变量 DASHSCOPE_API_KEY。")

    data_url = image_file_to_data_url(image_path, image_max_edge, image_jpeg_quality)
    print("大模型接口：" + api_url)
    print("大模型名称：" + model)
    print("大模型接口超时：%.0f 秒" % timeout)

    # DashScope 兼容 OpenAI Chat Completions 格式，图片用 image_url data URL 传入。
    payload = {
        "model": model,
        "messages": [
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": prompt},
                    {
                        "type": "image_url",
                        "image_url": {"url": data_url},
                    },
                ],
            }
        ],
    }
    body = json.dumps(payload).encode("utf-8")
    print("大模型请求体大小：%.1f KB" % (len(body) / 1024.0))
    api_request = request.Request(
        api_url,
        data=body,
        method="POST",
        headers={
            "Authorization": f"Bearer {api_key}",
            "Content-Type": "application/json",
        },
    )

    request_start_time = time.monotonic()
    try:
        with request.urlopen(api_request, timeout=timeout) as response:
            result = json.loads(response.read().decode("utf-8"))
    except error.HTTPError as exc:
        message = exc.read().decode("utf-8", errors="replace")
        raise SystemExit("模型接口调用失败：" + f"HTTP {exc.code} {message}") from exc
    except error.URLError as exc:
        elapsed = time.monotonic() - request_start_time
        if isinstance(exc.reason, (TimeoutError, SOCKET_TIMEOUT)):
            raise SystemExit(
                "模型接口调用超时：已等待 %.1f 秒，当前超时设置 %.1f 秒；"
                "请检查网络、API Key/模型名是否可用，或继续缩短图片大小。"
                % (elapsed, timeout)
            ) from exc
        raise SystemExit("模型接口调用失败：" + repr(exc.reason)) from exc
    except SOCKET_TIMEOUT as exc:
        elapsed = time.monotonic() - request_start_time
        raise SystemExit(
            "模型接口调用超时：已等待 %.1f 秒，当前超时设置 %.1f 秒；"
            "请检查网络、API Key/模型名是否可用，或继续缩短图片大小。"
            % (elapsed, timeout)
        ) from exc

    # 只取第一条回复文本。没有文本时认为识别失败。
    choices = result.get("choices", [])
    if choices:
        text = choices[0].get("message", {}).get("content")
        if isinstance(text, str) and text.strip():
            return text.strip()
    raise SystemExit("模型没有返回可读取的文本结果。")


def start_recognition_timer() -> tuple[threading.Event, threading.Thread, float]:
    # 大模型接口是阻塞调用，使用后台线程在等待期间每秒输出一次计时。
    stop_event = threading.Event()
    start_time = time.monotonic()

    def timer_loop() -> None:
        while not stop_event.wait(1.0):
            elapsed = time.monotonic() - start_time
            print("\r大模型识别计时：%.0f 秒" % elapsed, end="", flush=True)

    timer_thread = threading.Thread(target=timer_loop, daemon=True)
    timer_thread.start()
    return stop_event, timer_thread, start_time


def stop_recognition_timer(stop_event: threading.Event, timer_thread: threading.Thread, start_time: float) -> float:
    stop_event.set()
    timer_thread.join(timeout=1.0)
    print()
    return time.monotonic() - start_time


def format_task_card_result(text: str) -> str:
    # 统一输出和播报格式，避免模型返回“图片中有...”等不同措辞。
    cleaned = text.strip().rstrip("。")
    if cleaned.startswith("任务卡1中有"):
        objects_text = cleaned.removeprefix("任务卡1中有").removesuffix("物体")
        return "任务卡1中有" + objects_text + "。"
    if cleaned.startswith("图片中有"):
        objects_text = cleaned.removeprefix("图片中有").removesuffix("物体")
        return "任务卡1中有" + objects_text + "。"
    return "任务卡1中有" + cleaned.removesuffix("物体") + "。"


# noinspection DuplicatedCode
def speak_text(text: str) -> None:
    # 使用 Windows SAPI 直接朗读，不生成 wav 文件。
    # 优先选择中文语音；如果系统没有中文语音，则使用系统默认语音。
    script = r"""
$text = [Console]::In.ReadToEnd()
$speaker = New-Object -ComObject SAPI.SpVoice
$speaker.Rate = 0
$speaker.Volume = 100

$chineseVoice = $speaker.GetVoices() | Where-Object {
    $_.GetDescription() -match "Chinese|中文|Huihui|Kangkang|Yaoyao|Xiaoxiao|Yunxi|Yunyang|Xiaoyi"
} | Select-Object -First 1

if ($null -ne $chineseVoice) {
    $speaker.Voice = $chineseVoice
}

$speaker.Speak($text, 0) | Out-Null
"""
    try:
        subprocess.run(
            ["powershell", "-NoProfile", "-Command", script],
            input=text,
            text=True,
            check=True,
        )
    except (FileNotFoundError, subprocess.CalledProcessError) as exc:
        raise SystemExit("语音播放失败：" + str(exc)) from exc


# ==============================
# 4. 海康 MVS 相机拍照
# ==============================
def format_mvs_error(ret: int) -> str:
    # MVS SDK 返回的是整数错误码，统一格式化成 0xXXXXXXXX 便于查手册。
    return "0x%08x" % (ret & 0xFFFFFFFF)


def decode_mvs_text(values) -> str:
    # MVS 设备名/IP 等字段通常是以 0 结尾的 C 字符数组。
    chars = []
    for value in values:
        if value == 0:
            break
        chars.append(chr(value))
    return "".join(chars)


def get_mvs_device_ip(device_info) -> str:
    # GigE 相机 IP 在 SDK 中是一个 32 位整数，这里转成常见的点分十进制字符串。
    ip_value = device_info.SpecialInfo.stGigEInfo.nCurrentIp
    return ".".join(
        [
            str((ip_value & 0xFF000000) >> 24),
            str((ip_value & 0x00FF0000) >> 16),
            str((ip_value & 0x0000FF00) >> 8),
            str(ip_value & 0x000000FF),
        ]
    )


def load_mvs_sdk(mvs_sdk_dir: Path) -> Any:
    # 加载海康 MVS Python 示例中的 MvImport 封装。
    # 同时把 MVS 运行库目录加入 PATH，解决 PyCharm 里 DLL 找不到的问题。
    mv_import_dir = mvs_sdk_dir / "MvImport"
    if not mv_import_dir.exists():
        raise SystemExit("没有找到海康 MVS Python 封装目录：" + str(mv_import_dir))
    for dll_dir in [
        Path(r"C:\Program Files (x86)\Common Files\MVS\Runtime\Win64_x64"),
        Path(r"C:\Program Files (x86)\Common Files\MVS\Runtime\Win32_i86"),
    ]:
        if dll_dir.exists():
            os.environ["PATH"] = str(dll_dir) + os.pathsep + os.environ.get("PATH", "")
            if hasattr(os, "add_dll_directory"):
                os.add_dll_directory(str(dll_dir))
    sys.path.insert(0, str(mv_import_dir))
    try:
        return importlib.import_module("MvCameraControl_class")
    except (ImportError, OSError) as exc:
        raise SystemExit("加载海康 MVS Python SDK 失败：" + str(exc)) from exc


# noinspection DuplicatedCode
def capture_hik_mvs_image(
    mvs_sdk_dir: Path,
    device_index: int,
    camera_ip: str,
    output_path: Path,
    timeout_ms: int,
    exposure_time: float,
    gain: float,
) -> Path:
    # 完整拍照流程：枚举设备 -> 选择相机 -> 打开相机 -> 取一帧 -> 保存 jpg -> 释放资源。
    sdk = load_mvs_sdk(mvs_sdk_dir)
    mv_camera_class = sdk.MvCamera
    device_info_list_class = sdk.MV_CC_DEVICE_INFO_LIST
    device_info_class = sdk.MV_CC_DEVICE_INFO
    frame_out_class = sdk.MV_FRAME_OUT
    save_image_param_class = sdk.MV_SAVE_IMG_TO_FILE_PARAM
    gige_device = sdk.MV_GIGE_DEVICE
    usb_device = sdk.MV_USB_DEVICE
    access_exclusive = sdk.MV_ACCESS_Exclusive
    trigger_mode_off = sdk.MV_TRIGGER_MODE_OFF
    image_jpeg = sdk.MV_Image_Jpeg

    device_list = device_info_list_class()
    ret = mv_camera_class.MV_CC_EnumDevices(gige_device | usb_device, device_list)
    if ret != 0:
        raise SystemExit("枚举海康相机失败：" + format_mvs_error(ret))
    if device_list.nDeviceNum == 0:
        raise SystemExit("没有找到海康工业相机。请检查网线、交换机、相机 IP、MVS 客户端是否能预览。")

    selected_index = device_index
    print("找到海康相机数量：" + str(device_list.nDeviceNum))
    for index in range(device_list.nDeviceNum):
        # 打印枚举到的相机列表。指定 --mvs-camera-ip 时优先按 IP 选择。
        device_info = cast(device_list.pDeviceInfo[index], POINTER(device_info_class)).contents
        if device_info.nTLayerType == gige_device:
            model_name = decode_mvs_text(device_info.SpecialInfo.stGigEInfo.chModelName)
            current_ip = get_mvs_device_ip(device_info)
            print(f"[{index}] GigE 相机：{model_name}，IP：{current_ip}")
            if camera_ip and current_ip == camera_ip:
                selected_index = index
        elif device_info.nTLayerType == usb_device:
            model_name = decode_mvs_text(device_info.SpecialInfo.stUsb3VInfo.chModelName)
            print(f"[{index}] USB 相机：{model_name}")

    if selected_index < 0 or selected_index >= device_list.nDeviceNum:
        raise SystemExit("相机索引无效：" + str(selected_index))

    cam = mv_camera_class()
    st_device = cast(device_list.pDeviceInfo[selected_index], POINTER(device_info_class)).contents
    ret = cam.MV_CC_CreateHandle(st_device)
    if ret != 0:
        raise SystemExit("创建相机句柄失败：" + format_mvs_error(ret))

    is_open = False
    is_grabbing = False
    try:
        ret = cam.MV_CC_OpenDevice(access_exclusive, 0)
        if ret != 0:
            raise SystemExit(
                "打开相机失败："
                + format_mvs_error(ret)
                + "。请确认 MVS 或 Vision Master 没有占用相机。"
            )
        is_open = True

        if st_device.nTLayerType == gige_device:
            # GigE 相机设置最佳网络包大小，减少丢包和取图失败概率。
            packet_size = cam.MV_CC_GetOptimalPacketSize()
            if int(packet_size) > 0:
                ret = cam.MV_CC_SetIntValue("GevSCPSPacketSize", packet_size)
                if ret != 0:
                    print("警告：设置 GigE 最佳包大小失败：" + format_mvs_error(ret))

        # 当前任务每次只需要一张图，所以关闭硬触发，直接连续取流后抓一帧。
        ret = cam.MV_CC_SetEnumValue("TriggerMode", trigger_mode_off)
        if ret != 0:
            raise SystemExit("设置相机触发模式失败：" + format_mvs_error(ret))

        # 曝光和增益默认不设置；只有命令行传入有效值时才写入相机。
        if exposure_time > 0:
            ret = cam.MV_CC_SetFloatValue("ExposureTime", exposure_time)
            if ret != 0:
                print("警告：设置曝光时间失败：" + format_mvs_error(ret))
        if gain >= 0:
            ret = cam.MV_CC_SetFloatValue("Gain", gain)
            if ret != 0:
                print("警告：设置增益失败：" + format_mvs_error(ret))

        ret = cam.MV_CC_StartGrabbing()
        if ret != 0:
            raise SystemExit("开始取流失败：" + format_mvs_error(ret))
        is_grabbing = True

        frame = frame_out_class()
        memset(byref(frame), 0, sizeof(frame))
        ret = cam.MV_CC_GetImageBuffer(frame, timeout_ms)
        if ret != 0 or not frame.pBufAddr:
            raise SystemExit("获取相机图片失败：" + format_mvs_error(ret))

        try:
            output_path = output_path.expanduser().resolve()
            output_path.parent.mkdir(parents=True, exist_ok=True)

            # 使用 MVS SDK 保存 JPG，避免自己处理 Bayer/RGB 像素格式转换。
            save_param = save_image_param_class()
            memset(byref(save_param), 0, sizeof(save_param))
            save_param.enPixelType = frame.stFrameInfo.enPixelType
            save_param.pData = frame.pBufAddr
            save_param.nDataLen = frame.stFrameInfo.nFrameLen
            save_param.nWidth = frame.stFrameInfo.nWidth
            save_param.nHeight = frame.stFrameInfo.nHeight
            save_param.enImageType = image_jpeg
            save_param.nQuality = 90
            save_param.iMethodValue = 1
            image_path_bytes = str(output_path).encode("mbcs")
            if len(image_path_bytes) >= 256:
                raise SystemExit("MVS 保存图片路径过长，请换一个更短的路径：" + str(output_path))
            save_param.pImagePath = image_path_bytes

            ret = cam.MV_CC_SaveImageToFile(save_param)
            if ret != 0:
                raise SystemExit("保存相机图片失败：" + format_mvs_error(ret))

            print(
                "相机抓图成功："
                + str(output_path)
                + f"，宽 {frame.stFrameInfo.nWidth}，高 {frame.stFrameInfo.nHeight}"
            )
            return output_path
        finally:
            # GetImageBuffer 得到的帧必须释放，否则相机缓存会被占满。
            cam.MV_CC_FreeImageBuffer(frame)
    finally:
        # 不管成功还是失败，都尽量停止取流、关闭设备、销毁句柄。
        if is_grabbing:
            cam.MV_CC_StopGrabbing()
        if is_open:
            cam.MV_CC_CloseDevice()
        cam.MV_CC_DestroyHandle()


# ==============================
# 5. 录音和语音识别
# ==============================
def get_rms(block: bytes) -> float:
    # 计算音频块音量，用于判断是否有人开始说话。
    count = len(block) // 2
    fmt = "%dh" % count
    shorts = struct.unpack(fmt, block)
    sum_squares = 0.0
    for sample in shorts:
        normalized = sample / 32768.0
        sum_squares += normalized * normalized
    return math.sqrt(sum_squares / count) * 32768


def record_audio(output_file: Path) -> Path:
    # VAD 录音：先等待声音超过阈值，再在静音持续一段时间后自动停止。
    start_time = time.monotonic()
    log_asr_status("开始打开麦克风")
    audio = pyaudio.PyAudio()
    sample_width = audio.get_sample_size(FORMAT)
    stream = audio.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, frames_per_buffer=CHUNK)
    log_asr_status("麦克风已打开，开始录音")

    # 丢弃刚打开麦克风时的前 0.5 秒数据，减少设备初始化噪声和上轮播报尾音影响。
    for _ in range(int(RATE / CHUNK * 0.5)):
        stream.read(CHUNK, exception_on_overflow=False)

    frames = []
    silent_chunks = 0
    total_chunks = 0
    limit_chunks = int(SILENCE_LIMIT * RATE / CHUNK)
    max_chunks = int(MAX_DURATION * RATE / CHUNK)
    has_started = False

    try:
        while True:
            data = stream.read(CHUNK, exception_on_overflow=False)
            frames.append(data)
            total_chunks += 1
            rms = get_rms(data)

            if rms > THRESHOLD:
                if not has_started:
                    log_asr_status("检测到声音，RMS=%.1f，阈值=%d" % (rms, THRESHOLD))
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

    with wave.open(str(output_file), "wb") as wav_file:
        wav_file.setnchannels(CHANNELS)
        wav_file.setsampwidth(sample_width)
        wav_file.setframerate(RATE)
        wav_file.writeframes(b"".join(frames))
    log_asr_status(
        "录音结束，用时 %.2f 秒，已检测到声音=%s，音频块=%d"
        % (time.monotonic() - start_time, "是" if has_started else "否", total_chunks)
    )
    return output_file


def normalize_text(text: str) -> str:
    # 清理标点和空白，再应用热词纠错，得到程序内部使用的标准命令文本。
    text = re.sub(r"[，。！？、,!?\s]", "", text).strip()
    for wrong, right in sorted(ASR_CORRECTION_DICT.items(), key=lambda item: len(item[0]), reverse=True):
        if right in text:
            continue
        text = text.replace(wrong, right)
    return text


def has_wakeup_word(text: str) -> bool:
    # 优先精确匹配“小具同学”；匹配失败时用相似度兜底，适应普通话不标准的情况。
    if "小具同学" in text:
        return True
    for length in range(3, 6):
        for start in range(0, max(len(text) - length + 1, 0)):
            fragment = text[start:start + length]
            if difflib.SequenceMatcher(None, fragment, "小具同学").ratio() >= 0.65:
                return True
    return False


def remove_wakeup_word(text: str) -> str:
    # 从一句话命令里移除唤醒词，保留后面的实际命令。
    # 例如“小具同学拍摄照片”会变成“拍摄照片”。
    if "小具同学" in text:
        return text.replace("小具同学", "").strip()
    best_fragment = ""
    best_score = 0.0
    for length in range(3, 6):
        for start in range(0, max(len(text) - length + 1, 0)):
            fragment = text[start:start + length]
            score = difflib.SequenceMatcher(None, fragment, "小具同学").ratio()
            if score > best_score:
                best_score = score
                best_fragment = fragment
    if best_score >= 0.65 and best_fragment:
        return text.replace(best_fragment, "", 1).strip()
    return text


def is_exit_command(text: str) -> bool:
    # “退出系统”作为全局语音指令，监听唤醒词阶段和等待拍摄指令阶段都可生效。
    if "退出系统" in text:
        return True
    exit_phrases = ["退出", "关闭", "结束", "停止", "退岀", "推出系统", "退出西统", "关闭西统"]
    if any(phrase in text for phrase in exit_phrases):
        return True
    return difflib.SequenceMatcher(None, text, "退出系统").ratio() >= 0.6


def recognize_audio(asr_model, wav_file_path: Path) -> str:
    # 调用 FunASR 将临时 wav 文件转文字，并做热词纠错。
    try:
        start_time = time.monotonic()
        log_asr_status("开始 FunASR 识别：" + str(wav_file_path))
        result = asr_model.generate(input=str(wav_file_path), cache={}, language="zh", use_itn=True)
        raw_text = re.sub(r"<\|.*?\|>", "", result[0]["text"]).strip()
        clean_text = normalize_text(raw_text)
        if raw_text and raw_text != clean_text:
            print(f"语音纠错：{raw_text} -> {clean_text}")
        log_asr_status("FunASR 识别完成，用时 %.2f 秒，结果：%s" % (time.monotonic() - start_time, clean_text))
        return clean_text
    except Exception as exc:
        print("语音识别失败：" + str(exc))
        return ""


def listen_once(asr_model, temp_wav_file: Path) -> str:
    # 完成一次“录音 -> 识别 -> 返回标准命令文本”的流程。
    wav_file = record_audio(temp_wav_file)
    return recognize_audio(asr_model, wav_file)


class AsrWorkerTimeout(RuntimeError):
    pass


class AsrWorkerClient:
    def __init__(self, args) -> None:
        self.args = args
        self.process = None
        self.stderr_thread = None
        self.last_worker_status = ""
        self.start()

    def start(self) -> None:
        if self.process is not None and self.process.poll() is None:
            return

        command = [
            sys.executable,
            str(Path(__file__).resolve()),
            "--asr-worker",
            "--asr-model-dir",
            str(self.args.asr_model_dir),
            "--temp-wav-file",
            str(self.args.temp_wav_file),
        ]
        print("正在加载语音识别模型...")
        self.last_worker_status = ""
        self.process = subprocess.Popen(
            command,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            encoding="utf-8",
            errors="replace",
        )
        self._start_stderr_reader()
        ready = self._read_result(ASR_READY_TIMEOUT_SECONDS)
        if not ready.get("ok") or ready.get("event") != "ready":
            error = ready.get("error", "未知错误")
            self.stop()
            raise RuntimeError("语音识别模型加载失败：" + error)
        print("语音识别模型加载完成。")

    def listen_once(self) -> str:
        if self.process is None or self.process.poll() is not None:
            self.restart("语音识别进程未启动。")
            return ""

        try:
            print("语音识别状态：发送 listen 命令")
            self.process.stdin.write("listen\n")
            self.process.stdin.flush()
            result = self._read_result(ASR_LISTEN_TIMEOUT_SECONDS)
        except AsrWorkerTimeout as exc:
            reason = "语音识别超时：已等待 %.1f 秒，Python 进程没有返回结果。" % ASR_LISTEN_TIMEOUT_SECONDS
            if self.last_worker_status:
                reason += "最后状态：" + self.last_worker_status
            self.restart(reason)
            print("语音识别失败：" + str(exc) + "；已重启，请重新说指令。")
            return ""
        except Exception as exc:
            self.restart("发送语音识别命令失败：" + str(exc))
            return ""

        if not result.get("ok"):
            print("语音识别失败：" + str(result.get("error", "")))
            return ""
        return result.get("text", "")

    def _start_stderr_reader(self) -> None:
        def read_stderr() -> None:
            process = self.process
            if process is None or process.stderr is None:
                return
            while True:
                try:
                    line = process.stderr.readline()
                except Exception:
                    return
                if not line:
                    return
                line = line.strip()
                if line.startswith("[ASR]"):
                    status = line[len("[ASR]"):].strip()
                    self.last_worker_status = status
                    print("语音识别状态：" + status)

        self.stderr_thread = threading.Thread(target=read_stderr, daemon=True)
        self.stderr_thread.start()

    def restart(self, reason: str) -> None:
        print(reason)
        print("正在重启语音识别进程，请稍候...")
        self.stop()
        try:
            self.start()
            print("语音识别进程已重启，请重新说指令。")
        except Exception as exc:
            print("重启语音识别进程失败：" + str(exc))

    def stop(self) -> None:
        process = self.process
        self.process = None
        if process is None:
            return

        try:
            if process.poll() is None and process.stdin:
                try:
                    process.stdin.write("exit\n")
                    process.stdin.flush()
                except Exception:
                    pass
                try:
                    process.wait(timeout=1)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=2)
        finally:
            for stream in (process.stdin, process.stdout, process.stderr):
                try:
                    if stream:
                        stream.close()
                except Exception:
                    pass

    def _read_result(self, timeout_seconds: float) -> dict:
        deadline = time.monotonic() + timeout_seconds
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise AsrWorkerTimeout("语音识别超时")

            line = self._readline_with_timeout(remaining)
            if not line:
                raise RuntimeError("语音识别进程已退出。")

            line = line.strip()
            if not line:
                continue

            try:
                return json.loads(line)
            except json.JSONDecodeError:
                continue

    def _readline_with_timeout(self, timeout_seconds: float) -> str:
        result_queue = queue.Queue(maxsize=1)

        def read_line() -> None:
            try:
                result_queue.put(self.process.stdout.readline())
            except Exception:
                result_queue.put("")

        thread = threading.Thread(target=read_line, daemon=True)
        thread.start()
        try:
            return result_queue.get(timeout=timeout_seconds)
        except queue.Empty as exc:
            raise AsrWorkerTimeout("语音识别超时") from exc

    def close(self) -> None:
        self.stop()


def run_asr_worker(args) -> int:
    try:
        asr_model = load_asr_model(args.asr_model_dir)
        print(json.dumps({"ok": True, "event": "ready"}, ensure_ascii=False), flush=True)
        for line in sys.stdin:
            command = line.strip()
            if command == "exit":
                break
            if command != "listen":
                print(json.dumps({"ok": False, "error": "未知命令：" + command}, ensure_ascii=False), flush=True)
                continue

            try:
                log_asr_status("收到 listen 命令")
                text = listen_once(asr_model, args.temp_wav_file)
                print(json.dumps({"ok": True, "text": text}, ensure_ascii=False), flush=True)
            except Exception as exc:
                print(json.dumps({"ok": False, "error": str(exc)}, ensure_ascii=False), flush=True)
        return 0
    except Exception as exc:
        print(json.dumps({"ok": False, "error": str(exc)}, ensure_ascii=False), flush=True)
        return 1


# ==============================
# 6. 业务流程：拍照、识别、播报
# ==============================
def capture_identify_and_speak(args) -> None:
    # 被“拍摄照片”命令触发：先拍照，再调用大模型识别，最后朗读识别结果。
    if args.debug_image:
        image_path = args.debug_image
        print("调试模式：使用本地图片代替相机拍照：" + str(image_path))
    elif args.local_test_image.exists():
        image_path = args.local_test_image
        print("检测到本地任务卡图片，直接读取硬盘文件：" + str(image_path))
    else:
        image_path = capture_hik_mvs_image(
            mvs_sdk_dir=args.mvs_sdk_dir,
            device_index=args.mvs_device_index,
            camera_ip=args.mvs_camera_ip,
            output_path=args.mvs_capture_image,
            timeout_ms=args.mvs_timeout_ms,
            exposure_time=args.mvs_exposure_time,
            gain=args.mvs_gain,
        )
    print("开始调用大模型识别图片：" + str(image_path))
    print("大模型正在识别任务卡1")
    speak_text("大模型正在识别任务卡1")
    timer_stop_event, timer_thread, recognition_start_time = start_recognition_timer()
    try:
        result = identify_image_file(
            image_path,
            PROMPT_TEXT,
            args.model,
            args.api_url,
            args.api_timeout,
            args.model_image_max_edge,
            args.model_image_jpeg_quality,
        )
    except SystemExit as exc:
        stop_recognition_timer(timer_stop_event, timer_thread, recognition_start_time)
        message = str(exc)
        print("大模型识别失败：" + message)
        speak_text("大模型识别失败")
        return
    except Exception as exc:
        stop_recognition_timer(timer_stop_event, timer_thread, recognition_start_time)
        message = str(exc)
        print("大模型识别异常：" + message)
        speak_text("大模型识别异常")
        return

    recognition_elapsed = stop_recognition_timer(timer_stop_event, timer_thread, recognition_start_time)
    print("大模型识别耗时：%.2f 秒" % recognition_elapsed)
    result = format_task_card_result(result)
    print("大模型识别结果：" + result)
    speak_text(result)


def format_robot_coordinate(target_values: list[float]) -> str:
    # 控制台输出使用现场输入值，格式固定为 [x,y,z,rx,ry,rz]。
    values = []
    for value in target_values:
        if float(value).is_integer():
            values.append(str(int(value)))
        else:
            values.append(str(value))
    return "[" + ",".join(values) + "]"


def is_move_robot_command(text: str) -> bool:
    # 识别“移动机器人到指定位置”类命令。热词纠错已经尽量归一化到标准文本。
    if "移动机器人到指定位置" in text or "动机器人到指定位置" in text or "移动到指定位置" in text:
        return True
    has_target = "指定位置" in text or "坐标" in text
    return has_target and ("移动机器人" in text or "动机器人" in text)


def is_confirm_move_command(text: str) -> bool:
    # 确认移动必须单独识别，避免误触发机器人运动。
    if "确认移动" in text:
        return True
    return difflib.SequenceMatcher(None, text, "确认移动").ratio() >= 0.75


def connect_robot(args):
    # 连接 AUBO 机器人，流程参考 Project2/robot_core_v10.py。
    if RpcClient is None:
        raise RuntimeError("没有安装 pyaubo_sdk，无法连接 AUBO 机器人。")

    rpc = RpcClient()
    rpc.setRequestTimeout(args.robot_timeout_ms)
    rpc.connect(args.robot_ip, args.robot_port)
    try:
        rpc.login(args.robot_user, args.robot_password)
    except Exception:
        # robot_core_v10.py 中登录异常会静默通过，这里保持同样策略。
        pass
    return rpc, rpc.getRobotInterface(args.robot_name)


def calculate_position_error_mm(current_pose: list[float], target_pose: list[float]) -> float:
    # getTcpPose 返回米，转换成毫米误差，便于和命令行容差参数比较。
    dx = (current_pose[0] - target_pose[0]) * 1000.0
    dy = (current_pose[1] - target_pose[1]) * 1000.0
    dz = (current_pose[2] - target_pose[2]) * 1000.0
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def calculate_orientation_error_degrees(current_pose: list[float], target_pose: list[float]) -> float:
    # getTcpPose 返回弧度，转换成角度误差，便于现场查看和调整容差。
    drx = math.degrees(current_pose[3] - target_pose[3])
    dry = math.degrees(current_pose[4] - target_pose[4])
    drz = math.degrees(current_pose[5] - target_pose[5])
    return math.sqrt(drx * drx + dry * dry + drz * drz)


def wait_until_robot_arrives(robot_interface, target_pose: list[float], args) -> bool:
    # 轮询 TCP 位姿，只有真实位置和姿态都接近目标位姿后才认为移动到位。
    start_time = time.monotonic()
    last_report_time = 0.0
    while True:
        current_pose = robot_interface.getRobotState().getTcpPose()
        error_mm = calculate_position_error_mm(current_pose, target_pose)
        orientation_error = calculate_orientation_error_degrees(current_pose, target_pose)
        elapsed = time.monotonic() - start_time

        if elapsed - last_report_time >= 1.0:
            print("等待机器人到位，位置误差 %.2f mm，姿态误差 %.2f 度" % (error_mm, orientation_error))
            last_report_time = elapsed

        if error_mm <= args.robot_position_tolerance and orientation_error <= args.robot_orientation_tolerance:
            print("机器人到位，最终位置误差 %.2f mm，姿态误差 %.2f 度" % (error_mm, orientation_error))
            return True
        if elapsed >= args.robot_wait_timeout:
            print("等待机器人到位超时，最终位置误差 %.2f mm，姿态误差 %.2f 度" % (error_mm, orientation_error))
            return False
        time.sleep(0.2)


def move_robot_to_coordinate(target_values: list[float], args) -> bool:
    # 现场输入为 XYZ 毫米和 RX/RY/RZ 角度；AUBO SDK 使用米和弧度。
    coordinate_text = format_robot_coordinate(target_values)
    print("准备移动机器人到位姿：" + coordinate_text)
    try:
        rpc, robot_interface = connect_robot(args)
        target_x, target_y, target_z = [value / 1000.0 for value in target_values[:3]]
        target_rx, target_ry, target_rz = [math.radians(value) for value in target_values[3:]]
        target_pose = [
            target_x,
            target_y,
            target_z,
            target_rx,
            target_ry,
            target_rz,
        ]
        robot_interface.getMotionControl().moveLine(
            target_pose,
            args.robot_speed,
            args.robot_acceleration,
            0.0,
            True,
        )
        arrived = wait_until_robot_arrives(robot_interface, target_pose, args)
        if hasattr(rpc, "disconnect"):
            rpc.disconnect()
        if arrived:
            print("机器人已移动到位姿：" + coordinate_text)
            return True
        print("机器人未在限定时间内到达位姿：" + coordinate_text)
        return False
    except Exception as exc:
        print("机器人移动失败：" + str(exc))
        return False


def confirm_and_move_robot(voice, args) -> bool:
    # 机器人移动属于高风险动作，必须二次语音确认。
    coordinate_text = format_robot_coordinate(args.robot_target)
    print("机器人目标位姿：" + coordinate_text)
    print("是否确认移动机器人到位姿：" + coordinate_text)
    print("等待确认指令：确认移动")

    for attempt in range(1, 4):
        speak_text("请说确认移动" if attempt == 1 else "请再说确认移动")
        print("正在录音，请说确认移动...")
        confirm_text = voice.listen_once()
        if not confirm_text:
            print("未检测到确认指令，继续等待确认。")
            continue

        print("识别到确认指令：" + confirm_text)
        if is_exit_command(confirm_text):
            speak_text("收到，系统退出")
            return False
        if is_confirm_move_command(confirm_text):
            break

        print("未识别到确认移动，继续等待确认。")
    else:
        speak_text("未确认移动，已取消")
        return True

    speak_text("开始移动机器人")
    if move_robot_to_coordinate(args.robot_target, args):
        speak_text("已移动到位")
        # 任务 1：机器人到达拍照位姿后立即拍照识别，不再等待额外语音指令。
        speak_text("开始拍摄")
        capture_identify_and_speak(args)
    else:
        speak_text("机器人移动失败")
    return True


def handle_wakeup_command(voice, args) -> bool:
    # 用户只说“小具同学”时，程序进入第二轮等待，继续听“拍摄照片”或“退出系统”。
    while True:
        print("正在录音，请说指令...")
        command_text = voice.listen_once()
        if not command_text:
            speak_text("未检测到有效指令，请重新说指令")
            print("等待语音指令：拍摄照片 / 移动机器人到指定位置 / 退出系统")
            continue

        command_text = remove_wakeup_word(command_text)
        print("识别到指令：" + command_text)
        if is_exit_command(command_text):
            speak_text("收到，系统退出")
            return False
        if "拍摄照片" in command_text:
            speak_text("开始拍摄")
            capture_identify_and_speak(args)
            return True
        if is_move_robot_command(command_text):
            return confirm_and_move_robot(voice, args)

        speak_text("没有识别到有效指令，请重新说指令")
        print("没有识别到有效指令，继续等待指令。")
        print("等待语音指令：拍摄照片 / 移动机器人到指定位置 / 退出系统")


# ==============================
# 7. 程序入口
# ==============================
def parse_robot_target(value: str) -> list[float]:
    # 命令行依次传入 XYZ 毫米坐标和 RX/RY/RZ 角度。
    cleaned = value.strip().strip("[]")
    parts = [part.strip() for part in cleaned.split(",") if part.strip()]
    if len(parts) != 6:
        raise argparse.ArgumentTypeError("机器人目标位姿必须包含 x,y,z,rx,ry,rz 六个值。")
    try:
        return [float(part) for part in parts]
    except ValueError as exc:
        raise argparse.ArgumentTypeError("机器人目标位姿必须是数字。") from exc


def main() -> None:
    # 命令行参数保留现场可调项，PyCharm 里通常只需要传 --mvs-camera-ip。
    parser = argparse.ArgumentParser(description="语音唤醒后调用海康 MVS 相机拍照，并识别图片中的物体。")
    parser.add_argument("--asr-worker", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--asr-model-dir", type=Path, default=Path(DEFAULT_ASR_MODEL_DIR), help="FunASR 模型目录。")
    parser.add_argument("--temp-wav-file", type=Path, default=Path(TEMP_WAV_FILE), help="临时录音文件路径。")
    parser.add_argument("--mvs-sdk-dir", type=Path, default=Path(DEFAULT_MVS_SDK_DIR), help="海康 MVS Python 示例根目录。")
    parser.add_argument("--mvs-device-index", type=int, default=0, help="海康相机索引。")
    parser.add_argument("--mvs-camera-ip", default=DEFAULT_CAMERA_IP, help="海康 GigE 相机 IP。")
    parser.add_argument("--mvs-capture-image", type=Path, default=Path(DEFAULT_MVS_CAPTURE_NAME), help="拍照保存路径。")
    parser.add_argument("--mvs-timeout-ms", type=int, default=3000, help="获取一帧图片的超时时间，单位毫秒。")
    parser.add_argument("--mvs-exposure-time", type=float, default=-1.0, help="曝光时间，单位微秒。小于等于 0 表示不设置。")
    parser.add_argument("--mvs-gain", type=float, default=-1.0, help="相机增益。小于 0 表示不设置。")
    parser.add_argument("--debug-image", type=Path, default=None, help="无相机调试图片。填写后拍摄照片命令会直接识别该图片。")
    parser.add_argument(
        "--local-test-image",
        type=Path,
        default=Path(DEFAULT_LOCAL_TEST_IMAGE_NAME),
        help="默认本地测试图片。存在时优先读取，不存在时调用真实相机。",
    )
    parser.add_argument("--model", default=os.getenv("QWEN_MODEL", DEFAULT_MODEL), help="Qwen 视觉模型名称。")
    parser.add_argument("--api-url", default=os.getenv("QWEN_API_URL", DEFAULT_API_URL), help="Qwen 接口地址。")
    parser.add_argument("--api-timeout", type=float, default=DEFAULT_API_TIMEOUT, help="Qwen 接口超时时间，单位秒。")
    parser.add_argument(
        "--model-image-max-edge",
        type=int,
        default=DEFAULT_MODEL_IMAGE_MAX_EDGE,
        help="发给大模型前的图片最长边，0 表示不缩放。",
    )
    parser.add_argument(
        "--model-image-jpeg-quality",
        type=int,
        default=DEFAULT_MODEL_IMAGE_JPEG_QUALITY,
        help="发给大模型前的 JPEG 压缩质量，1-95。",
    )
    parser.add_argument("--robot-ip", default=DEFAULT_ROBOT_IP, help="AUBO 机器人 IP。")
    parser.add_argument("--robot-port", type=int, default=DEFAULT_ROBOT_PORT, help="AUBO 机器人端口。")
    parser.add_argument("--robot-name", default=DEFAULT_ROBOT_NAME, help="AUBO 机器人实例名称。")
    parser.add_argument("--robot-user", default=DEFAULT_ROBOT_USER, help="AUBO 登录用户名。")
    parser.add_argument("--robot-password", default=DEFAULT_ROBOT_PASSWORD, help="AUBO 登录密码。")
    parser.add_argument("--robot-timeout-ms", type=int, default=5000, help="AUBO SDK 请求超时时间，单位毫秒。")
    parser.add_argument("--robot-speed", type=float, default=DEFAULT_ROBOT_SPEED, help="机器人直线运动速度。")
    parser.add_argument("--robot-acceleration", type=float, default=DEFAULT_ROBOT_ACCELERATION, help="机器人直线运动加速度。")
    parser.add_argument("--robot-wait-timeout", type=float, default=DEFAULT_ROBOT_WAIT_TIMEOUT, help="等待机器人到位超时时间，单位秒。")
    parser.add_argument(
        "--robot-position-tolerance",
        type=float,
        default=DEFAULT_ROBOT_POSITION_TOLERANCE,
        help="机器人到位判定容差，单位毫米。",
    )
    parser.add_argument(
        "--robot-orientation-tolerance",
        type=float,
        default=DEFAULT_ROBOT_ORIENTATION_TOLERANCE,
        help="机器人到位姿态判定容差，单位度。",
    )
    parser.add_argument(
        "--robot-target",
        type=parse_robot_target,
        default=DEFAULT_ROBOT_TARGET,
        help="机器人目标位姿，格式为 x,y,z,rx,ry,rz；前三项单位毫米，后三项单位度。",
    )
    args = parser.parse_args()

    configure_logs()

    if args.asr_worker:
        raise SystemExit(run_asr_worker(args))

    install_timestamped_print()

    voice = AsrWorkerClient(args)
    speak_text("语音拍照识别系统已启动，请呼叫小具同学")

    try:
        while True:
            # 主循环：持续监听环境音，只有听到唤醒词或退出指令才继续处理。
            print("监听中，等待唤醒词：小具同学")
            spoken_text = voice.listen_once()
            if not spoken_text:
                continue

            print("听到：" + spoken_text)
            if is_exit_command(spoken_text):
                speak_text("收到，系统退出")
                break

            if not has_wakeup_word(spoken_text):
                continue

            command_text = remove_wakeup_word(spoken_text)
            if is_exit_command(command_text):
                speak_text("收到，系统退出")
                break

            if "拍摄照片" in command_text:
                # 支持一句话连说：“小具同学拍摄照片”。
                speak_text("开始拍摄")
                capture_identify_and_speak(args)
                continue

            if is_move_robot_command(command_text):
                if not confirm_and_move_robot(voice, args):
                    break
                continue

            print("等待语音指令：拍摄照片 / 移动机器人到指定位置 / 退出系统")
            speak_text("我在，请说指令")
            if not handle_wakeup_command(voice, args):
                break
    except KeyboardInterrupt:
        speak_text("系统退出")
    finally:
        voice.close()
        # 程序退出时删除临时录音文件，避免目录中残留旧音频。
        if args.temp_wav_file.exists():
            try:
                args.temp_wav_file.unlink()
            except OSError:
                pass


if __name__ == "__main__":
    main()
