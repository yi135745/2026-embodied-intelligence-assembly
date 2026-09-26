"""全局配置：现场可调参数集中在这里。

比赛现场一般只需要修改：
    - CAMERA_IP       海康相机 IP
    - ROBOT_IP        遨博机器人 IP（以及端口/用户名/密码）
    - ROBOT_TARGET    任务卡 1 拍照位（六维位姿，需示教标定）
    - VOICE_BACKEND / AI_BOX_URL  语音后端及AI盒子HTTP地址
    - DASHSCOPE_API_KEY 大模型密钥（用环境变量，不要写进这里）
"""

import os
import json
from pathlib import Path


def _load_local_secrets() -> dict:
    """读取仅保存在本机、不会提交Git的密码文件。"""
    secrets_path = Path(__file__).resolve().parent / "secrets.json"
    if not secrets_path.exists():
        return {}
    try:
        data = json.loads(secrets_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise RuntimeError("读取secrets.json失败：%s" % exc) from exc
    if not isinstance(data, dict):
        raise RuntimeError("secrets.json顶层必须是JSON对象。")
    return data


_LOCAL_SECRETS = _load_local_secrets()

# ==============================
# 0. 项目路径：所有路径以项目根为锚点，换机器无需改动
# ==============================
PROJECT_ROOT = Path(__file__).resolve().parent
RESOURCES_DIR = PROJECT_ROOT / "resources"
DATA_DIR = PROJECT_ROOT / "data"
OUTPUT_DIR = PROJECT_ROOT / "output"

# ==============================
# 1. 语音唤醒 / 录音 / 识别
# ==============================
WAKE_WORD = "小具同学"                  # 唤醒词
READY_REPLY = "我已就绪，请下达指令"    # 唤醒后就绪播报
TASK1_COMMAND = "任务一"                # 进入任务一的提示词
TASK2_COMMAND = "任务二"                # 进入任务二的提示词
RETURN_REPLY = "任务已完成，请再次呼叫小具同学"  # 任务返回后的提示播报

# 国赛默认由盒子录音/识别/播放；local显式切回省赛电脑麦克风/扬声器。
VOICE_BACKEND = os.getenv("VOICE_BACKEND", "ai_box")
AI_BOX_URL = os.getenv("AI_BOX_URL", "http://192.168.34.200:8765")
AI_BOX_HEALTH_TIMEOUT = 5.0
AI_BOX_ASR_TIMEOUT = 90.0       # 含录音、首次模型初始化；超时不自动重发
AI_BOX_TTS_TIMEOUT = 120.0
AI_BOX_START_TIMEOUT = 5.0
AI_BOX_MAX_RECORD_SECONDS = 15.0
AI_BOX_VAD_THRESHOLD = 0.5
# 盒子原生关键词模型为“小E同学”。当前走prewoken ASR后匹配WAKE_WORD，
# 不改服务端模型；不支持播报中唤醒打断。下面录音/模型参数仅供local后端。

CHUNK = 1024
CHANNELS = 1
RATE = 16000
TEMP_WAV_FILE = str(OUTPUT_DIR / "temp_voice_command.wav")

# VAD 录音参数：
# THRESHOLD 越大越不容易被环境噪声触发；SILENCE_LIMIT 为检测到人声后连续静音多少秒停止。
THRESHOLD = 300
SILENCE_LIMIT = 2
MAX_DURATION = 15

# FunASR 本地模型目录（相对项目根：resources/modelscope/，已 gitignore，需自行放置模型）
ASR_MODEL_DIR = str(RESOURCES_DIR / "modelscope" / "hub" / "models" / "iic" / "SenseVoiceSmall")

# 语音识别热词纠错表：左 ASR 可能识别错的文本 -> 右程序标准命令。
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
    # 任务派发（FunASR 开启 ITN 后中文数字可能被转成阿拉伯数字）
    "任务1": "任务一",
    "任务2": "任务二",
}

# ==============================
# 2. 本地 TTS（pyttsx3）
# ==============================
TTS_RATE = 150  # 正常语速

# ==============================
# 3. 海康 MVS 相机
# ==============================
CAMERA_IP = "192.168.34.20"   # 实际摄像头 IP，需按现场修改
# 海康 MVS Python 例程地址：外部软件安装目录，换机器需改成这台机器上 MVS 的 Samples\Python 路径
MVS_SDK_DIR = r"E:\software\mvs\MVS\Development\Samples\Python"
MVS_CAPTURE_NAME = str(OUTPUT_DIR / "hik_mvs_capture.jpg")   # 实机拍照保存文件名
MVS_DEVICE_INDEX = 0
MVS_TIMEOUT_MS = 3000          # 取一帧超时（毫秒）
MVS_EXPOSURE_TIME = -1.0       # 默认曝光时间（微秒），<=0 表示不设置
MVS_GAIN = -1.0                # 默认增益，<0 表示不设置

LOCAL_TEST_IMAGE_NAME = str(PROJECT_ROOT / "任务卡1.png")   # 测试用图片：目录下存在时优先读取，否则实机拍摄
DEBUG_IMAGE = None                      # 无相机调试图片路径（None 表示不启用）

# ==============================
# 4. 大模型（DashScope Qwen 视觉）
# ==============================
MODEL = "qwen3-vl-flash"      # 千问大模型名称
API_URL = "https://dashscope.aliyuncs.com/compatible-mode/v1/chat/completions"
DASHSCOPE_API_KEY = os.getenv("DASHSCOPE_API_KEY") or _LOCAL_SECRETS.get("DASHSCOPE_API_KEY", "")
API_TIMEOUT = 60.0           # 识别超时（秒）
IMAGE_MAX_EDGE = 1280        # 上传前图片最长边压缩到该尺寸以内
JPEG_QUALITY = 85            # 上传前 JPEG 压缩质量

# 视觉识别提示词：要求只输出一行，便于直接语音播报。
PROMPT_TEXT = (
    "识别工业相机现场拍摄图片中主要可见的物体。"
    "只输出一行，格式严格为：任务卡1中有A、B、C。"
    "A、B、C替换为中文物体名称，用顿号分隔，不要解释，不要换行。"
)

# ==============================
# 4.1 任务二：任务卡 / 方块 / 托盘视觉
# ==============================
TASK2_CARD_DEBUG_IMAGE = None   # 调试时填本地图片绝对路径；None 表示 MVS 抓图
TASK2_BLOCK_DEBUG_IMAGE = None
TASK2_TRAY_DEBUG_IMAGE = None
TASK2_CARD_CAPTURE_NAME = "task2_card.jpg"
TASK2_BLOCK_CAPTURE_NAME = "task2_blocks.jpg"
TASK2_TRAY_CAPTURE_NAME = "task2_trays.jpg"
TASK2_OUTPUT_DIR = str(OUTPUT_DIR / "task2")
TASK2_TUNING_FILE = str(DATA_DIR / "task2_tuning.json")  # 独立调参工具保存；存在时自动覆盖下列默认参数
TASK2_OFFSET_FILE = str(DATA_DIR / "task2_offsets_v2.json")  # 坐标逻辑修正后的偏差文件
TASK2_VERIFIED_TRAY_FILE = str(DATA_DIR / "task2_verified_trays.json")  # 仅供独立HSV/中心调试，正式任务不读取

# AUBO四点位姿唯一数据源；不存在开关，任务运行时必须读取该文件。
AUBO_POSE_JSON_FILE = str(DATA_DIR / "aubo_poses.json")

TASK2_HSV_RANGES = {
    # 当前实拍中紫色H约173，属于HSV环回端；红色只保留0附近，避免抢走紫色。
    "红色": [((0, 100, 90), (8, 230, 230))],
    "橙色": [((11, 80, 60), (24, 255, 255))],
    "黄色": [((25, 80, 60), (35, 255, 255))],
    "绿色": [((55, 40, 85), (75, 100, 180))],
    "蓝色": [((86, 70, 45), (130, 255, 255))],
    "紫色": [((165, 65, 60), (179, 160, 155))],
}
# 方块表面反光且当前曝光偏暗，H较稳定而V波动很大；降低V下限，避免亮度变化漏检。
# 方块紫(H约128~132)与托盘印刷紫(H约171~176)不同，必须分区配置。
TASK2_BLOCK_HSV_RANGES = {
    # 当前粉色在强补光下也落到H 3~4；候选重叠时用实拍原型的S/V选择真正红块。
    "红色": [((0, 80, 25), (8, 255, 255)), ((175, 80, 25), (179, 255, 255))],
    "橙色": [((9, 70, 25), (19, 255, 255))],
    "黄色": [((20, 70, 25), (35, 255, 255))],
    "绿色": [((55, 50, 25), (90, 255, 255))],
    "蓝色": [((95, 60, 25), (119, 255, 255))],
    "紫色": [((120, 50, 25), (145, 255, 255))],
    # 国赛新增三色：初始调参值，须用现场实物/光照重新验收。
    "青色": [((80, 55, 40), (100, 255, 255))],
    "粉色": [((145, 35, 100), (174, 180, 255)), ((0, 110, 130), (8, 195, 255))],
    "棕色": [((5, 70, 20), (20, 255, 130)), ((8, 25, 35), (20, 100, 130))],
}
# 低饱和颜色不能靠宽HSV掩膜找轮廓；由形状检测找到整块后，用内部HSV中位数作为额外分类原型。
TASK2_BLOCK_COLOR_PROTOTYPES_HSV = {
    "红色": ((4, 180, 118),),
    "橙色": ((14, 192, 251),),
    "黄色": ((25, 170, 253),),
    "绿色": ((73, 145, 75),),
    "蓝色": ((107, 160, 159),),
    "紫色": ((128, 115, 94),),
    "青色": ((94, 88, 222),),
    "粉色": ((3, 158, 211), (174, 136, 154)),
    "棕色": ((13, 51, 77), (35, 22, 58)),
}
TASK2_TRAY_HSV_RANGES = dict(TASK2_HSV_RANGES)
TASK2_TRAY_COLORS = ("红色", "橙色", "黄色", "绿色", "蓝色", "紫色")
TASK2_EXTRA_BLOCK_COLORS = ("青色", "粉色", "棕色")
TASK2_BLOCK_COLORS = TASK2_TRAY_COLORS + TASK2_EXTRA_BLOCK_COLORS
# 临时缺件时可填写要跳过的方块颜色；当前九色齐全，全部参与检测与完整性校验。
TASK2_DISABLED_BLOCK_COLORS = ()
# None：按卡面接受1~3个叠放动作；规则明确后设1/2/3。0仅供省赛六步回归。
TASK2_EXPECTED_STACK_COUNT = None
TASK2_MIN_CONTOUR_AREA = 300
TASK2_MAX_CONTOUR_AREA = 200000
TASK2_MORPH_KERNEL = 5
# HSV直接检测异常时的六色联合兜底。权重越大，该项对最终颜色分配影响越大。
TASK2_COLOR_FALLBACK_ENABLED = True
TASK2_COLOR_MASK_WEIGHT = 0.45       # 像素落入该颜色HSV区间的比例
TASK2_COLOR_HUE_WEIGHT = 0.35        # HSV色相距离
TASK2_COLOR_LAB_WEIGHT = 0.20        # Lab感知颜色距离
TASK2_COLOR_FALLBACK_MIN_S = 30      # 绿色托盘实拍S约38~58，需保留其完整主体
TASK2_COLOR_FALLBACK_MIN_V = 20      # 宽掩膜：保留暗红、暗紫、暗绿
TASK2_COLOR_MAX_ASPECT_RATIO = 1.8   # 排除灯带、线缆等细长背景轮廓
TASK2_COLOR_MIN_RECT_FILL = 0.55     # 轮廓面积/最小外接矩形面积
TASK2_COLOR_MAX_AREA_JUMP = 1.8      # 最大候选若远大于次大候选，判为背景区域
TASK2_COLOR_MAX_CANDIDATES = 10      # 保留额外候选，由联合评分排除台外同色杂物
TASK2_COLOR_MAX_ASSIGNMENT_COST = 0.65  # 保留省赛暗紫实拍；新三色需现场验收该阈值
TASK2_COLOR_BAD_AREA_RATIO = 0.25    # 某轮廓面积低于中位数该比例时视为只识别到边缘
# strict：边界失败停止；fallback：边界可靠时限区，失败则警告并退回全图形状/颜色；off：不找白板。
TASK2_BLOCK_BOARD_GUARD_MODE = "fallback"
TASK2_CALIBRATION_FILE = str(RESOURCES_DIR / "visionmaster_task2_calibration.xml")  # VisionMaster兼容标定XML，通过版本工具启用
TASK2_CALIBRATION_TEMPLATE_FILE = str(RESOURCES_DIR / "visionmaster_task2_template.xml")
TASK2_CALIBRATION_HISTORY_DIR = str(RESOURCES_DIR / "calibration_history")
TASK2_CALIBRATION_BINDING_FILE = str(RESOURCES_DIR / "visionmaster_task2_calibration_binding.json")
TASK2_CALIBRATION_REPORT_FILE = str(RESOURCES_DIR / "visionmaster_task2_calibration_report.json")
# 方块区与托盘区保持相同相机Z/RZ，只改变XY，因此共用一个平面标定接口。
# 两个旧名称保留为同一路径，避免外部代码引用时报错，不再代表独立矩阵。
TASK2_TRAY_CALIBRATION_FILE = TASK2_CALIBRATION_FILE
TASK2_TRAY_CALIBRATION_BINDING_FILE = TASK2_CALIBRATION_BINDING_FILE
TASK2_SHARED_CALIBRATION_MAX_Z_DIFF_MM = 2.0
TASK2_SHARED_CALIBRATION_MAX_RZ_DIFF_RAD = 0.02
TASK2_TRAY_CALIBRATION_STEP_MM = 8.0  # 一次九点时相机相邻采样位移；每帧同时取6个托盘中心
TASK2_CALIBRATION_WORLD_SCALE_MM = 1.0  # 当前VM九点标定矩阵已直接输出mm
# 物理标定只做一次方块区人工粗对准；这里只改变后续自动采样数量。
# quick用于现场快速建立，robust用于学校充分调试。两者写入相同的正式偏移接口。
TASK2_OFFSET_CALIBRATION_PROFILE = "robust"
TASK2_OFFSET_CALIBRATION_PLANS = {
    "quick": {
        "rotation_moves_deg": (25.0,),
    },
    "robust": {
        "rotation_moves_deg": (25.0, -25.0),
    },
}
TASK2_REQUIRE_ALL_COLORS = True  # 国赛识别9色方块/6色托盘；False仍强制校验指令涉及的目标
TASK2_EXECUTE_ROBOT = True  # True：执行抓放；False：只拍照识别，不移动机器人
TASK2_REQUIRE_OFFSET_FILE = True  # 坐标逻辑调整后，未生成V2偏差文件时禁止真实抓放

# 三区拍照参数。None 表示沿用全局 MVS_EXPOSURE_TIME / MVS_GAIN。
TASK2_CARD_EXPOSURE_TIME = None
TASK2_CARD_GAIN = None
TASK2_CARD_IMAGE_MAX_EDGE = 2560  # 任务卡原图上传分辨率，避免小字在API前被过度缩放
TASK2_CARD_JPEG_QUALITY = 95
TASK2_BLOCK_EXPOSURE_TIME = None
TASK2_BLOCK_GAIN = None
TASK2_TRAY_EXPOSURE_TIME = None
TASK2_TRAY_GAIN = None

# XY标定原点和XY偏移只保存在data/task2_offsets_v2.json，不在config保留副本。
TASK2_BLOCK_PICK_Z = 180
TASK2_TRAY_PLACE_Z = 183
# 上述抓取/放置Z是在参考高度物块上标定的TCP高度。
# 任务书写明高度未知，以下30/28只是暂定值；实测高度需与参考Z一起标定。
TASK2_REFERENCE_BLOCK_HEIGHT_MM = 30.0
# 【现场必改】物块顶面相对托盘标定参考平面的高度(mm)。
# 新物理标定启用后：方块实际拍照Z = aubo_poses中的方块参考Z + 本值；托盘Z不变。
TASK2_BLOCK_PHOTO_HEIGHT_MM = 30.0
TASK2_BLOCK_HEIGHT_MM = {
    **{color: 30.0 for color in TASK2_TRAY_COLORS},
    **{color: 28.0 for color in TASK2_EXTRA_BLOCK_COLORS},
}
TASK2_ROTATION_ENABLED = True
# shortest：选择[-45°,45°)最短等价角；positive/negative：利用正方形90°等价，
# 强制所有非零RZ增量为正/负。学校受限工位使用negative，开阔赛场通常用shortest。
TASK2_ROTATION_DIRECTION = "shortest"
# 使用九点矩阵把边向量变到机器人XY后计算角度，无需人工猜图像角度正负。
TASK2_ROTATE_AT_CLEARANCE = True
# Z只从本文件读取；抓取/放置姿态分别取aubo_poses.json中的方块/托盘拍照姿态。
TASK2_LIFT_DISTANCE_MM = 80.0
TASK2_SETTLE_SECONDS = 0.8
TASK2_CARD_PROMPT = (
    "识别图片中的任务卡2装配指令。必须返回 JSON 数组，不要 Markdown。"
    "按原文顺序逐个展开抓放动作，每项仅含step、source_color、target_type、target_color。"
    "step从1连续编号；target_type只能是tray（托盘）或block（方块）。"
    "方块颜色包括红色、橙色、黄色、绿色、蓝色、紫色、青色、粉色、棕色；托盘仅前六色。"
    "前六次抓放是六色方块放到托盘，后续是青/粉/棕方块叠放到原文指定方块上。"
    "最后一句若含多个抓放关系，展开为多个动作。只识别原文，不补全、遗漏或重排动作。"
    "例如红块到黄托盘为{\"step\":1,\"source_color\":\"红色\",\"target_type\":\"tray\",\"target_color\":\"黄色\"}。"
    "无法辨认或内容不完整时返回空数组，不要猜测。"
)

# ==============================
# 5. 遨博 AUBO 机器人
# ==============================
ROBOT_IP = "192.168.34.10"                # 实际机器人 IP，接入实机需修改
# ROBOT_IP = "192.168.193.129"           # AUBO 虚拟机器人 IP（调试用）
ROBOT_PORT = 30004
ROBOT_NAME = "rob1"
ROBOT_USER = "aubo"
ROBOT_PASSWORD = "123456"
ROBOT_TIMEOUT_MS = 5000                  # SDK 请求超时（毫秒）

# 所有位姿统一为：[x,y,z,rx,ry,rz]，XYZ毫米，RX/RY/RZ弧度。（任务一位姿）
ROBOT_TARGET = None  # 运行时强制从data/aubo_poses.json读取

# 任务二三区拍照位；None时跳过移动，便于先测固定相机或调试图。
TASK2_CARD_VIEW_POSE = None   # 运行时强制从data/aubo_poses.json读取
TASK2_BLOCK_VIEW_POSE = None
TASK2_TRAY_VIEW_POSE = None

# 已实机测试的末端吸盘Tool IO；不是控制柜Standard DO。
ROBOT_VACUUM_ENABLED = True
TOOL_IO_VOLTAGE = 12
TOOL_IO_CONFIGURE_VOLTAGE = True  # False时跳过公共电压设置，仅控制外部供电的IO0/IO1
TOOL_IO_VENT_INDEX = 0
TOOL_IO_PUMP_INDEX = 1
TOOL_IO_VENT_OPEN_LEVEL = True
TOOL_IO_PUMP_ON_LEVEL = True
TOOL_IO_SUCTION_WAIT_SEC = 1.0
# 放置释放：先开泄压阀并短暂保持气泵运行，避免气路瞬态；停泵后继续泄压。
TOOL_IO_VENT_BEFORE_PUMP_OFF_SEC = 0.3
TOOL_IO_RELEASE_WAIT_SEC = 0.8
TOOL_IO_READBACK_TIMEOUT_SEC = 1.0
TOOL_IO_READBACK_INTERVAL_SEC = 0.05

# 放下物块后保持泄压阀开启，先低速竖直脱离，再关闭泄压阀并正常抬升。
TASK2_RELEASE_SLOW_LIFT_MM = 10.0
TASK2_RELEASE_SLOW_SPEED = 0.05
TASK2_RELEASE_SLOW_ACCELERATION = 0.10

ROBOT_SPEED = 0.3
ROBOT_ACCELERATION = 0.3
# 大范围转移的安全高度（mm）：拍照点之间等跨区域移动时，机械臂先升到该Z再平移/旋转，避免撞击台面物体。
# 经现场测试约 500mm；按实际工位抬高/降低此值。
ROBOT_SAFE_Z = 400
ROBOT_WAIT_TIMEOUT = 120.0               # 跨区域长距离运动到位超时（秒）
ROBOT_POSITION_TOLERANCE = 2.0           # 位置容差（mm）
ROBOT_ORIENTATION_TOLERANCE = 1.0        # 姿态容差（度）
