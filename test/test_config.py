"""test/ 下脚本专用配置，生产代码不得引用。"""

from pathlib import Path


# 路径标注：测试目录与项目根目录，供测试文件直接运行时定位资源。
TEST_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = TEST_DIR.parent

# AUBO独立运动测试。默认关闭危险动作；只读连接测试不受影响。
AUBO_ENABLE_MOTION = True
AUBO_TARGET_POSE = [-556.24, 33.41, 371.94, 0.168, -0.061, 0.426]
AUBO_TARGET_ANGLE_UNIT = "rad"  # 位置mm，姿态rad
AUBO_SPEED = 0.03
AUBO_ACCELERATION = 0.03

# 吸盘原地启停实机测试。默认只允许读取IO；改为True后仍需命令--run和现场文字确认。
SUCTION_ENABLE_OUTPUT_TEST = True
SUCTION_HOLD_SECONDS = 3.0

# 外部格式样本，仅用于解析器单元测试。
CALIBRATION_SAMPLE_FILE = Path(r"E:\暂存\2026具身智能比赛\标定格式式样.xml")

# 独立API连通测试。
API_KEY = "sk-ws-H.EYHLPXH.NIq5.MEUCIQDvTpmg1zJnmJmzpnbNMyyo26ZhXIgRE5PaDe7cyHT6cwIge3AWv8W1hMvLjz_h1uG68A-CBmt5F0iXP2XqK99u-XU"
API_URL = "https://dashscope.aliyuncs.com/compatible-mode/v1/chat/completions"
API_MODEL = "qwen-vl-max"
API_TEST_IMAGE = TEST_DIR / "test.jpg"
