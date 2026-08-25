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
SUCTION_ENABLE_OUTPUT_TEST = False
SUCTION_HOLD_SECONDS = 3.0

# 独立API连通测试。
API_KEY = ""  # 用环境变量 DASHSCOPE_API_KEY 提供，勿提交密钥
API_URL = "https://dashscope.aliyuncs.com/compatible-mode/v1/chat/completions"
API_MODEL = "qwen-vl-max"
API_TEST_IMAGE = TEST_DIR / "test.jpg"
