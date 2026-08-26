"""任务一：机器人到位 + 视觉识别 + 大模型理解 + 语音播报

分层约定：
    - 本文件（task1）只负责流程编排
    - 语音 / 视觉 / 大模型 / 机器人能力由 modules/ 下对应模块提供
    - 本文件不直接调用 FunASR / MVS / Qwen API / AUBO SDK 等底层库
    - 语音唤醒与任务派发由 main.py 负责，本文件不再重复唤醒

task1 依赖的模块接口约定：
    Voice.speak(text)              -> None   语音播报
    Vision.capture()               -> Path   单次采集图像（返回图片文件路径）
    LLM.identify_image(image)      -> str    视觉大模型理解
    Robot.move_to(pose_mm_rad)     -> bool   XYZ毫米、姿态弧度（连接失败返回False）
"""

import os
import sys

# 保证无论从项目根目录运行（python main.py）还是直接运行本文件（python task/task1.py），
# 都能 import 到根目录的 config 与 modules。
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import config


def task1_run(voice, vision, robot, llm):
    """任务一主流程。

    功能：
        机器人移动到任务卡拍照位，采集任务卡与现场画面，
        交由视觉大模型完成场景识别，最后语音播报识别结果。
        语音唤醒与命令派发已由 main.py 完成，本流程直接执行任务。

    输入：
        voice  -- Voice 模块实例，提供语音播报能力
        vision -- Vision 模块实例，提供图像采集能力
        robot  -- Robot 模块实例，提供机器人运动能力（无臂时自动跳过）
        llm    -- LLM 模块实例，提供视觉大模型理解能力

    输出：
        无。识别结果通过 voice.speak 直接播报，执行完毕后返回 main.py。
    """
    # ── 阶段 1：机器人移动到任务卡拍照位 ────────────────
    # 功能：机器人到位后相机才能拍到任务卡；连接/移动失败时兜底跳过，继续采集
    arrived = robot.move_to_safe(config.ROBOT_TARGET)
    if not arrived:
        voice.speak("机器人未到位，改为直接采集")

    # ── 阶段 2：单次采集图像 ──────────────────────────
    # 功能：采集任务卡与现场画面，返回图片文件路径
    image = vision.capture()

    # ── 阶段 3：视觉大模型理解 ────────────────────────
    # 功能：识别图像中任务卡内容，完成场景初始化解析
    try:
        result = llm.identify_image(image=image)
    except Exception as exc:
        print("大模型识别失败：" + str(exc))
        voice.speak("大模型识别失败")
        return

    # ── 阶段 4：语音播报结果 ──────────────────────────
    voice.speak(result)


if __name__ == "__main__":
    # 独立测试入口：绕过 main.py 的语音唤醒与派发，直接跑任务一流程。
    from modules.voice import Voice
    from modules.vision import Vision
    from modules.robot import Robot
    from modules.llm import LLM

    task1_run(
        voice=Voice(),
        vision=Vision(),
        robot=Robot(),
        llm=LLM(),
    )
