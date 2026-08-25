"""程序入口：语音唤醒后派发到任务一 / 任务二。

职责：
    - 只负责「唤醒 -> 识别任务提示词 -> 派发到对应任务流程」，
      不包含任何具体任务逻辑（任务逻辑在 task/task1.py 与 task/task2.py 中）。
    - 唤醒词（小具同学）与任务提示词（任务一 / 任务二）分开两轮识别，
      唤醒由 main.py 完成，task 流程内不再重复唤醒。
"""

import config

from modules.voice import Voice
from modules.vision import Vision
from modules.robot import Robot
from modules.llm import LLM

from task.task1 import task1_run

# 任务二尚未实现（task/task2.py 为空），先用占位方式处理。
try:
    from task.task2 import task2_run
except ImportError:
    task2_run = None


def main():
    voice = Voice()
    vision = Vision()
    robot = Robot()
    llm = LLM()

    print("系统启动")
    voice.speak("系统已启动，请呼叫" + config.WAKE_WORD)

    while True:
        # ── 阶段 1：语音唤醒 ──────────────────────────────
        # 等待唤醒词，命中才继续；听到退出指令则结束程序。
        if not voice.wake(wake_word=config.WAKE_WORD):
            voice.speak("收到，系统退出")
            break

        # ── 阶段 2：播报就绪，等待任务提示词 ──────────────
        voice.speak(config.READY_REPLY)
        while True:
            command = voice.listen()
            if not command:
                continue
            print("听到：" + command)

            if voice.is_exit(command):
                voice.speak("收到，系统退出")
                return

            if  config.TASK1_COMMAND in command:
                task1_run(voice, vision, robot, llm)
                voice.speak(config.RETURN_REPLY)
                break

            if config.TASK2_COMMAND in command:
                if task2_run is None:
                    print("任务二尚未实现")
                    voice.speak("任务二尚未实现")
                else:
                    task2_run(voice, vision, robot, llm)
                    voice.speak(config.RETURN_REPLY)
                break

            # ── 未识别的提示词，提示后继续监听 ────────────
            voice.speak(
                "没有识别到有效指令，请说" + config.TASK1_COMMAND + "或" + config.TASK2_COMMAND
            )


if __name__ == "__main__":
    main()
