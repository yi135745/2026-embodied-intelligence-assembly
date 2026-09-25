"""程序入口：语音唤醒后派发到任务一 / 任务二。

职责：
    - 只负责「唤醒 -> 识别任务提示词 -> 派发到对应任务流程」，
      不包含任何具体任务逻辑（任务逻辑在 task/task1.py 与 task/task2.py 中）。
    - 唤醒词（小具同学）与任务提示词（任务一 / 任务二）分开两轮识别，
      唤醒由 main.py 完成，task 流程内不再重复唤醒。
"""

import config

from modules.camera import Camera
from modules.robot import Robot
from modules.interpreter import Interpreter
from runtime.site_data import apply_aubo_pose_records

from task.task1 import task1_run

from task.task2 import task2_run


def main():
    from modules.voice import Voice

    apply_aubo_pose_records()
    voice = Voice()
    camera = Camera()
    robot = Robot()
    interpreter = Interpreter()

    print("系统启动")
    try:
        voice.speak("系统已启动，请呼叫" + config.WAKE_WORD)
        run_tasks(voice, camera, robot, interpreter)
    finally:
        robot.disconnect()


def run_tasks(voice, camera, robot, interpreter):
    """任务返回成功才计入本轮；失败后清空本轮状态，等待裁判重新发卡。"""
    completed = set()

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

            task_id = (1 if config.TASK1_COMMAND in command else
                       2 if config.TASK2_COMMAND in command else None)
            if task_id is not None:
                if task_id in completed:
                    voice.speak("本项任务已完成，请执行另一个任务")
                    break
                if task_id == 1:
                    success = task1_run(voice, camera, robot, interpreter) is True
                else:
                    result = task2_run(voice, camera, robot, interpreter)
                    success = bool(result and result.get("status") == "completed")
                if success:
                    completed.add(task_id)
                    voice.speak(config.RETURN_REPLY)
                    if completed == {1, 2}:
                        voice.speak("两个任务均已完成")
                        return
                else:
                    completed.clear()
                    voice.speak("本轮未完成；重试前请由裁判重新发放任务卡")
                break

            # ── 未识别的提示词，提示后继续监听 ────────────
            voice.speak(
                "没有识别到有效指令，请说" + config.TASK1_COMMAND + "或" + config.TASK2_COMMAND
            )


if __name__ == "__main__":
    main()
