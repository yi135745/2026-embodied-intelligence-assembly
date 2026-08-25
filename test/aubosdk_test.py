from pyaubo_sdk import *

import os
import sys

# 保证无论从项目根目录运行（python main.py）还是直接运行本文件（python task/task1.py），
# 都能 import 到根目录的 config 与 modules。
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

ROBOT_IP = "192.168.1.10"
ROBOT_PORT = 30004


def main():
    print("开始测试 AUBO SDK")

    robot = Robot()

    print("尝试连接:")
    print(f"{ROBOT_IP}:{ROBOT_PORT}")

    ret = robot.connect(ROBOT_IP, ROBOT_PORT)

    print("connect返回:", ret)

    connected = robot.hasConnected()

    print("hasConnected:", connected)

    if not connected:
        print("连接失败")
        return

    print("连接成功")

    # 尝试获取机器人状态
    try:
        state = robot.getRobotState()
        print("机器人状态:")
        print(state)
    except Exception as e:
        print("读取状态失败:", e)

    robot.disconnect()
    print("释放连接")


if __name__ == "__main__":
    main()