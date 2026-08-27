"""AUBO气泵与泄压阀独立测试，不执行任何机器人运动。"""

import argparse
import time

import config
from modules.robot import Robot


def main():
    parser = argparse.ArgumentParser(description="AUBO气泵/泄压阀独立测试，不移动机械臂")
    parser.add_argument("--skip-voltage", action="store_true",
                        help="跳过工具端公共电压设置，仅测试外部供电的IO0/IO1")
    args = parser.parse_args()
    if args.skip_voltage:
        config.TOOL_IO_CONFIGURE_VOLTAGE = False
    print("工具端电压模式：%s" %
          (("SDK设置%dV" % config.TOOL_IO_VOLTAGE)
           if config.TOOL_IO_CONFIGURE_VOLTAGE else "跳过设置（外部供电测试）"))
    print("气泵通道：末端TOOL_IO[%d]，高有效=%s" %
          (config.TOOL_IO_PUMP_INDEX, config.TOOL_IO_PUMP_ON_LEVEL))
    print("泄压通道：末端TOOL_IO[%d]，高有效=%s" %
          (config.TOOL_IO_VENT_INDEX, config.TOOL_IO_VENT_OPEN_LEVEL))
    robot = Robot()
    if not robot.available:
        raise RuntimeError("AUBO未连接，无法测试IO。")
    try:
        input("确认周围安全后按回车：开启气泵3秒...")
        if not robot.vacuum_on():
            raise RuntimeError("气泵开启调用失败。")
        time.sleep(3.0)
        input("按回车：关闭气泵并开启泄压阀...")
        if not robot.vacuum_off():
            raise RuntimeError("吸盘释放调用失败。")
        print("IO测试完成。请对照气泵声音、末端Tool IO状态和控制台回读结果。")
    finally:
        robot.disconnect()


if __name__ == "__main__":
    main()
