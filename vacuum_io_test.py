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
        robot.print_tool_io_status("连接后的初始状态")
        input("确认机器人已完全上电，按回车：按官方接口配置Tool IO...")
        if not robot.configure_suction():
            raise RuntimeError("Tool IO配置失败；请根据上方具体步骤排查。")
        robot.print_tool_io_status("配置后、动作前")

        input("按回车：关闭泄压阀并开启气泵3秒...")
        if not robot.vacuum_on():
            robot.print_tool_io_status("气泵开启失败后的状态")
            raise RuntimeError("气泵开启失败；请保留上方返回码、方向、Runstate和位图。")
        robot.print_tool_io_status("气泵开启后的状态")
        time.sleep(3.0)
        input("按回车：关闭气泵并开启泄压阀...")
        if not robot.vacuum_off():
            robot.print_tool_io_status("吸盘释放失败后的状态")
            raise RuntimeError("吸盘释放失败；请保留上方返回码、方向、Runstate和位图。")
        robot.print_tool_io_status("测试结束状态")
        print("IO测试完成。请对照气泵声音、末端Tool IO状态和控制台回读结果。")
    finally:
        if robot.available and robot._suction_configured:
            print("退出前执行一次停泵和泄压。")
            robot.vacuum_off()
        robot.disconnect()


if __name__ == "__main__":
    main()
