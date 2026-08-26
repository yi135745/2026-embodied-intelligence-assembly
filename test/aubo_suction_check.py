"""AUBO吸盘独立实机测试，不调用任何机器人运动接口。

默认静态读取：
    python test/aubo_suction_check.py

原地启停测试：
    1. 在config.py设置SUCTION_ENABLE_OUTPUT_TEST=True
    2. 运行 python test/aubo_suction_check.py --run
    3. 按提示输入1，分步观察吸盘吸取和泄压动作
"""

import argparse
import math
import sys
from pathlib import Path

# 路径标注：直接运行本文件时，将项目根目录加入模块搜索路径。
PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

import config  # noqa: E402
from modules.robot import Robot  # noqa: E402


def read_tool_io(robot):
    """仅读取工具IO，不设置电压、方向或输出。"""
    io_control = robot.robot_interface.getIoControl()
    print("工具端电压域：" + repr(io_control.getToolVoltageOutputDomain()))
    print("工具数字输出数量：" + repr(io_control.getToolDigitalOutputNum()))
    print("工具数字输出位图：" + repr(io_control.getToolDigitalOutputs()))
    for index, label in (
        (config.TOOL_IO_VENT_INDEX, "泄压阀"),
        (config.TOOL_IO_PUMP_INDEX, "真空泵"),
    ):
        is_input = bool(io_control.isToolIoInput(int(index)))
        output = bool(io_control.getToolDigitalOutput(int(index)))
        runstate = (
            io_control.getToolDigitalOutputRunstate(int(index))
            if hasattr(io_control, "getToolDigitalOutputRunstate") else "SDK不支持读取"
        )
        print(
            "%s TOOL_IO[%d]：方向=%s，输出电平=%s，输出运行状态=%s"
            % (label, index, "输入" if is_input else "输出", output, runstate)
        )


def pose_delta(before, after):
    position_error = math.sqrt(sum((after[i] - before[i]) ** 2 for i in range(3)))
    orientation_error = math.sqrt(sum((after[i] - before[i]) ** 2 for i in range(3, 6)))
    return position_error, orientation_error


def wait_for_one(prompt):
    """等待人工检查；只有输入1才继续，其他输入均终止测试。"""
    value = input(prompt + "（输入1继续，其他内容终止）：").strip()
    if value != "1":
        raise RuntimeError("人工终止吸盘测试。")


def main():
    parser = argparse.ArgumentParser(description="AUBO吸盘静态读取与原地启停测试")
    parser.add_argument("--run", action="store_true", help="实际写IO并执行吸盘开启/关闭")
    args = parser.parse_args()

    print("本脚本不会调用moveLine、moveJoint或任何机器人运动接口。")
    print("连接目标：%s:%d，机器人实例：%s" % (config.ROBOT_IP, config.ROBOT_PORT, config.ROBOT_NAME))
    robot = Robot()
    suction_may_be_on = False
    try:
        if not robot.available:
            print("机器人连接失败，无法读取工具IO。")
            return 1
        before_pose = robot.get_current_pose()
        print("测试前基座系TCP [mm,mm,mm,rad,rad,rad]：" + repr(before_pose))
        print("\n--- 静态读取测试 ---")
        read_tool_io(robot)
        print("静态读取测试完成，未写入任何工具IO。")

        if not args.run:
            print("如需原地启停测试，请完成config安全开关后添加--run。")
            return 0
        if not config.SUCTION_ENABLE_OUTPUT_TEST:
            print("config.SUCTION_ENABLE_OUTPUT_TEST=False，已拒绝写IO。")
            return 2

        print("\n--- 原地启停测试 ---")
        print("即将设置工具端%dV，并操作泄压阀IO[%d]和真空泵IO[%d]。" % (
            config.TOOL_IO_VOLTAGE, config.TOOL_IO_VENT_INDEX, config.TOOL_IO_PUMP_INDEX
        ))
        confirmation = input("确认机械臂保持原地、吸盘周围安全。请输入“确认吸盘测试”：").strip()
        if confirmation != "确认吸盘测试":
            print("确认文本不匹配，取消测试。")
            return 3

        print("开启吸盘，请人工观察真空泵和吸取状态。")
        suction_may_be_on = True
        if not robot.set_suction(True):
            raise RuntimeError("吸盘开启调用失败。")
        print("\n--- 吸盘开启后的当前状态 ---")
        read_tool_io(robot)
        wait_for_one("请确认泄压阀已关闭、负压气泵持续运行")

        print("关闭吸盘并泄压，请人工观察释放状态。")
        if not robot.set_suction(False):
            raise RuntimeError("吸盘关闭调用失败。")
        suction_may_be_on = False

        print("\n--- 关闭后的静态读取 ---")
        read_tool_io(robot)
        wait_for_one("请确认气泵已停止，泄压动作已经完成")
        after_pose = robot.get_current_pose()
        position_error, orientation_error = pose_delta(before_pose, after_pose)
        print("测试后基座系TCP [mm,mm,mm,rad,rad,rad]：" + repr(after_pose))
        print("原地检查：位置变化 %.6f mm，姿态变化 %.9f rad" % (position_error, orientation_error))
        print("吸盘原地启停测试通过。")
        return 0
    except Exception as exc:
        print("吸盘测试失败：" + str(exc))
        return 1
    finally:
        if suction_may_be_on and robot.available:
            print("异常清理：尝试停止真空泵并泄压。")
            robot.set_suction(False)
        robot.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
