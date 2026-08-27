"""机器人模块：遨博 AUBO 机械臂连接与直线运动。

对外接口：
    Robot.move_to(pose_mm_rad) -> bool        移动到位姿（XYZ毫米，姿态弧度），返回是否到位
    Robot.move_to_safe(pose_mm_rad) -> bool   安全移动到目标位姿（先升安全Z，再平移/转向，后下降）
    Robot.disconnect()                         释放连接

连接失败不抛异常，置 available=False（无臂模式），供任务流程兜底跳过运动。
"""

import time
from typing import Optional

try:
    from pyaubo_sdk import RpcClient, StandardOutputRunState
except ImportError:
    RpcClient = None
    StandardOutputRunState = None

import config


def _format_pose(values) -> str:
    """控制台输出用，格式固定为 [x,y,z,rx,ry,rz]。"""
    parts = []
    for value in values:
        if float(value).is_integer():
            parts.append(str(int(value)))
        else:
            parts.append(str(value))
    return "[" + ",".join(parts) + "]"


class Robot:
    """封装 AUBO 机器人连接与运动控制。"""

    def __init__(self):
        self.rpc: Optional[object] = None
        self.robot_interface: Optional[object] = None
        self.available = False
        self._suction_configured = False
        self._connect()

    def _connect(self) -> None:
        """连接 AUBO 控制器。任何异常都降级为无臂模式，不中断主流程。"""
        if RpcClient is None:
            print("警告：没有安装 pyaubo_sdk，无法连接 AUBO 机器人，进入无臂模式。")
            return

        print("正在握手 AUBO ARCS 控制器（%s:%d）..." % (config.ROBOT_IP, config.ROBOT_PORT))
        try:
            rpc = RpcClient()
            rpc.setRequestTimeout(config.ROBOT_TIMEOUT_MS)
            rpc.connect(config.ROBOT_IP, config.ROBOT_PORT)
            try:
                rpc.login(config.ROBOT_USER, config.ROBOT_PASSWORD)
            except Exception:
                # 登录异常静默通过，与示例保持一致。
                pass

            self.robot_interface = rpc.getRobotInterface(config.ROBOT_NAME)
            self.rpc = rpc
            self.available = True
            print("机械臂实例 [%s] 已就绪。" % config.ROBOT_NAME)
        except Exception as exc:
            print("机械臂通信初始化失败：%s（将以无臂模式运行）" % exc)

    def get_current_pose(self):
        """读取基座坐标系TCP位姿：[X/Y/Z毫米，RX/RY/RZ弧度]。"""
        if not self.available or self.robot_interface is None:
            raise RuntimeError("机器人未连接，无法读取当前基座TCP位姿。")
        pose = self.robot_interface.getRobotState().getTcpPose()
        if not isinstance(pose, (list, tuple)) or len(pose) < 6:
            raise RuntimeError("getTcpPose返回格式异常：%r" % (pose,))
        result = [float(pose[0]) * 1000.0, float(pose[1]) * 1000.0, float(pose[2]) * 1000.0,
                  float(pose[3]), float(pose[4]), float(pose[5])]
        print("AUBO当前基座TCP位姿：" + _format_pose(result))
        return result

    def move_to(self, pose_mm_rad) -> bool:
        """移动到目标位姿（XYZ毫米，RX/RY/RZ弧度），返回是否到位。"""
        if not self.available or self.robot_interface is None:
            print("机器人未连接，跳过运动执行。")
            return False

        if not isinstance(pose_mm_rad, (list, tuple)) or len(pose_mm_rad) != 6:
            print("机器人目标位姿必须是六个数值。")
            return False
        pose_text = _format_pose(pose_mm_rad)
        print("准备移动机器人到位姿：" + pose_text)
        try:
            target_x, target_y, target_z = [value / 1000.0 for value in pose_mm_rad[:3]]
            target_rx, target_ry, target_rz = [float(value) for value in pose_mm_rad[3:]]
            target_pose = [target_x, target_y, target_z, target_rx, target_ry, target_rz]

            # AUBO签名：moveLine(pose, acceleration, velocity, blend_radius, duration)。
            # duration=0时由速度/加速度自动计算轨迹时间；True会被当作1秒而不是阻塞开关。
            result = self.robot_interface.getMotionControl().moveLine(
                target_pose,
                config.ROBOT_ACCELERATION,
                config.ROBOT_SPEED,
                0.0,
                0.0,
            )
            print("AUBO moveLine SDK返回：%r" % result)
            # 13表示目标路径过短、控制器忽略请求；仍由实际TCP到位检查决定成功与否。
            if result not in (None, 0, 13):
                raise RuntimeError("moveLine返回错误码：%r" % result)
            arrived = self._wait_until_arrives(target_pose)
            if arrived:
                print("机器人已移动到位姿：" + pose_text)
            else:
                print("机器人未在限定时间内到达位姿：" + pose_text)
            return arrived
        except Exception as exc:
            print("机器人移动失败：" + str(exc))
            return False

    def move_to_safe(self, pose_mm_rad, safe_z=None) -> bool:
        """安全移动到大范围目标位姿（XYZ毫米，RX/RY/RZ弧度）。

        动作序列：原地升到安全Z -> 安全高度平移到目标XY并转到目标姿态 -> 下降到目标位姿。
        仅用于拍照点之间等跨区域大范围转移，避免低空水平移动撞击台面物体；
        抓放动作仍走 pick_and_place。
        """
        if not self.available or self.robot_interface is None:
            print("机器人未连接，跳过安全运动。")
            return False
        if not isinstance(pose_mm_rad, (list, tuple)) or len(pose_mm_rad) != 6:
            print("机器人安全移动目标位姿必须是六个数值。")
            return False

        target = [float(value) for value in pose_mm_rad]
        safe = float(config.ROBOT_SAFE_Z if safe_z is None else safe_z)
        try:
            current = self.get_current_pose()
        except Exception as exc:
            print("读取当前位姿失败，无法规划安全路径：" + str(exc))
            return False

        # 若当前已高于安全Z，则不下降，用当前高度作为过渡高度。
        lift_z = max(float(current[2]), safe)
        rise = [current[0], current[1], lift_z, current[3], current[4], current[5]]
        hover = [target[0], target[1], lift_z, target[3], target[4], target[5]]
        descend = [target[0], target[1], target[2], target[3], target[4], target[5]]

        for stage, pose in (("上升", rise), ("平移旋转", hover), ("下降", descend)):
            if not self.move_to(pose):
                print("安全移动在%s阶段失败，已终止。" % stage)
                return False
        return True

    def _wait_until_arrives(self, target_pose) -> bool:
        """轮询 TCP 位姿，位置和姿态都接近目标位姿后才认为移动到位。"""
        start_time = time.monotonic()
        last_report_time = 0.0
        while True:
            current_pose = self.robot_interface.getRobotState().getTcpPose()
            error_mm = self._position_error_mm(current_pose, target_pose)
            orientation_error = self._orientation_error_degrees(current_pose, target_pose)
            elapsed = time.monotonic() - start_time

            if elapsed - last_report_time >= 1.0:
                print("等待机器人到位，位置误差 %.2f mm，姿态误差 %.2f 度" % (error_mm, orientation_error))
                last_report_time = elapsed

            if error_mm <= config.ROBOT_POSITION_TOLERANCE and orientation_error <= config.ROBOT_ORIENTATION_TOLERANCE:
                print("机器人到位，最终位置误差 %.2f mm，姿态误差 %.2f 度" % (error_mm, orientation_error))
                return True
            if elapsed >= config.ROBOT_WAIT_TIMEOUT:
                print("等待机器人到位超时，最终位置误差 %.2f mm，姿态误差 %.2f 度" % (error_mm, orientation_error))
                return False
            time.sleep(0.2)

    @staticmethod
    def _position_error_mm(current_pose, target_pose) -> float:
        """getTcpPose 返回米，转成毫米误差。"""
        dx = (current_pose[0] - target_pose[0]) * 1000.0
        dy = (current_pose[1] - target_pose[1]) * 1000.0
        dz = (current_pose[2] - target_pose[2]) * 1000.0
        return (dx * dx + dy * dy + dz * dz) ** 0.5

    @staticmethod
    def _orientation_error_degrees(current_pose, target_pose) -> float:
        """getTcpPose 返回弧度，转成角度误差。"""
        scale = 180.0 / 3.141592653589793
        drx = (current_pose[3] - target_pose[3]) * scale
        dry = (current_pose[4] - target_pose[4]) * scale
        drz = (current_pose[5] - target_pose[5]) * scale
        return (drx * drx + dry * dry + drz * drz) ** 0.5

    @staticmethod
    def _require_zero_result(result, operation):
        if result not in (None, 0):
            raise RuntimeError("%s返回错误码：%r" % (operation, result))

    @staticmethod
    def _wait_for_io_value(reader, expected, label):
        """等待控制器状态刷新，避免把正常的网络/IO延迟判作失败。"""
        timeout = max(0.0, float(config.TOOL_IO_READBACK_TIMEOUT_SEC))
        interval = max(0.01, float(config.TOOL_IO_READBACK_INTERVAL_SEC))
        deadline = time.monotonic() + timeout
        actual = reader()
        while actual != expected and time.monotonic() < deadline:
            time.sleep(interval)
            actual = reader()
        if actual != expected:
            raise RuntimeError("%s回读不一致：期望%s，实际%s。" % (label, expected, actual))

    def configure_suction(self) -> bool:
        """配置已实机验证的末端Tool IO：12V、输出方向、手动输出模式。"""
        if not self.available or self.robot_interface is None:
            print("机器人未连接，无法配置吸盘Tool IO。")
            return False
        try:
            io_control = self.robot_interface.getIoControl()
            if config.TOOL_IO_CONFIGURE_VOLTAGE:
                self._require_zero_result(
                    io_control.setToolVoltageOutputDomain(int(config.TOOL_IO_VOLTAGE)),
                    "设置工具端电压",
                )
                self._wait_for_io_value(
                    lambda: int(io_control.getToolVoltageOutputDomain()),
                    int(config.TOOL_IO_VOLTAGE),
                    "工具端电压",
                )
            else:
                actual_voltage = io_control.getToolVoltageOutputDomain()
                print("跳过工具端公共电压设置，当前回读=%sV；仅控制外部供电的Tool IO。" %
                      actual_voltage)
            for index, label in (
                (config.TOOL_IO_VENT_INDEX, "泄压阀"),
                (config.TOOL_IO_PUMP_INDEX, "真空泵"),
            ):
                self._require_zero_result(
                    io_control.setToolIoInput(int(index), False),
                    "配置%s Tool IO为输出" % label,
                )
                if hasattr(io_control, "isToolIoInput") and io_control.isToolIoInput(int(index)):
                    raise RuntimeError("%s TOOL_IO[%d]仍处于输入模式。" % (label, index))
                if hasattr(io_control, "setToolDigitalOutputRunstate"):
                    if StandardOutputRunState is None:
                        raise RuntimeError("SDK缺少StandardOutputRunState，无法切换手动输出模式。")
                    manual_state = StandardOutputRunState.__members__["None"]
                    self._require_zero_result(
                        io_control.setToolDigitalOutputRunstate(int(index), manual_state),
                        "设置%s为手动输出模式" % label,
                    )
                    if hasattr(io_control, "getToolDigitalOutputRunstate"):
                        actual_state = io_control.getToolDigitalOutputRunstate(int(index))
                        if int(actual_state) != int(manual_state):
                            raise RuntimeError(
                                "%s TOOL_IO[%d]输出模式不是None：%s" %
                                (label, index, actual_state)
                            )
            self._suction_configured = True
            voltage_text = ("SDK设置%dV" % config.TOOL_IO_VOLTAGE
                            if config.TOOL_IO_CONFIGURE_VOLTAGE else "跳过公共电压设置")
            print("吸盘末端Tool IO配置完成：%s，泄压阀IO%d，气泵IO%d。" %
                  (voltage_text, config.TOOL_IO_VENT_INDEX, config.TOOL_IO_PUMP_INDEX))
            return True
        except Exception as exc:
            self._suction_configured = False
            print("吸盘末端Tool IO配置失败：" + str(exc))
            return False

    def _write_tool_output(self, channel: int, value: bool, label: str) -> None:
        io_control = self.robot_interface.getIoControl()
        result = io_control.setToolDigitalOutput(int(channel), bool(value))
        self._require_zero_result(result, "写入%s" % label)
        print("AUBO末端TOOL_IO[%d] <- %s（%s），SDK返回：%s" %
              (channel, bool(value), label, result))
        self._wait_for_io_value(
            lambda: bool(io_control.getToolDigitalOutput(int(channel))),
            bool(value),
            label + "输出",
        )

    def set_suction(self, enabled: bool) -> bool:
        """True吸取；False停泵、短暂泄压，然后关闭泄压阀。"""
        if not config.ROBOT_VACUUM_ENABLED:
            print("吸盘控制已被ROBOT_VACUUM_ENABLED=False禁用。")
            return False
        if not self.available or self.robot_interface is None:
            print("机器人未连接，无法控制吸盘。")
            return False
        if not self._suction_configured and not self.configure_suction():
            return False
        vent_open = bool(config.TOOL_IO_VENT_OPEN_LEVEL)
        pump_on = bool(config.TOOL_IO_PUMP_ON_LEVEL)
        try:
            if enabled:
                self._write_tool_output(config.TOOL_IO_VENT_INDEX, not vent_open, "泄压阀关闭")
                self._write_tool_output(config.TOOL_IO_PUMP_INDEX, pump_on, "真空泵启动")
                time.sleep(max(0.0, float(config.TOOL_IO_SUCTION_WAIT_SEC)))
                print("吸盘已开启。")
            else:
                self._write_tool_output(config.TOOL_IO_PUMP_INDEX, not pump_on, "真空泵停止")
                self._write_tool_output(config.TOOL_IO_VENT_INDEX, vent_open, "泄压阀开启")
                time.sleep(max(0.0, float(config.TOOL_IO_RELEASE_WAIT_SEC)))
                self._write_tool_output(config.TOOL_IO_VENT_INDEX, not vent_open, "泄压阀关闭")
                print("吸盘已关闭并完成泄压。")
            return True
        except Exception as exc:
            print("吸盘控制失败：" + str(exc))
            return False

    def vacuum_on(self) -> bool:
        """关闭泄压阀并打开负压气泵。"""
        return self.set_suction(True)

    def vacuum_off(self) -> bool:
        """关闭气泵，短暂开启泄压阀后恢复关闭。"""
        return self.set_suction(False)

    def pick_and_place(self, pick_pose, place_pose, lift_mm=None) -> bool:
        """上方接近、下降吸取、原地抬升、水平移动、原地下降并释放。"""
        lift = float(config.TASK2_LIFT_DISTANCE_MM if lift_mm is None else lift_mm)
        pick, place = list(map(float, pick_pose)), list(map(float, place_pose))
        pick_above, place_above = pick.copy(), place.copy()
        pick_above[2] += lift
        place_above[2] += lift
        if not self.move_to(pick_above) or not self.move_to(pick):
            return False
        if not self.vacuum_on():
            return False
        if not self.move_to(pick_above):
            self.vacuum_off()
            return False
        if not self.move_to(place_above) or not self.move_to(place):
            self.vacuum_off()
            return False
        if not self.vacuum_off():
            return False
        return self.move_to(place_above)

    def disconnect(self) -> None:
        """释放机器人连接。"""
        if self.rpc is not None and hasattr(self.rpc, "disconnect"):
            try:
                self.rpc.disconnect()
            except Exception as exc:
                print("机器人断开连接失败：" + str(exc))
        self.available = False
