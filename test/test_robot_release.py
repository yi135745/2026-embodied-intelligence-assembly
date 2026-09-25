import unittest
from unittest.mock import patch

import config
from modules.robot import Robot


class FakeReleaseRobot(Robot):
    def __init__(self):
        self.available = True
        self.robot_interface = object()
        self._suction_configured = True
        self.outputs = []

    def _write_tool_output(self, channel, value, label):
        self.outputs.append((channel, value, label))


class FakePickRobot:
    def __init__(self):
        self.moves = []
        self.release_calls = []
        self.vent_closed = False

    def get_current_pose(self):
        return [0.0, 0.0, 400.0, 0.0, 0.0, 0.0]

    def move_to(self, pose, speed=None, acceleration=None):
        self.moves.append((list(pose), speed, acceleration))
        return True

    def vacuum_on(self):
        return True

    def vacuum_off(self):
        return True

    def set_suction(self, enabled, keep_vent_open=False):
        self.release_calls.append((enabled, keep_vent_open))
        return True

    def _close_release_vent(self):
        self.vent_closed = True
        return True


class RobotReleaseTests(unittest.TestCase):
    @patch("modules.robot.time.sleep")
    def test_release_opens_vent_before_stopping_pump(self, sleep):
        robot = FakeReleaseRobot()
        self.assertTrue(robot.set_suction(False, keep_vent_open=True))
        self.assertEqual(robot.outputs[0][2], "泄压阀开启")
        self.assertEqual(robot.outputs[1][2], "真空泵停止")
        self.assertNotIn("泄压阀关闭", [item[2] for item in robot.outputs])
        sleep.assert_any_call(float(config.TOOL_IO_VENT_BEFORE_PUMP_OFF_SEC))
        sleep.assert_any_call(float(config.TOOL_IO_RELEASE_WAIT_SEC))

    def test_pick_and_place_slowly_lifts_ten_mm_before_closing_vent(self):
        robot = FakePickRobot()
        pick = [10.0, 20.0, 100.0, 0.0, 0.0, 0.0]
        place = [30.0, 40.0, 120.0, 0.0, 0.0, 0.0]
        self.assertTrue(Robot.pick_and_place(robot, pick, place, lift_mm=20.0))
        self.assertEqual(robot.release_calls[-1], (False, True))
        slow_moves = [item for item in robot.moves
                      if item[1] == config.TASK2_RELEASE_SLOW_SPEED]
        self.assertEqual(len(slow_moves), 1)
        self.assertEqual(slow_moves[0][0][2], 130.0)
        self.assertTrue(robot.vent_closed)


if __name__ == "__main__":
    unittest.main()
