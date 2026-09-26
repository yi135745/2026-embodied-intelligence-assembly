import unittest
from unittest.mock import MagicMock, patch

from tools.task2._support import move_to_scene_view


class Task2ToolSupportTest(unittest.TestCase):
    @patch("tools.task2._support.Robot")
    @patch("tools.task2._support.get_task2_block_view_pose")
    @patch("tools.task2._support.load_aubo_pose_records")
    def test_block_scene_uses_height_adjusted_pose(self, load_records, adjust, robot_type):
        recorded = [1, 2, 3, 4, 5, 6]
        adjusted = [1, 2, 33, 4, 5, 6]
        load_records.return_value = {"TASK2_BLOCK_VIEW_POSE": recorded}
        adjust.return_value = adjusted
        robot = MagicMock(available=True)
        robot.move_to_safe.return_value = True
        robot_type.return_value = robot
        move_to_scene_view("block")
        adjust.assert_called_once_with(recorded)
        robot.move_to_safe.assert_called_once_with(adjusted)
        robot.disconnect.assert_called_once()

    @patch("tools.task2._support.Robot")
    @patch("tools.task2._support.get_task2_block_view_pose")
    @patch("tools.task2._support.load_aubo_pose_records")
    def test_card_scene_uses_recorded_pose_directly(self, load_records, adjust, robot_type):
        recorded = [1, 2, 3, 4, 5, 6]
        load_records.return_value = {"TASK2_CARD_VIEW_POSE": recorded}
        robot = MagicMock(available=True)
        robot.move_to_safe.return_value = True
        robot_type.return_value = robot
        move_to_scene_view("card")
        adjust.assert_not_called()
        robot.move_to_safe.assert_called_once_with(recorded)


if __name__ == "__main__":
    unittest.main()
