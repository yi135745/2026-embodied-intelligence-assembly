"""任务二纯代码离线测试，不连接相机、麦克风、API 或机器人。"""

import json
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

import os
import sys

# 保证无论从项目根目录运行（python main.py）还是直接运行本文件（python task/task1.py），
# 都能 import 到根目录的 config 与 modules。
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import config
from modules.robot import Robot
from modules.task2_vision import ColorObjectDetector, CoordinateTransformer, validate_six_colors
from task.task2 import task2_run


COLORS_BGR = {
    "红色": (0, 0, 255), "橙色": (0, 140, 255), "黄色": (0, 255, 255),
    "绿色": (0, 180, 0), "蓝色": (255, 0, 0), "紫色": (180, 0, 180),
}


def make_six_color_image(path):
    image = np.full((500, 900, 3), 245, np.uint8)
    for index, color in enumerate(config.TASK2_BLOCK_HSV_RANGES):
        x = 40 + index * 140
        lower, upper = config.TASK2_BLOCK_HSV_RANGES[color][0]
        hsv_color = np.uint8([[[sum(pair) // 2 for pair in zip(lower, upper)]]])
        bgr_color = tuple(int(value) for value in cv2.cvtColor(hsv_color, cv2.COLOR_HSV2BGR)[0, 0])
        cv2.rectangle(image, (x, 170), (x + 80, 250), bgr_color, -1)
    assert cv2.imwrite(str(path), image)


class FakeVoice:
    def __init__(self): self.messages = []
    def speak(self, text): self.messages.append(text)


class FakeVision:
    def __init__(self, image): self.image = image
    def capture(self, output_name=None, debug_image=None):
        del output_name, debug_image
        return self.image


class FakeLLM:
    def parse_task2_card(self, image):
        del image
        colors = list(config.TASK2_HSV_RANGES)
        return [{"step": i+1, "block_color": c, "tray_color": colors[-i-1]} for i, c in enumerate(colors)]


class Task2Test(unittest.TestCase):
    def test_visionmaster_nine_point_matrix(self):
        calibration = Path(__file__).parents[1] / config.TASK2_CALIBRATION_FILE
        transformer = CoordinateTransformer(calibration)
        world_x, world_y = transformer.pixel_to_world(1243.6643, 799.7326)
        self.assertAlmostEqual(0.0, world_x, delta=0.2)
        self.assertAlmostEqual(0.0, world_y, delta=0.2)

    def test_pick_and_place_uses_vertical_clearance(self):
        robot = Robot.__new__(Robot)
        moves = []
        robot.move_to = lambda pose: moves.append(list(pose)) or True
        robot.vacuum_on = lambda: True
        robot.vacuum_off = lambda: True
        self.assertTrue(robot.pick_and_place([10, 20, 30, 1, 2, 3], [100, 200, 40, 1, 2, 3], 50))
        self.assertEqual([10, 20, 80, 1, 2, 3], moves[0])
        self.assertEqual([10, 20, 30, 1, 2, 3], moves[1])
        self.assertEqual([10, 20, 80, 1, 2, 3], moves[2])
        self.assertEqual([100, 200, 90, 1, 2, 3], moves[3])
        self.assertEqual([100, 200, 40, 1, 2, 3], moves[4])
        self.assertEqual([100, 200, 90, 1, 2, 3], moves[5])

    def test_color_detection_and_flow(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            image = root / "six.jpg"
            make_six_color_image(image)
            targets, _ = ColorObjectDetector().detect(image, "方块")
            validate_six_colors(targets, "方块")
            old_output = config.TASK2_OUTPUT_DIR
            old_offset_file = config.TASK2_OFFSET_FILE
            old_view_poses = (
                config.TASK2_CARD_VIEW_POSE,
                config.TASK2_BLOCK_VIEW_POSE,
                config.TASK2_TRAY_VIEW_POSE,
            )
            old_tray_ranges = config.TASK2_TRAY_HSV_RANGES
            config.TASK2_OUTPUT_DIR = str(root / "output")
            config.TASK2_OFFSET_FILE = str(root / "missing_offsets.json")
            config.TASK2_CARD_VIEW_POSE = None
            config.TASK2_BLOCK_VIEW_POSE = None
            config.TASK2_TRAY_VIEW_POSE = None
            config.TASK2_TRAY_HSV_RANGES = config.TASK2_BLOCK_HSV_RANGES
            try:
                result = task2_run(FakeVoice(), FakeVision(image), None, FakeLLM())
            finally:
                config.TASK2_OUTPUT_DIR = old_output
                config.TASK2_OFFSET_FILE = old_offset_file
                (config.TASK2_CARD_VIEW_POSE,
                 config.TASK2_BLOCK_VIEW_POSE,
                 config.TASK2_TRAY_VIEW_POSE) = old_view_poses
                config.TASK2_TRAY_HSV_RANGES = old_tray_ranges
            self.assertIsNotNone(result)
            self.assertEqual(6, len(result["plan"]))
            self.assertTrue(all(len(x["pick"]["pixel_center"]) == 2 for x in result["plan"]))
            logs = list((root / "output").glob("task2_*.json"))
            self.assertEqual(1, len(logs))
            self.assertEqual(6, len(json.loads(logs[0].read_text(encoding="utf-8"))["steps"]))


if __name__ == "__main__":
    unittest.main()
