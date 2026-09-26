import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

from contracts.task2 import VisionTarget
from modules.task2_perception import ColorObjectDetector
from tools.task2.workflow.task2_tuner import (
    _build_tuning_payload, _commit_range, _sample_confirmed_hsv_prototypes,
    _swap_red_pink_prototypes, save_tuning,
)


class Task2TunerTest(unittest.TestCase):
    def test_payload_is_snapshot_and_uses_scene_specific_capture(self):
        ranges = {"蓝色": [[[90, 40, 10], [130, 255, 255]]]}
        payload = _build_tuning_payload(
            "tray", ranges, 5, 300, 230100, 23118, 0.0)
        ranges["蓝色"][0][0][0] = 1
        self.assertEqual(payload["tray_hsv_ranges"]["蓝色"][0][0][0], 90)
        self.assertEqual(payload["capture"], {
            "tray_exposure_time": 23118.0, "tray_gain": 0.0})

    def test_commit_supports_multiple_hsv_ranges(self):
        ranges = {"红色": [[[0, 80, 20], [8, 255, 255]],
                           [[175, 80, 20], [179, 255, 255]]]}
        _commit_range(ranges, "红色", 1, [170, 70, 30], [179, 240, 250])
        self.assertEqual(ranges["红色"][0][0], [0, 80, 20])
        self.assertEqual(ranges["红色"][1][0], [170, 70, 30])

    def test_rejects_inverted_hsv_range(self):
        ranges = {"红色": [[[0, 80, 20], [8, 255, 255]]]}
        with self.assertRaises(ValueError):
            _commit_range(ranges, "红色", 0, [9, 80, 20], [8, 255, 255])

    def test_save_preserves_other_scene_review_and_capture(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "task2_tuning.json"
            path.write_text(json.dumps({
                "capture": {"tray_gain": 3.0},
                "human_reviews": {"tray": {"operator_visual_confirmation": True}},
            }), encoding="utf-8")
            with patch("tools.task2.workflow.task2_tuner.config.TASK2_TUNING_FILE",
                       str(path)):
                save_tuning({
                    "capture": {"block_gain": 2.0},
                    "human_reviews": {"block": {"operator_visual_confirmation": True}},
                })
            saved = json.loads(path.read_text(encoding="utf-8"))
            self.assertEqual(set(saved["human_reviews"]), {"block", "tray"})
            self.assertEqual(saved["capture"], {"tray_gain": 3.0, "block_gain": 2.0})

    def test_confirmed_prototype_samples_target_interior(self):
        import cv2
        hsv = np.zeros((60, 60, 3), dtype=np.uint8)
        hsv[:, :] = [25, 170, 220]
        image = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        target = VisionTarget("方块", "黄色", (30.0, 30.0), 900.0, 0.0)
        sampled = _sample_confirmed_hsv_prototypes(image, [target])
        self.assertEqual(sampled["黄色"][0], [25.0, 170.0, 220.0])

    def test_red_pink_cost_uses_saturation_and_brightness(self):
        red_prototype = ((4, 180, 118),)
        pink_prototype = ((3, 158, 211),)
        red_pixel = np.asarray([4, 178, 120], dtype=float)
        pink_pixel = np.asarray([3, 160, 208], dtype=float)
        self.assertLess(
            ColorObjectDetector._prototype_cost(red_pixel, red_prototype),
            ColorObjectDetector._prototype_cost(red_pixel, pink_prototype))
        self.assertLess(
            ColorObjectDetector._prototype_cost(pink_pixel, pink_prototype),
            ColorObjectDetector._prototype_cost(pink_pixel, red_prototype))

    def test_manual_red_pink_swap_only_swaps_prototype_identity(self):
        original = {"红色": ((1, 2, 3),), "粉色": ((4, 5, 6),),
                    "蓝色": ((7, 8, 9),)}
        swapped = _swap_red_pink_prototypes(original)
        self.assertEqual(swapped["红色"], original["粉色"])
        self.assertEqual(swapped["粉色"], original["红色"])
        self.assertEqual(swapped["蓝色"], original["蓝色"])


if __name__ == "__main__":
    unittest.main()
