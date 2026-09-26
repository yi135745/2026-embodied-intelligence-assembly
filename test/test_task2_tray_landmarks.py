import unittest
from unittest.mock import patch

import numpy as np

from modules.task2_perception import detect_tray_landmarks


class TrayLandmarkBootstrapTests(unittest.TestCase):
    def test_color_centers_fill_missing_geometry_in_first_frame(self):
        image = np.zeros((1000, 1200, 3), dtype=np.uint8)
        centers = (
            (200.0, 300.0), (600.0, 300.0), (1000.0, 300.0),
            (200.0, 700.0), (600.0, 700.0), (1000.0, 700.0),
        )
        geometry_centers = (centers[1], centers[2], centers[4], centers[5])
        candidates = [
            {
                "center": np.asarray(center, dtype=np.float64),
                "side": 120.0,
                "box": np.asarray([
                    [center[0] - 60, center[1] - 60],
                    [center[0] + 60, center[1] - 60],
                    [center[0] + 60, center[1] + 60],
                    [center[0] - 60, center[1] + 60],
                ], dtype=np.float32),
                "strong": True,
            }
            for center in geometry_centers
        ]
        edges = np.zeros(image.shape[:2], dtype=np.uint8)

        with patch(
            "modules.task2_perception._tray._square_candidates",
            return_value=(candidates, edges),
        ):
            landmarks, _annotated, returned_edges, _score = detect_tray_landmarks(
                image, centers)

        self.assertEqual([item.slot for item in landmarks], [
            "top_left", "top_center", "top_right",
            "bottom_left", "bottom_center", "bottom_right",
        ])
        self.assertEqual(
            [item.source for item in landmarks].count("geometry"), 4)
        self.assertEqual(
            [item.source for item in landmarks].count("color_bootstrap"), 2)
        self.assertIs(returned_edges, edges)

    def test_missing_geometry_still_fails_without_color_centers(self):
        image = np.zeros((1000, 1200, 3), dtype=np.uint8)
        edges = np.zeros(image.shape[:2], dtype=np.uint8)
        with patch(
            "modules.task2_perception._tray._square_candidates",
            return_value=([], edges),
        ):
            with self.assertRaisesRegex(RuntimeError, "颜色视觉补充0组"):
                detect_tray_landmarks(image)


if __name__ == "__main__":
    unittest.main()
