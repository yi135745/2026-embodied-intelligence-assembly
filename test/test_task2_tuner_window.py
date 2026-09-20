"""轨道条窗口被关闭时，调参工具正常退出。"""
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import task2_tuner


class TunerWindowTest(unittest.TestCase):
    def test_closed_window_is_not_read(self):
        with patch.object(task2_tuner.cv2, "getWindowProperty", return_value=-1):
            self.assertFalse(task2_tuner._window_open())
        with patch.object(task2_tuner.cv2, "getWindowProperty", side_effect=cv2.error("closed")):
            self.assertFalse(task2_tuner._window_open())
        with patch.object(task2_tuner.cv2, "getWindowProperty", return_value=1):
            self.assertTrue(task2_tuner._window_open())

    def test_static_image_close_before_first_slider_read(self):
        with tempfile.TemporaryDirectory() as folder:
            image_path = Path(folder) / "image.jpg"
            self.assertTrue(cv2.imwrite(str(image_path), np.zeros((16, 16, 3), dtype=np.uint8)))
            with patch.object(sys, "argv", ["task2_tuner.py", "--image", str(image_path)]), \
                    patch.object(task2_tuner, "load_task2_tuning"), \
                    patch.object(task2_tuner, "_window_open", return_value=False), \
                    patch.object(task2_tuner.cv2, "namedWindow"), \
                    patch.object(task2_tuner.cv2, "createTrackbar"), \
                    patch.object(task2_tuner.cv2, "setTrackbarPos"), \
                    patch.object(task2_tuner.cv2, "getTrackbarPos") as get_slider, \
                    patch.object(task2_tuner.cv2, "imshow"), \
                    patch.object(task2_tuner.cv2, "waitKey"), \
                    patch.object(task2_tuner.cv2, "destroyAllWindows"):
                task2_tuner.main()
            get_slider.assert_not_called()


if __name__ == "__main__":
    unittest.main()
