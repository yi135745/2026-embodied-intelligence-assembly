import unittest

from tools.task2.workflow.task2_card_capture_tuner import parse_args


class Task2CardCaptureTunerTest(unittest.TestCase):
    def test_default_uses_camera_and_allows_automatic_move(self):
        args = parse_args([])
        self.assertIsNone(args.image)
        self.assertFalse(args.no_move)

    def test_no_move_is_explicit(self):
        self.assertTrue(parse_args(["--no-move"]).no_move)


if __name__ == "__main__":
    unittest.main()
