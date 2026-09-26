import unittest

from modules.task2_perception._board import _scale_bounds


class BlockBoardRegionScaleTest(unittest.TestCase):
    def test_expands_from_center(self):
        self.assertEqual(
            _scale_bounds((100, 200, 500, 600), (800, 1000, 3), 1.05),
            (90, 190, 510, 610),
        )

    def test_clips_to_image(self):
        self.assertEqual(
            _scale_bounds((0, 0, 100, 100), (120, 120, 3), 1.30),
            (0, 0, 115, 115),
        )

    def test_rejects_unsafe_scale(self):
        with self.assertRaisesRegex(ValueError, "比例"):
            _scale_bounds((10, 10, 20, 20), (100, 100, 3), 1.31)


if __name__ == "__main__":
    unittest.main()
