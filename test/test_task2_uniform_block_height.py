import unittest
from unittest.mock import patch

import config
from modules import task2_planning
from runtime.task2_state import get_task2_block_view_pose


class UniformBlockHeightTests(unittest.TestCase):
    def test_one_scalar_drives_photo_height_and_planning_height(self):
        reference_pose = [1.0, 2.0, 400.0, 0.1, 0.2, 0.3]
        with patch.object(config, "TASK2_BLOCK_HEIGHT_MM", 31.5):
            effective = get_task2_block_view_pose(reference_pose)
            self.assertEqual(effective, [1.0, 2.0, 431.5, 0.1, 0.2, 0.3])
            self.assertEqual(task2_planning._height(), 31.5)

    def test_removed_height_aliases_do_not_form_second_source(self):
        self.assertFalse(hasattr(config, "TASK2_BLOCK_PHOTO_HEIGHT_MM"))
        self.assertFalse(hasattr(config, "TASK2_REFERENCE_BLOCK_HEIGHT_MM"))


if __name__ == "__main__":
    unittest.main()
