import tempfile
import unittest
from pathlib import Path

from contracts.task2 import validate_motion_compensation
from tools.task2.workflow.task2_closed_loop_offset_calibrate import (
    _build_manual_candidate, _return_transfer_poses, _write_json,
)


class ManualOffsetCandidateTest(unittest.TestCase):
    def test_manual_candidate_zeros_compensation(self):
        offsets = {
            "calibration_world_scale_mm": 1.0, "calibration_xml_sha256": "abc",
            "block_origin_xy": [1.0, 2.0], "block_view_orientation_rad": [0.0] * 3,
            "block_xy_offset": [0.0, 0.0], "tray_origin_xy": [3.0, 4.0],
            "tray_view_orientation_rad": [0.0] * 3, "tray_xy_offset": [0.0, 0.0],
        }
        block = {"pixel": [10.0, 20.0], "predicted_xy_without_offset": [1.0, 2.0],
                 "actual_tcp_xy": [6.0, 8.0]}
        tray = {"pixel": [30.0, 40.0], "predicted_xy_without_offset": [3.0, 4.0],
                "actual_tcp_xy": [10.0, 12.0]}
        candidate = _build_manual_candidate(
            offsets, [5.0, 6.0], [7.0, 8.0], block, tray, "report.json")
        self.assertEqual(candidate["block_xy_offset"], [5.0, 6.0])
        self.assertEqual(candidate["tray_xy_offset"], [7.0, 8.0])
        compensation = validate_motion_compensation(candidate["motion_compensation"])
        for model in compensation["models"].values():
            self.assertEqual(model["constant_bias_mm"], [0.0, 0.0])
            self.assertEqual(model["spatial_residual_matrix"], [[0.0, 0.0], [0.0, 0.0]])
            self.assertEqual(model["rotation_center_bias_mm"], [0.0, 0.0])

    def test_checkpoint_write_is_atomic(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "checkpoint.json"
            _write_json(path, {"stage": "block"})
            self.assertTrue(path.exists())
            self.assertFalse(path.with_suffix(".json.tmp").exists())

    def test_cross_zone_return_uses_action_z_semantics(self):
        offsets = {
            "tray_origin_xy": [100.0, 200.0],
            "tray_view_orientation_rad": [1.0, 2.0, 3.0],
            "block_origin_xy": [300.0, 400.0],
            "block_view_orientation_rad": [4.0, 5.0, 6.0],
        }
        retrieve, returned = _return_transfer_poses(
            {"world_xy": [10.0, 20.0]}, {"world_xy": [30.0, 40.0]},
            offsets, [1.0, 2.0], [3.0, 4.0], 111.0, 222.0)
        self.assertEqual(retrieve[2], 111.0)
        self.assertEqual(returned[2], 222.0)


if __name__ == "__main__":
    unittest.main()
