import tempfile
import unittest
from pathlib import Path

from contracts.task2 import validate_motion_compensation
from modules.task2_calibration import (
    build_physical_calibration_record,
    derive_rotation_center_bias_from_pixels,
    derive_shared_xy_calibration,
)
from tools.task2.workflow.task2_closed_loop_offset_calibrate import (
    _build_manual_candidate, _write_json,
)
from runtime.task2_state import Task2RuntimeState


class ManualOffsetCandidateTest(unittest.TestCase):
    def test_legacy_runtime_state_does_not_enable_height_adjusted_view(self):
        state = Task2RuntimeState({}, {"model_version": 1, "models": {}}, True)
        self.assertFalse(state.uses_physical_alignment)

    def test_physical_anchor_is_rederived_by_current_matrix(self):
        payload = build_physical_calibration_record(
            [10.0, 20.0], [105.0, 207.0, 1.0, 0.0, 0.0, 0.0],
            [100.0, 200.0, 400.0, 0.0, 0.0, 0.0], 30.0)
        first = derive_shared_xy_calibration(
            payload, lambda x, y: (x, y),
            [100.0, 200.0, 400.0, 0.0, 0.0, 0.0],
            [300.0, 200.0, 400.0, 0.0, 0.0, 0.0])
        second = derive_shared_xy_calibration(
            payload, lambda x, y: (2 * x, 2 * y),
            [100.0, 200.0, 400.0, 0.0, 0.0, 0.0],
            [300.0, 200.0, 400.0, 0.0, 0.0, 0.0])
        self.assertEqual(first["block_xy_offset"], [-5.0, -13.0])
        self.assertEqual(first["block_xy_offset"], first["tray_xy_offset"])
        self.assertEqual(second["block_xy_offset"], [-15.0, -33.0])

    def test_rotation_bias_is_derived_from_pixel_observations(self):
        # b=(2,3), theta=90deg => (I-R)b=(5,1)
        result = derive_rotation_center_bias_from_pixels([{
            "before_pixel_center": [10.0, 10.0],
            "after_pixel_center": [15.0, 11.0],
            "command_rotation_deg": 90.0,
        }], lambda x, y: (x, y))
        self.assertAlmostEqual(result["rotation_center_bias_mm"][0], 2.0)
        self.assertAlmostEqual(result["rotation_center_bias_mm"][1], 3.0)
        self.assertEqual(result["source_unit"], "pixel")

    def test_manual_candidate_zeros_compensation(self):
        offsets = {
            "calibration_world_scale_mm": 1.0, "calibration_xml_sha256": "abc",
            "block_origin_xy": [1.0, 2.0], "block_view_orientation_rad": [0.0] * 3,
            "block_xy_offset": [0.0, 0.0], "tray_origin_xy": [3.0, 4.0],
            "tray_view_orientation_rad": [0.0] * 3, "tray_xy_offset": [0.0, 0.0],
        }
        block = {"pixel": [10.0, 20.0], "predicted_xy_without_offset": [1.0, 2.0],
                 "actual_tcp_xy": [6.0, 8.0]}
        tray = {"mode": "shared_camera_tool_offset"}
        candidate = _build_manual_candidate(
            offsets, [5.0, 6.0], [7.0, 8.0], block, tray, "report.json")
        self.assertEqual(candidate["block_xy_offset"], [5.0, 6.0])
        self.assertEqual(candidate["tray_xy_offset"], [5.0, 6.0])
        self.assertIn("physical_alignment", candidate)
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

if __name__ == "__main__":
    unittest.main()
