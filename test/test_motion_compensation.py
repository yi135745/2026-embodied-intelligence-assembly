import unittest

from modules.task2_planning import (
    fit_constant_motion_compensation,
    predict_motion_residual,
)


class ConstantMotionCompensationTests(unittest.TestCase):
    def test_constant_model_uses_median_and_zeros_other_terms(self):
        trials = [
            {"target_robot_xy": [0.0, 0.0], "placement_residual_mm": [2.0, -1.0]},
            {"target_robot_xy": [100.0, 0.0], "placement_residual_mm": [2.2, -0.8]},
            {"target_robot_xy": [0.0, 100.0], "placement_residual_mm": [20.0, 15.0]},
        ]
        model = fit_constant_motion_compensation(trials)
        self.assertEqual(model["mode"], "constant_median")
        self.assertEqual(model["constant_bias_mm"], [2.2, -0.8])
        self.assertEqual(model["spatial_residual_matrix"], [[0.0, 0.0], [0.0, 0.0]])
        self.assertEqual(model["rotation_center_bias_mm"], [0.0, 0.0])
        self.assertEqual(predict_motion_residual(model, [999.0, -999.0], 37.0).tolist(),
                         [2.2, -0.8])

    def test_constant_model_requires_samples(self):
        with self.assertRaises(ValueError):
            fit_constant_motion_compensation([])


if __name__ == "__main__":
    unittest.main()
