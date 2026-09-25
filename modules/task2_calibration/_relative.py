"""标定库内部：利用眼在手上的相机相对运动拟合像素到机器人XY的单应矩阵。

同一区域的物块保持不动；不同区域不需要人工测量物块绝对坐标。
拟合只确定矩阵的形状，平移零点由后续方块/托盘XY偏移标定确定。
"""

from collections import defaultdict
import math

import cv2
import numpy as np
from scipy.optimize import least_squares


def _group_samples(samples):
    groups = defaultdict(list)
    for sample in samples:
        pixel = np.asarray(sample["image_point"], dtype=np.float64)
        camera = np.asarray(sample["camera_tcp_pose_mm_rad"][:2], dtype=np.float64)
        if pixel.shape != (2,) or camera.shape != (2,) or not np.all(np.isfinite(pixel)) or not np.all(np.isfinite(camera)):
            raise ValueError("标定样本的像素中心或相机TCP XY无效。")
        groups[sample["region"]].append((pixel, camera))
    if len(groups) < 3 or any(len(group) < 5 for group in groups.values()):
        raise ValueError("相对标定至少需要3个区域，每区至少5张有效图。")
    return groups


def _anchored_world(parameters, pixels, origin):
    local = (pixels - origin) / 1000.0
    denominator = 1.0 + parameters[4] * local[:, 0] + parameters[5] * local[:, 1]
    if np.any(np.abs(denominator) < 0.1):
        raise ValueError("拟合的投影矩阵在采样范围内接近奇异。")
    x = (parameters[0] * local[:, 0] + parameters[1] * local[:, 1]) / denominator
    y = (parameters[2] * local[:, 0] + parameters[3] * local[:, 1]) / denominator
    return np.column_stack((x, y))


def _matrix(parameters, origin):
    x0, y0 = origin
    a, b, c, d, e, f = parameters
    matrix = np.asarray([
        [a / 1000, b / 1000, -(a * x0 + b * y0) / 1000],
        [c / 1000, d / 1000, -(c * x0 + d * y0) / 1000],
        [e / 1000, f / 1000, 1 - (e * x0 + f * y0) / 1000],
    ], dtype=np.float64)
    if abs(matrix[2, 2]) < 1e-9 or abs(np.linalg.det(matrix)) < 1e-15:
        raise ValueError("相对标定产生不可用矩阵。")
    return matrix / matrix[2, 2]


def fit_relative_homography(samples, *, cross_validate=True):
    """返回矩阵、各组内残差和供XML使用的相对世界点；不含人工绝对对准。"""
    groups = _group_samples(samples)
    if cross_validate and len(groups) < 4:
        raise ValueError("留出区域验证至少需要4个区域。")
    pixels = np.asarray([sample["image_point"] for sample in samples], dtype=np.float64)
    cameras = np.asarray([sample["camera_tcp_pose_mm_rad"][:2] for sample in samples], dtype=np.float64)
    origin = np.median(pixels, axis=0)
    indices = [np.asarray([i for i, sample in enumerate(samples) if sample["region"] == name])
               for name in groups]
    mean_group_size = float(np.mean([len(group) for group in indices]))
    group_weights = [math.sqrt(mean_group_size / len(group)) for group in indices]

    def residuals(parameters):
        try:
            projected = _anchored_world(parameters, pixels, origin) + cameras
        except ValueError:
            return np.full(pixels.size, 1e6)
        errors = np.zeros_like(projected)
        for group, weight in zip(indices, group_weights):
            errors[group] = (projected[group] - projected[group].mean(axis=0)) * weight
        return errors.ravel()

    centered_pixels = np.zeros_like(pixels)
    centered_world = np.zeros_like(cameras)
    for group, weight in zip(indices, group_weights):
        centered_pixels[group] = ((pixels[group] - pixels[group].mean(axis=0)) /
                                  1000.0 * weight)
        centered_world[group] = -(cameras[group] - cameras[group].mean(axis=0)) * weight
    affine, *_ = np.linalg.lstsq(centered_pixels, centered_world, rcond=None)
    if np.linalg.matrix_rank(centered_pixels) < 2:
        raise ValueError("相机运动方向不足，无法拟合平面变换。")
    initial = np.asarray([affine[0, 0], affine[1, 0], affine[0, 1], affine[1, 1], 0., 0.])
    result = least_squares(residuals, initial, loss="soft_l1", f_scale=0.7,
                           x_scale="jac", max_nfev=1000)
    if not result.success or not np.all(np.isfinite(result.x)):
        raise ValueError("相对运动矩阵拟合未收敛：" + result.message)
    matrix = _matrix(result.x, origin)
    predicted = cv2.perspectiveTransform(pixels.reshape(-1, 1, 2), matrix).reshape(-1, 2)
    region_positions = {}
    errors = np.zeros_like(predicted)
    world_points = np.zeros_like(predicted)
    for name, group in zip(groups, indices):
        # 每区固定物块的未知基座XY由九次自动观测求均值，不取人工示教坐标。
        position = (predicted[group] + cameras[group]).mean(axis=0)
        region_positions[name] = position.tolist()
        world_points[group] = position - cameras[group]
        errors[group] = predicted[group] - world_points[group]
    lengths = np.linalg.norm(errors, axis=1)
    region_rms = {name: float(np.sqrt(np.mean(lengths[group] ** 2)))
                  for name, group in zip(groups, indices)}
    report = {
        "mode": "relative_camera_motion_no_manual_region_alignment",
        "reference_pixel_zero": origin.tolist(),
        "region_estimated_block_xy": region_positions,
        "relative_world_rms_mm": float(np.sqrt(np.mean(lengths ** 2))),
        "relative_world_max_mm": float(np.max(lengths)),
        "region_relative_rms_mm": region_rms,
        "region_sample_counts": {name: len(group) for name, group in zip(groups, indices)},
        "region_total_weighting": "equalized_by_sqrt_mean_count_over_group_count",
        "requires_new_xy_offset_calibration": True,
        "training_error_is_not_independent_placement_accuracy": True,
    }
    if cross_validate:
        held_out = {}
        for name in groups:
            training = [sample for sample in samples if sample["region"] != name]
            candidate, _world, _errors, _details = fit_relative_homography(
                training, cross_validate=False)
            group = indices[list(groups).index(name)]
            mapped = cv2.perspectiveTransform(
                pixels[group].reshape(-1, 1, 2), candidate).reshape(-1, 2)
            predicted_base = mapped + cameras[group]
            held_error = predicted_base - predicted_base.mean(axis=0)
            held_out[name] = float(np.sqrt(np.mean(np.sum(held_error ** 2, axis=1))))
        report["leave_one_region_out_relative_rms_mm"] = held_out
    return matrix, world_points, errors, report
