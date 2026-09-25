"""任务二双区人工锚定与可变采样闭环的纯计算部分。"""

import math

import numpy as np

from contracts.task2 import normalize_square_angle


def rebase_xy_offset(origin_xy, reference_world_xy, actual_tcp_xy):
    """用已保存的人工对准锚点，在新像素矩阵上重算XY偏移。"""
    origin = np.asarray(origin_xy, dtype=float)
    reference = np.asarray(reference_world_xy, dtype=float)
    actual = np.asarray(actual_tcp_xy, dtype=float)
    if any(value.shape != (2,) for value in (origin, reference, actual)):
        raise ValueError("偏移重建的原点、矩阵坐标和TCP坐标必须各含2个数。")
    return (actual - origin - reference).tolist()


def fit_rotation_center_bias(trials):
    """由旋转抓放后的中心残差拟合参考姿态下的二维抓取偏心。"""
    matrices, residuals = [], []
    for trial in trials:
        angle = math.radians(float(trial["command_rotation_deg"]))
        if abs(angle) < math.radians(2.0):
            continue
        rotation = np.asarray([[math.cos(angle), -math.sin(angle)],
                               [math.sin(angle), math.cos(angle)]], dtype=np.float64)
        matrices.append(np.eye(2) - rotation)
        residuals.append(np.asarray(trial["center_residual_mm"], dtype=np.float64))
    if not matrices:
        raise ValueError("至少需要一次非零角度抓放才能拟合旋转中心偏差。")
    design = np.vstack(matrices)
    observed = np.concatenate(residuals)
    bias, *_ = np.linalg.lstsq(design, observed, rcond=None)
    fitted = design @ bias
    rms = float(np.sqrt(np.mean((fitted - observed) ** 2)))
    return bias, rms


def summarize_rotation_response(trials):
    commands = np.asarray([float(item["command_rotation_deg"]) for item in trials], dtype=float)
    actual = np.asarray([float(item["observed_rotation_deg"]) for item in trials], dtype=float)
    valid = np.abs(commands) >= 2.0
    if not np.any(valid):
        return {"command_to_observed_scale": None, "mean_error_deg": None,
                "max_abs_error_deg": None}
    commands, actual = commands[valid], actual[valid]
    scale = float(np.dot(commands, actual) / np.dot(commands, commands))
    errors = np.asarray([normalize_square_angle(a - c) for a, c in zip(actual, commands)])
    return {
        "command_to_observed_scale": scale,
        "mean_error_deg": float(np.mean(errors)),
        "max_abs_error_deg": float(np.max(np.abs(errors))),
    }


def proposed_block_offset(current_offset, manual_seed_delta, rotation_trials,
                          max_correction_mm=12.0, rotation_gain=0.7):
    current = np.asarray(current_offset, dtype=float)
    seed = np.asarray(manual_seed_delta, dtype=float)
    bias, fit_rms = fit_rotation_center_bias(rotation_trials)
    # 人工对准是重建绝对基准，不应与自动闭环微调共用同一限幅。
    # 这里只限制由单次旋转观测推导的自动修正。
    rotation_step = -float(rotation_gain) * bias
    if np.linalg.norm(rotation_step) > float(max_correction_mm):
        raise ValueError("旋转观测的自动修正量%.3f mm超过%.3f mm保护阈值。" %
                         (np.linalg.norm(rotation_step), max_correction_mm))
    correction = seed + rotation_step
    return (current + correction).tolist(), {
        "manual_seed_delta_mm": seed.tolist(),
        "fitted_reference_pose_center_bias_mm": bias.tolist(),
        "rotation_center_fit_rms_mm": fit_rms,
        "rotation_bias_update_gain": float(rotation_gain),
        "applied_rotation_offset_step_mm": rotation_step.tolist(),
        "applied_block_offset_correction_mm": correction.tolist(),
        "rotation_response": summarize_rotation_response(rotation_trials),
    }


def apply_tray_residual(offset, residual_mm, gain=0.7, max_step_mm=8.0):
    """单次结果导向更新；落点偏向+X时，下次命令向-X补偿。"""
    current = np.asarray(offset, dtype=float)
    residual = np.asarray(residual_mm, dtype=float)
    step = -float(gain) * residual
    length = float(np.linalg.norm(step))
    if length > float(max_step_mm):
        raise ValueError("托盘自动闭环修正量%.3f mm超过%.3f mm保护阈值。" %
                         (length, max_step_mm))
    return (current + step).tolist(), step.tolist()


def proposed_tray_offset(current_offset, trials, gain=0.7, max_step_mm=8.0):
    """用同一冻结偏移下的多次跨区落点残差，一次性生成托盘候选偏移。"""
    residuals = np.asarray(
        [trial["placement_residual_mm"] for trial in trials], dtype=float)
    if residuals.ndim != 2 or residuals.shape[0] < 1 or residuals.shape[1] != 2:
        raise ValueError("托盘偏移修正至少需要1组二维落点残差。")
    if not np.all(np.isfinite(residuals)):
        raise ValueError("托盘落点残差包含非有限数值。")

    # 各试验必须使用同一冻结偏移；按坐标中位数聚合，避免执行顺序改变结果。
    center = np.median(residuals, axis=0)
    deviations = residuals - center
    lengths = np.linalg.norm(deviations, axis=1)
    candidate, step = apply_tray_residual(
        current_offset, center, gain=gain, max_step_mm=max_step_mm)
    return candidate, {
        "sample_count": int(residuals.shape[0]),
        "residual_median_mm": center.tolist(),
        "residual_rms_about_median_mm": float(
            np.sqrt(np.mean(np.sum(deviations ** 2, axis=1)))),
        "residual_max_deviation_mm": float(np.max(lengths)),
        "update_gain": float(gain),
        "applied_offset_step_mm": step,
    }
