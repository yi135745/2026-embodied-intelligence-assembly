"""任务二相机/吸盘物理锚定的统一数据接口。"""

import json
import math
import os
from datetime import datetime
from pathlib import Path

import numpy as np

from ._closed_loop import fit_rotation_center_bias


SCHEMA_VERSION = 3


def build_physical_calibration_record(pixel_center, aligned_tcp_pose,
                                      block_view_pose, block_height_mm,
                                      motion_compensation=None, evidence=None):
    """建立与平面矩阵无关的物理观测；不保存由矩阵派生的区域偏移。"""
    pixel = [float(value) for value in pixel_center]
    tcp = [float(value) for value in aligned_tcp_pose]
    view = [float(value) for value in block_view_pose]
    height = float(block_height_mm)
    if len(pixel) != 2 or len(tcp) != 6 or len(view) != 6:
        raise ValueError("物理标定要求2维像素中心和两个6维TCP位姿。")
    if not all(math.isfinite(value) for value in pixel + tcp + view + [height]):
        raise ValueError("物理标定数据必须为有限数值。")
    if height <= 0:
        raise ValueError("物块高度必须大于0。")
    return {
        "schema_version": SCHEMA_VERSION,
        "updated_at": datetime.now().isoformat(timespec="seconds"),
        "physical_alignment": {
            "reference_kind": "block",
            "pixel_center": pixel,
            "aligned_tcp_pose_mm_rad": tcp,
            "reference_view_pose_mm_rad": view,
            "block_height_mm": height,
            "evidence": evidence or {},
        },
        "motion_compensation": motion_compensation or {
            "model_version": 1, "models": {}},
    }


def save_physical_calibration(path, payload):
    """原子保存人工粗校、物块高度和偏心数据。"""
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_suffix(target.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=2),
                         encoding="utf-8")
    os.replace(temporary, target)


def derive_shared_xy_calibration(payload, pixel_to_world, block_view_pose,
                                 tray_view_pose):
    """用当前矩阵组合原始物理锚点；矩阵更新无需重做人工对准。"""
    alignment = payload.get("physical_alignment")
    if not isinstance(alignment, dict):
        raise ValueError("物理标定缺少physical_alignment。")
    pixel = alignment.get("pixel_center")
    tcp = alignment.get("aligned_tcp_pose_mm_rad")
    saved_view = alignment.get("reference_view_pose_mm_rad")
    if not isinstance(pixel, list) or len(pixel) != 2:
        raise ValueError("物理标定像素中心无效。")
    if not isinstance(tcp, list) or len(tcp) != 6:
        raise ValueError("物理标定人工对准TCP位姿无效。")
    if not isinstance(saved_view, list) or len(saved_view) != 6:
        raise ValueError("物理标定参考拍照位无效。")
    current = [float(value) for value in block_view_pose]
    if max(abs(float(saved_view[i]) - current[i]) for i in (0, 1, 3, 4, 5)) > 0.5:
        raise ValueError("方块参考拍照位XY/姿态已改变，物理锚点不可复用。")
    world = pixel_to_world(float(pixel[0]), float(pixel[1]))
    if world is None:
        raise ValueError("当前公共矩阵无法转换物理锚点像素。")
    offset = (np.asarray(tcp[:2], dtype=float) -
              np.asarray(current[:2], dtype=float) -
              np.asarray(world, dtype=float))
    return {
        "block_origin_xy": current[:2],
        "block_xy_offset": offset.tolist(),
        "block_view_orientation_rad": current[3:],
        "tray_origin_xy": [float(value) for value in tray_view_pose[:2]],
        "tray_xy_offset": offset.tolist(),
        "tray_view_orientation_rad": [float(value) for value in tray_view_pose[3:]],
        "shared_camera_tool_offset": offset.tolist(),
    }


def derive_rotation_center_bias_from_pixels(trials, pixel_to_world):
    """通过当前公共矩阵把原始像素旋转观测派生为毫米偏心。"""
    converted = []
    for trial in trials or []:
        before = trial.get("before_pixel_center")
        after = trial.get("after_pixel_center")
        if (not isinstance(before, list) or len(before) != 2 or
                not isinstance(after, list) or len(after) != 2):
            raise ValueError("旋转偏心观测必须包含旋转前后二维像素中心。")
        before_world = np.asarray(pixel_to_world(*map(float, before)), dtype=float)
        after_world = np.asarray(pixel_to_world(*map(float, after)), dtype=float)
        converted.append({
            "command_rotation_deg": float(trial["command_rotation_deg"]),
            "center_residual_mm": (after_world - before_world).tolist(),
        })
    bias, rms = fit_rotation_center_bias(converted)
    return {
        "rotation_center_bias_mm": bias.tolist(),
        "fit_rms_mm": float(rms),
        "sample_count": len(converted),
        "source_unit": "pixel",
        "derived_unit": "mm",
    }
