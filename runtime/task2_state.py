"""任务二正式矩阵绑定、XY 偏移和执行补偿的唯一运行时加载入口。"""

import hashlib
import json
from dataclasses import dataclass
from pathlib import Path

import config
from contracts.task2 import validate_motion_compensation
from modules.task2_calibration import (
    derive_rotation_center_bias_from_pixels,
    derive_shared_xy_calibration,
)
from modules.task2_perception import CoordinateTransformer


@dataclass(frozen=True)
class Task2RuntimeState:
    xy_calibration: dict | None
    motion_compensation: dict
    loaded: bool
    uses_physical_alignment: bool = False


def _file_sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def get_task2_block_view_pose(block_pose=None, block_height_mm=None):
    """由参考平面拍照位派生方块顶面拍照位，不建立第二个位姿真源。"""
    source = config.TASK2_BLOCK_VIEW_POSE if block_pose is None else block_pose
    if source is None:
        raise RuntimeError("尚未加载方块参考平面拍照位。")
    height = (config.TASK2_BLOCK_HEIGHT_MM if block_height_mm is None
              else block_height_mm)
    pose = [float(value) for value in source]
    pose[2] += float(height)
    return pose


def require_shared_calibration_view_compatibility(block_pose=None, tray_pose=None,
                                                  block_height_mm=None):
    """公共矩阵要求相机到观测平面的有效高度及RZ相同。"""
    block_pose = config.TASK2_BLOCK_VIEW_POSE if block_pose is None else block_pose
    tray_pose = config.TASK2_TRAY_VIEW_POSE if tray_pose is None else tray_pose
    if block_pose is None or tray_pose is None:
        raise RuntimeError("共用标定矩阵需要方块区和托盘区拍照位。")
    height = float(config.TASK2_BLOCK_HEIGHT_MM if block_height_mm is None
                   else block_height_mm)
    effective_block_z = get_task2_block_view_pose(block_pose, height)[2] - height
    z_diff = abs(effective_block_z - float(tray_pose[2]))
    rz_diff = abs(float(block_pose[5]) - float(tray_pose[5]))
    max_z = float(config.TASK2_SHARED_CALIBRATION_MAX_Z_DIFF_MM)
    max_rz = float(config.TASK2_SHARED_CALIBRATION_MAX_RZ_DIFF_RAD)
    if z_diff > max_z:
        raise RuntimeError(
            "方块顶面/托盘平面的有效拍照高度差%.3f mm超过共用矩阵阈值%.3f mm。" %
            (z_diff, max_z))
    if rz_diff > max_rz:
        raise RuntimeError(
            "方块/托盘拍照位RZ差%.6f rad超过共用矩阵阈值%.6f rad。" %
            (rz_diff, max_rz))
    return {"z_diff_mm": z_diff, "rz_diff_rad": rz_diff,
            "max_z_diff_mm": max_z, "max_rz_diff_rad": max_rz}


def load_task2_runtime_state(path=None):
    """加载并验证唯一 XY 状态；不修改感知或规划功能库的全局状态。"""
    offset_path = Path(path or config.TASK2_OFFSET_FILE)
    empty = Task2RuntimeState(
        xy_calibration=None,
        motion_compensation={"model_version": 1, "models": {}},
        loaded=False,
    )
    if not offset_path.exists():
        return empty
    data = json.loads(offset_path.read_text(encoding="utf-8"))
    if data.get("physical_alignment") is None:
        raise RuntimeError(
            "正式偏离数据仍是旧双区格式；请直接运行"
            "tools/task2/workflow/task2_closed_loop_offset_calibrate.py，"
            "重新建立单一人工物理锚点。托盘公共矩阵不会被修改。")
    active_hash = _file_sha256(config.TASK2_CALIBRATION_FILE)
    binding_path = Path(config.TASK2_CALIBRATION_BINDING_FILE)
    if binding_path.exists():
        binding = json.loads(binding_path.read_text(encoding="utf-8"))
        if binding.get("active_xml_sha256") != active_hash:
            raise RuntimeError("正式标定XML与版本指纹不一致；请核对标定文件。")
        if data.get("physical_alignment") is None and data.get("calibration_xml_sha256") != active_hash:
            raise RuntimeError(
                "XY偏移属于旧矩阵；启用新标定后必须重新运行"
                "tools/task2/workflow/task2_closed_loop_offset_calibrate.py。")
    elif (data.get("physical_alignment") is None and data.get("calibration_xml_sha256") is not None and
          data["calibration_xml_sha256"] != active_hash):
        raise RuntimeError("XY偏移与当前标定XML不匹配；请重新标定偏移。")

    if data.get("physical_alignment") is not None:
        transformer = CoordinateTransformer(config.TASK2_CALIBRATION_FILE)
        try:
            calibration = derive_shared_xy_calibration(
                data, transformer.pixel_to_world, config.TASK2_BLOCK_VIEW_POSE,
                config.TASK2_TRAY_VIEW_POSE)
        except ValueError as exc:
            raise RuntimeError("任务二物理标定与当前矩阵无法组合：%s" % exc) from exc
        compensation = validate_motion_compensation(data.get("motion_compensation"))
        trials = data["physical_alignment"].get("rotation_trials")
        if trials:
            derived = derive_rotation_center_bias_from_pixels(
                trials, transformer.pixel_to_world)
            models = {name: dict(model) for name, model in
                      compensation.get("models", {}).items()}
            block_model = dict(models.get("block", {
                "model_version": 1,
                "mode": "rotation_only",
                "reference_robot_xy": calibration["block_origin_xy"],
                "constant_bias_mm": [0.0, 0.0],
                "spatial_residual_matrix": [[0.0, 0.0], [0.0, 0.0]],
                "sample_count": 0,
                "fit_rms_mm": 0.0,
            }))
            block_model["rotation_center_bias_mm"] = derived["rotation_center_bias_mm"]
            block_model["sample_count"] = derived["sample_count"]
            block_model["fit_rms_mm"] = derived["fit_rms_mm"]
            models["block"] = block_model
            compensation = validate_motion_compensation(
                {"model_version": 1, "models": models})
        require_shared_calibration_view_compatibility()
        print("已组合任务二公共矩阵与物理标定：" + str(offset_path))
        return Task2RuntimeState(calibration, compensation, True, True)

    saved_scale = data.get("calibration_world_scale_mm")
    current_scale = float(config.TASK2_CALIBRATION_WORLD_SCALE_MM)
    if saved_scale is None:
        raise RuntimeError(
            "任务二XY标定JSON未记录九点矩阵单位倍率，可能由旧配置生成；"
            "请重新运行tools/task2/workflow/task2_closed_loop_offset_calibrate.py。")
    if abs(float(saved_scale) - current_scale) > 1e-9:
        raise RuntimeError(
            "任务二XY标定JSON的九点矩阵倍率为%s，当前配置为%s；请重新标定。" %
            (saved_scale, current_scale))

    required = ("block_origin_xy", "block_xy_offset", "block_view_orientation_rad",
                "tray_origin_xy", "tray_xy_offset", "tray_view_orientation_rad")
    missing = [key for key in required if key not in data]
    if missing:
        raise RuntimeError("任务二XY标定JSON缺少字段：%s" % missing)
    calibration = {}
    for key in required:
        value = data[key]
        expected_length = 3 if key.endswith("orientation_rad") else 2
        if not isinstance(value, list) or len(value) != expected_length:
            raise RuntimeError("任务二XY标定字段%s必须是%d个数。" %
                               (key, expected_length))
        calibration[key] = [float(item) for item in value]

    for prefix, view_pose in (("block", config.TASK2_BLOCK_VIEW_POSE),
                              ("tray", config.TASK2_TRAY_VIEW_POSE)):
        if view_pose is None:
            raise RuntimeError("尚未从aubo_poses.json加载%s拍照位。" % prefix)
        saved_xy = calibration[prefix + "_origin_xy"]
        saved_orientation = calibration[prefix + "_view_orientation_rad"]
        xy_error = max(abs(saved_xy[i] - float(view_pose[i])) for i in range(2))
        angle_error = max(abs(saved_orientation[i] - float(view_pose[i + 3]))
                          for i in range(3))
        if xy_error > 0.5 or angle_error > 0.005:
            raise RuntimeError(
                "%s拍照位已改变，但XY标定JSON仍属于旧位姿；请重新运行"
                "tools/task2/workflow/task2_closed_loop_offset_calibrate.py。" % prefix)

    require_shared_calibration_view_compatibility()
    try:
        compensation = validate_motion_compensation(data.get("motion_compensation"))
    except (KeyError, TypeError, ValueError) as exc:
        raise RuntimeError("任务二执行补偿模型无效：%s" % exc) from exc
    print("已加载任务二XY标定文件（Z仍取config）：" + str(offset_path))
    return Task2RuntimeState(calibration, compensation, True)
