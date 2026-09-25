"""任务二正式矩阵绑定、XY 偏移和执行补偿的唯一运行时加载入口。"""

import hashlib
import json
from dataclasses import dataclass
from pathlib import Path

import config
from contracts.task2 import validate_motion_compensation


@dataclass(frozen=True)
class Task2RuntimeState:
    xy_calibration: dict | None
    motion_compensation: dict
    loaded: bool


def _file_sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def require_shared_calibration_view_compatibility(block_pose=None, tray_pose=None):
    """公共矩阵要求两个拍照位保持相同相机 Z 和 RZ。"""
    block_pose = config.TASK2_BLOCK_VIEW_POSE if block_pose is None else block_pose
    tray_pose = config.TASK2_TRAY_VIEW_POSE if tray_pose is None else tray_pose
    if block_pose is None or tray_pose is None:
        raise RuntimeError("共用标定矩阵需要方块区和托盘区拍照位。")
    z_diff = abs(float(block_pose[2]) - float(tray_pose[2]))
    rz_diff = abs(float(block_pose[5]) - float(tray_pose[5]))
    max_z = float(config.TASK2_SHARED_CALIBRATION_MAX_Z_DIFF_MM)
    max_rz = float(config.TASK2_SHARED_CALIBRATION_MAX_RZ_DIFF_RAD)
    if z_diff > max_z:
        raise RuntimeError(
            "方块/托盘拍照位Z差%.3f mm超过共用矩阵阈值%.3f mm。" %
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
    active_hash = _file_sha256(config.TASK2_CALIBRATION_FILE)
    binding_path = Path(config.TASK2_CALIBRATION_BINDING_FILE)
    if binding_path.exists():
        binding = json.loads(binding_path.read_text(encoding="utf-8"))
        if binding.get("active_xml_sha256") != active_hash:
            raise RuntimeError("正式标定XML与版本指纹不一致；请核对标定文件。")
        if data.get("calibration_xml_sha256") != active_hash:
            raise RuntimeError(
                "XY偏移属于旧矩阵；启用新标定后必须重新运行"
                "tools/task2/workflow/task2_closed_loop_offset_calibrate.py。")
    elif (data.get("calibration_xml_sha256") is not None and
          data["calibration_xml_sha256"] != active_hash):
        raise RuntimeError("XY偏移与当前标定XML不匹配；请重新标定偏移。")

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
