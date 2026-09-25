"""任务二双区人工锚定与可变采样闭环：先粗对准，再自动采样生成候选。"""

import argparse
import hashlib
import json
import math
import os
import shutil
import sys
import tempfile
import time
from datetime import datetime
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[3]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import cv2
import numpy as np

import config
from runtime.site_data import apply_aubo_pose_records
from modules.robot import Robot
from modules.task2_calibration import (
    calibration_sha256,
    proposed_block_offset,
    rebase_xy_offset,
    require_calibration_quality,
)
from modules.task2_planning import (
    compensate_place_pose,
    fit_constant_motion_compensation,
    fit_motion_compensation,
)
from contracts.task2 import normalize_square_angle, select_square_rotation
from modules.task2_perception import (ColorObjectDetector,
                                  detect_block_board,
                                  valid_block_contour,
                                  load_task2_tuning)
from modules.camera import Camera
from runtime.task2_state import (
    load_task2_runtime_state,
    require_shared_calibration_view_compatibility,
)


BLOCK_COLOR = "红色"
TRAY_REFERENCE_COLOR = "黄色"
# 两区人工对准用来重建绝对基准，允许相对矩阵零点与基座坐标相差较大；
# 自动旋转微调仍由 MAX_BLOCK_CORRECTION_MM 独立限幅。
# 首次建立绝对锚点时，矩阵平移项可与基座坐标相差较大；这里只拦截明显
# 对错物体/坐标系的情况，后续自动平移与旋转仍使用更严格的独立门禁。
MAX_MANUAL_SEED_MM = 250.0
MAX_BLOCK_CORRECTION_MM = 12.0
MAX_TRANSLATION_RESIDUAL_MM = 3.0
MAX_TRAY_VERIFICATION_RESIDUAL_MM = 3.0
MAX_SCENE_SHIFT_PX = 15.0
CALIBRATION_BOARD_GUARD = "fallback"


def _write_json(path, payload):
    """原子写入运行证据，确保中途异常仍保留已完成阶段。"""
    path = Path(path)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=2),
                         encoding="utf-8")
    os.replace(temporary, path)


def _zero_motion_model(reference_xy):
    return {
        "model_version": 1,
        "mode": "zero_manual_fallback",
        "reference_robot_xy": list(map(float, reference_xy)),
        "constant_bias_mm": [0.0, 0.0],
        "spatial_residual_matrix": [[0.0, 0.0], [0.0, 0.0]],
        "rotation_center_bias_mm": [0.0, 0.0],
        "sample_count": 0,
        "fit_rms_mm": 0.0,
    }


def _build_manual_candidate(offsets, block_offset, tray_offset,
                            block_record, tray_record, report_path):
    candidate = dict(offsets)
    candidate.update({
        "updated_at": datetime.now().isoformat(timespec="seconds"),
        "block_xy_offset": list(map(float, block_offset)),
        "tray_xy_offset": list(map(float, tray_offset)),
        "motion_compensation": {
            "model_version": 1,
            "models": {
                "block": _zero_motion_model(block_record["actual_tcp_xy"]),
                "tray": _zero_motion_model(tray_record["actual_tcp_xy"]),
            },
        },
        "block_reference_color": BLOCK_COLOR,
        "block_pixel_center": block_record["pixel"],
        "block_predicted_xy": block_record["predicted_xy_without_offset"],
        "block_actual_xy": block_record["actual_tcp_xy"],
        "tray_reference_color": TRAY_REFERENCE_COLOR,
        "tray_pixel_center": tray_record["pixel"],
        "tray_predicted_xy": tray_record["predicted_xy_without_offset"],
        "tray_actual_xy": tray_record["actual_tcp_xy"],
        "closed_loop_calibration": {
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "mode": "manual_fallback_zero_motion_compensation",
            "report": str(Path(report_path).resolve()),
            "manual_block_alignment": block_record,
            "manual_tray_alignment": tray_record,
            "motion_compensation_forced_zero": True,
        },
    })
    return candidate


def _sampling_plan():
    profile = str(config.TASK2_OFFSET_CALIBRATION_PROFILE)
    plans = config.TASK2_OFFSET_CALIBRATION_PLANS
    if profile not in plans:
        raise RuntimeError(
            "未知偏移标定采样方案%s；可选：%s。" %
            (profile, ", ".join(sorted(plans))))
    raw = plans[profile]
    required = {
        "translation_moves_mm", "rotation_moves_deg",
        "tray_calibration_trials", "tray_verification_trials",
    }
    if set(raw) != required:
        raise RuntimeError(
            "偏移标定采样方案字段必须恰好为：%s。" % ", ".join(sorted(required)))

    translations = [list(map(float, item)) for item in raw["translation_moves_mm"]]
    rotations = [float(item) for item in raw["rotation_moves_deg"]]
    tray_calibration = [
        {"target_color": str(color), "rotation_deg": float(angle)}
        for color, angle in raw["tray_calibration_trials"]
    ]
    tray_verification = [
        {"target_color": str(color), "rotation_deg": float(angle)}
        for color, angle in raw["tray_verification_trials"]
    ]
    if not translations or any(len(item) != 2 for item in translations):
        raise RuntimeError("平移采样至少需要1组二维位移。")
    if (not np.all(np.isfinite(np.asarray(translations, dtype=float))) or
            any(np.linalg.norm(item) < 1e-6 for item in translations)):
        raise RuntimeError("平移采样必须是有限且非零的二维位移。")
    if not np.all(np.isfinite(np.asarray(rotations, dtype=float))):
        raise RuntimeError("旋转采样角度必须是有限数值。")
    if not rotations or not any(abs(item) >= 2.0 for item in rotations):
        raise RuntimeError("旋转采样至少需要1个绝对值不小于2°的角度。")
    if not tray_calibration or not tray_verification:
        raise RuntimeError("托盘修正和修正后验证各至少需要1次跨区放置。")
    for trial in tray_calibration + tray_verification:
        if trial["target_color"] not in config.TASK2_TRAY_COLORS:
            raise RuntimeError("采样方案包含非法托盘颜色：" + trial["target_color"])
        if not math.isfinite(trial["rotation_deg"]):
            raise RuntimeError("托盘采样旋转角必须是有限数值。")
    if not any(abs(item["rotation_deg"]) <= 1e-9 for item in tray_calibration):
        raise RuntimeError("动作补偿拟合至少需要1组0°基准样本。")
    if not any(abs(item["rotation_deg"]) >= 2.0 for item in tray_calibration):
        raise RuntimeError("动作补偿拟合至少需要1组绝对值不小于2°的旋转样本。")
    return {
        "profile": profile,
        "translation_moves_mm": translations,
        "rotation_moves_deg": rotations,
        "tray_calibration_trials": tray_calibration,
        "tray_verification_trials": tray_verification,
    }


def _capture(vision, path, scene):
    prefix = scene.upper()
    return vision.capture(
        output_name=path,
        exposure_time=getattr(config, "TASK2_%s_EXPOSURE_TIME" % prefix),
        gain=getattr(config, "TASK2_%s_GAIN" % prefix),
    )


def _move_and_capture(robot, vision, view_pose, run_dir, name, scene):
    if not robot.move_to_safe(view_pose):
        raise RuntimeError("移动到%s拍照位失败。" % scene)
    time.sleep(max(0.0, float(config.TASK2_SETTLE_SECONDS)))
    return _capture(vision, run_dir / (name + "_raw.jpg"), scene)


def _detect_colored_square(image_path, color, transformer, expected_pixel=None):
    """只检测指定颜色的完整方块；expected_pixel用于排除同色托盘/背景。"""
    image = cv2.imread(str(image_path))
    if image is None:
        raise RuntimeError("无法读取闭环图片：" + str(image_path))
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lower, upper in config.TASK2_BLOCK_HSV_RANGES[color]:
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))
    kernel = np.ones((max(1, int(config.TASK2_MORPH_KERNEL)),) * 2, np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if not max(1500.0, float(config.TASK2_MIN_CONTOUR_AREA)) <= area <= float(config.TASK2_MAX_CONTOUR_AREA):
            continue
        rect = cv2.minAreaRect(contour)
        (cx, cy), (width, height), _angle = rect
        if min(width, height) <= 0:
            continue
        if max(width, height) / min(width, height) > 1.55:
            continue
        if area / (width * height) < 0.60:
            continue
        distance = (0.0 if expected_pixel is None else
                    float(math.dist((cx, cy), expected_pixel)))
        if expected_pixel is not None and distance > 420.0:
            continue
        candidates.append((distance, -area, contour, rect))
    if not candidates:
        raise RuntimeError("未在预期位置找到完整%s方块。" % color)
    _distance, negative_area, contour, rect = min(candidates)
    (cx, cy), _, _ = rect
    world = transformer.pixel_to_world(cx, cy)
    angle = transformer.rectangle_angle_to_robot(rect)
    annotated = image.copy()
    cv2.drawContours(annotated, [cv2.boxPoints(rect).astype(np.int32)], 0, (0, 255, 255), 3)
    cv2.circle(annotated, (round(cx), round(cy)), 8, (0, 0, 255), -1)
    cv2.putText(annotated, "%s center=(%.1f,%.1f)" % (color, cx, cy), (30, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
    return {
        "pixel": [float(cx), float(cy)],
        "world_xy": [float(world[0]), float(world[1])],
        "robot_angle_deg": float(angle),
        "area": float(-negative_area),
        "annotated": annotated,
        "mask": mask,
    }


def _save_detection(run_dir, name, detection):
    annotated_path = run_dir / (name + "_detected.jpg")
    mask_path = run_dir / (name + "_mask.png")
    cv2.imwrite(str(annotated_path), detection["annotated"])
    cv2.imwrite(str(mask_path), detection["mask"])
    return str(annotated_path.resolve()), str(mask_path.resolve())


def _require_single_block_scene(image_path, expected_pixel=None,
                                board_guard="fallback"):
    """闭环标定禁止多物块；使用九色并集只计算完整方块轮廓。"""
    image = cv2.imread(str(image_path))
    if image is None:
        raise RuntimeError("无法读取物块场景：" + str(image_path))
    board_audit = {"mode": board_guard, "status": "passed", "reason": None}
    bounds = None
    if board_guard != "off":
        try:
            bounds = detect_block_board(image)
        except ValueError as exc:
            if board_guard == "strict":
                raise
            board_audit.update(status="fallback", reason=str(exc))
            print("警告：白板边界审计失败，标定流程降级为全画面HSV单方块审计：%s" % exc)
    else:
        board_audit.update(status="disabled", reason="operator_requested")
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for ranges in config.TASK2_BLOCK_HSV_RANGES.values():
        for lower, upper in ranges:
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))
    kernel = np.ones((max(1, int(config.TASK2_MORPH_KERNEL)),) * 2, np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if bounds is not None:
        contours = [contour for contour in contours if valid_block_contour(contour, bounds)]
    else:
        filtered = []
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if not max(1500.0, float(config.TASK2_MIN_CONTOUR_AREA)) <= area <= float(config.TASK2_MAX_CONTOUR_AREA):
                continue
            (_center, (width, height), _angle) = cv2.minAreaRect(contour)
            if min(width, height) <= 0 or max(width, height) / min(width, height) > 1.55:
                continue
            if area / (width * height) < 0.60:
                continue
            filtered.append(contour)
        contours = filtered
    centers = [cv2.minAreaRect(contour)[0] for contour in contours]
    if len(centers) != 1:
        raise RuntimeError(
            "闭环标定要求物块区只有1个完整方块，当前检测到%d个；"
            "请只保留一个红色方块。" % len(centers))
    center = tuple(map(float, centers[0]))
    if expected_pixel is not None:
        distance = float(math.dist(center, expected_pixel))
        if distance > MAX_SCENE_SHIFT_PX:
            raise RuntimeError(
                "人工锚定或自动采样前的静态复拍期间，方块/场景发生变化："
                "中心偏移%.1f px > %.1f px。" %
                (distance, MAX_SCENE_SHIFT_PX))
    board_audit["detected_block_count"] = len(centers)
    return center, board_audit


def _pose_for_detection(detection, scene, offsets, xy_offset, z):
    origin = offsets[scene + "_origin_xy"]
    orientation = offsets[scene + "_view_orientation_rad"]
    return [origin[0] + detection["world_xy"][0] + xy_offset[0],
            origin[1] + detection["world_xy"][1] + xy_offset[1],
            float(z), *map(float, orientation)]


def _tray_target(detector, image_path, color, run_dir, name):
    targets, annotated = detector.detect(image_path, "托盘", include_robot_pose=False)
    path = run_dir / (name + "_trays_detected.jpg")
    cv2.imwrite(str(path), annotated)
    target = next((item for item in targets if item.color == color), None)
    if target is None:
        raise RuntimeError("未识别到%s托盘。" % color)
    transformer = detector.get_transformer("托盘")
    world = transformer.pixel_to_world(*target.pixel_center)
    return {
        "color": color,
        "pixel": list(target.pixel_center),
        "world_xy": [float(world[0]), float(world[1])],
        "robot_angle_deg": float(target.robot_angle_deg),
        "annotated_image": str(path.resolve()),
    }


def _pose_for_tray_target(target, offsets, tray_offset, z):
    origin = offsets["tray_origin_xy"]
    orientation = offsets["tray_view_orientation_rad"]
    return [origin[0] + target["world_xy"][0] + tray_offset[0],
            origin[1] + target["world_xy"][1] + tray_offset[1],
            float(z), *map(float, orientation)]


def _return_transfer_poses(placed, block_return_target, offsets,
                           tray_offset, block_offset, pick_z, place_z):
    """反向取回仍按动作语义选Z：托盘吸取用pick_z，物块区释放用place_z。"""
    retrieve_pose = _pose_for_detection(
        placed, "tray", offsets, tray_offset, pick_z)
    return_pose = _pose_for_detection(
        block_return_target, "block", offsets, block_offset, place_z)
    return retrieve_pose, return_pose


def _transfer(robot, pick_pose, place_pose, rotation_deg, label):
    rotation_deg = select_square_rotation(rotation_deg, config.TASK2_ROTATION_DIRECTION)
    commanded_place = list(place_pose)
    commanded_place[3:] = pick_pose[3:]
    commanded_place[5] = ((pick_pose[5] + math.radians(rotation_deg) + math.pi) %
                          (2 * math.pi) - math.pi)
    print("%s：XY目标=[%.3f, %.3f] mm，旋转=%+.1f°" %
          (label, commanded_place[0], commanded_place[1], rotation_deg))
    if not robot.pick_and_place(pick_pose, commanded_place):
        raise RuntimeError(label + "抓放失败。")
    return rotation_deg


def _atomic_activate(candidate, active, history_dir):
    active, candidate, history_dir = Path(active), Path(candidate), Path(history_dir)
    history_dir.mkdir(parents=True, exist_ok=True)
    backup = history_dir / (active.stem + "_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f") + active.suffix)
    if active.exists():
        shutil.copy2(active, backup)
    else:
        backup = None
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="wb", dir=active.parent,
                                         prefix=".task2_offsets_", suffix=".tmp",
                                         delete=False) as output:
            temporary = Path(output.name)
            output.write(candidate.read_bytes())
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary, active)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()
    return backup


def _offer_activation(candidate_path, offset_path, original_bytes, report,
                      report_path):
    if input("确认候选内容后，输入 yes 启用；其他输入仅保留候选：").strip().lower() != "yes":
        print("未覆盖正式偏移。")
        return False
    if original_bytes is None:
        if offset_path.exists():
            raise RuntimeError("运行期间新出现正式偏移文件，拒绝覆盖。")
    elif not offset_path.exists() or offset_path.read_bytes() != original_bytes:
        raise RuntimeError("运行期间正式偏移文件发生变化，拒绝覆盖。")
    backup = _atomic_activate(candidate_path, offset_path,
                              Path(config.DATA_DIR) / "offset_history")
    report["formal_offset_modified"] = True
    report["activated_offset_file"] = str(offset_path.resolve())
    report["previous_offset_backup"] = str(backup.resolve()) if backup else None
    _write_json(report_path, report)
    print("正式偏移已更新。" +
          (("旧版备份：" + str(backup.resolve())) if backup else "此前没有旧版。"))
    return True


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _print_offset_activation_summary(active_path, candidate_path, report):
    old_block = report["previous_block_xy_offset"]
    old_tray = report["previous_tray_xy_offset"]
    new_block = np.asarray(report["candidate_block_xy_offset"], dtype=float)
    new_tray = np.asarray(report["candidate_tray_xy_offset"], dtype=float)
    rotation_step = report["block_fit"]["applied_rotation_offset_step_mm"]
    print("\n========== XY偏移启用决策 ==========")
    print("当前正式：%s" % Path(active_path).resolve())
    print("当前指纹：%s" % (_sha256(active_path) if Path(active_path).exists() else "无"))
    print("候选文件：%s" % Path(candidate_path).resolve())
    print("候选指纹：%s" % _sha256(candidate_path))
    if old_block is None:
        print("方块正式旧值：无；候选=%s" % new_block.tolist())
    else:
        old_block = np.asarray(old_block, dtype=float)
        print("方块正式旧值：%s -> %s（变化%.3f mm）" %
              (old_block.tolist(), new_block.tolist(), np.linalg.norm(new_block - old_block)))
    if old_tray is None:
        print("托盘正式旧值：无；候选=%s" % new_tray.tolist())
    else:
        old_tray = np.asarray(old_tray, dtype=float)
        print("托盘正式旧值：%s -> %s（变化%.3f mm）" %
              (old_tray.tolist(), new_tray.tolist(), np.linalg.norm(new_tray - old_tray)))
    print("人工粗对准：方块=%s mm；托盘=%s mm" %
          (report["manual_block_alignment"]["coarse_xy_offset"],
           report["manual_tray_alignment"]["coarse_xy_offset"]))
    print("纯平移验收：%d组，最大残差=%.3f / %.3f mm -> PASS" %
          (len(report["translation_trials"]),
           report["translation_summary"]["max_residual_norm_mm"],
           MAX_TRANSLATION_RESIDUAL_MM))
    block_model = report.get("candidate_motion_compensation", {}).get("models", {}).get("block", {})
    if block_model.get("mode") == "zero_manual_fallback":
        print("旋转偏心诊断：%s mm（constant模式仅记录，不进入动作补偿）" % rotation_step)
    else:
        print("旋转偏心诊断：%s mm（只进入动作补偿，不改坐标偏移）" % rotation_step)
    print("托盘动作模型样本残差：%s" %
          [item["placement_residual_mm"] for item in report["tray_trials"]])
    print("托盘动作补偿：%s，%d组，拟合RMS=%.3f mm" %
          (report["tray_fit"]["mode"], report["tray_fit"]["sample_count"],
           report["tray_fit"]["fit_rms_mm"]))
    print("托盘动作补偿后验证残差：%s" %
          [item["placement_residual_mm"]
           for item in report["tray_verification_trials"]])
    print("采样方案：%s" % report["sampling_plan"]["profile"])
    print("质量结论：PASS（所有自动门禁已通过）")
    print("启用影响：替换正式方块/托盘XY偏移，矩阵XML不修改。")
    print("备份策略：启用前将当前正式偏移写入data/offset_history。")


def _fresh_offsets_from_current_calibration(matrix_hash, _transformer=None,
                                            _matrix_quality=None):
    """建立双区人工锚定前的空偏移；不从相对九点报告猜测绝对零点。"""
    return {
        "updated_at": datetime.now().isoformat(timespec="seconds"),
        "calibration_world_scale_mm": float(config.TASK2_CALIBRATION_WORLD_SCALE_MM),
        "calibration_xml_sha256": matrix_hash,
        "block_origin_xy": list(map(float, config.TASK2_BLOCK_VIEW_POSE[:2])),
        "block_view_orientation_rad": list(map(float, config.TASK2_BLOCK_VIEW_POSE[3:])),
        "block_xy_offset": [0.0, 0.0],
        "tray_origin_xy": list(map(float, config.TASK2_TRAY_VIEW_POSE[:2])),
        "tray_view_orientation_rad": list(map(float, config.TASK2_TRAY_VIEW_POSE[3:])),
        "tray_xy_offset": [0.0, 0.0],
    }, {
        "mode": "dual_manual_absolute_anchor",
        "block_seed_xy_offset": [0.0, 0.0],
        "tray_seed_xy_offset": [0.0, 0.0],
        "uses_previous_offset_file": False,
        "uses_relative_report_as_absolute_anchor": False,
    }


def _capture_block_detection(robot, vision, transformer, run_dir, name,
                             expected_pixel=None, enforce_scene_stability=False,
                             board_guard=None):
    image = _move_and_capture(
        robot, vision, config.TASK2_BLOCK_VIEW_POSE, run_dir, name, "block")
    stability_reference = expected_pixel if enforce_scene_stability else None
    board_guard = board_guard or CALIBRATION_BOARD_GUARD
    center, board_audit = _require_single_block_scene(
        image, expected_pixel=stability_reference, board_guard=board_guard)
    detection = _detect_colored_square(
        image, BLOCK_COLOR, transformer,
        expected_pixel=(expected_pixel if expected_pixel is not None else center))
    _save_detection(run_dir, name, detection)
    detection["board_guard_audit"] = board_audit
    _write_json(run_dir / (name + "_board_guard_audit.json"), board_audit)
    return detection


def _capture_tray_target(robot, vision, detector, run_dir, name, color):
    image = _move_and_capture(
        robot, vision, config.TASK2_TRAY_VIEW_POSE, run_dir, name, "tray")
    return _tray_target(detector, image, color, run_dir, name)


def _manual_block_anchor(robot, vision, transformer, offsets, run_dir, pick_z,
                         board_guard="fallback"):
    initial = _capture_block_detection(
        robot, vision, transformer, run_dir, "01_block_anchor_before",
        board_guard=board_guard)
    predicted = _pose_for_detection(initial, "block", offsets, [0.0, 0.0], pick_z)
    answer = input(
        "请用示教器把吸盘XY粗对准%s方块中心（Z不采用）；回到窗口按回车读取，输入q取消：" %
        BLOCK_COLOR).strip().lower()
    if answer == "q":
        raise RuntimeError("用户取消物块区人工粗对准。")
    actual_pose = robot.get_current_pose()
    verified = _capture_block_detection(
        robot, vision, transformer, run_dir, "02_block_anchor_after",
        expected_pixel=initial["pixel"], enforce_scene_stability=True,
        board_guard=board_guard)
    scene_shift = float(math.dist(initial["pixel"], verified["pixel"]))
    offset = rebase_xy_offset(
        offsets["block_origin_xy"], verified["world_xy"], actual_pose[:2])
    predicted = _pose_for_detection(verified, "block", offsets, [0.0, 0.0], pick_z)
    distance = float(np.linalg.norm(offset))
    if distance > MAX_MANUAL_SEED_MM:
        raise RuntimeError(
            "物块区人工对准与矩阵零偏预测相差%.3f mm，超过%.1f mm；"
            "请检查是否对准红块中心及TCP坐标。" %
            (distance, MAX_MANUAL_SEED_MM))
    return offset, verified, {
        "reference_color": BLOCK_COLOR,
        "pixel": list(map(float, verified["pixel"])),
        "world_xy": list(map(float, verified["world_xy"])),
        "predicted_xy_without_offset": list(map(float, predicted[:2])),
        "actual_tcp_xy": list(map(float, actual_pose[:2])),
        "coarse_xy_offset": list(map(float, offset)),
        "coarse_offset_norm_mm": distance,
        "scene_shift_px": scene_shift,
        "board_guard_audits": [initial["board_guard_audit"],
                               verified["board_guard_audit"]],
    }


def _manual_tray_anchor(robot, vision, detector, offsets, run_dir, place_z):
    initial = _capture_tray_target(
        robot, vision, detector, run_dir, "03_tray_anchor_before", TRAY_REFERENCE_COLOR)
    predicted = _pose_for_tray_target(initial, offsets, [0.0, 0.0], place_z)
    answer = input(
        "请用示教器把吸盘XY粗对准%s托盘几何中心（Z不采用）；"
        "回到窗口按回车读取，输入q取消：" % TRAY_REFERENCE_COLOR
    ).strip().lower()
    if answer == "q":
        raise RuntimeError("用户取消托盘区人工粗对准。")
    actual_pose = robot.get_current_pose()
    verified = _capture_tray_target(
        robot, vision, detector, run_dir, "04_tray_anchor_after", TRAY_REFERENCE_COLOR)
    scene_shift = float(math.dist(initial["pixel"], verified["pixel"]))
    if scene_shift > MAX_SCENE_SHIFT_PX:
        raise RuntimeError(
            "托盘区人工对准期间参考托盘或场景发生变化：中心偏移%.1f px > %.1f px。" %
            (scene_shift, MAX_SCENE_SHIFT_PX))
    offset = rebase_xy_offset(
        offsets["tray_origin_xy"], verified["world_xy"], actual_pose[:2])
    predicted = _pose_for_tray_target(verified, offsets, [0.0, 0.0], place_z)
    distance = float(np.linalg.norm(offset))
    if distance > MAX_MANUAL_SEED_MM:
        raise RuntimeError(
            "托盘区人工对准与矩阵零偏预测相差%.3f mm，超过%.1f mm；"
            "请检查是否对准%s托盘中心及TCP坐标。" %
            (distance, MAX_MANUAL_SEED_MM, TRAY_REFERENCE_COLOR))
    return offset, verified, {
        "reference_color": TRAY_REFERENCE_COLOR,
        "pixel": list(map(float, verified["pixel"])),
        "world_xy": list(map(float, verified["world_xy"])),
        "predicted_xy_without_offset": list(map(float, predicted[:2])),
        "actual_tcp_xy": list(map(float, actual_pose[:2])),
        "coarse_xy_offset": list(map(float, offset)),
        "coarse_offset_norm_mm": distance,
        "scene_shift_px": scene_shift,
    }


def _cross_zone_trial(robot, vision, transformer, offsets, run_dir, name,
                      current_block, block_return_target, block_offset,
                      tray_offset, tray_target, pick_z, place_z, rotation_deg,
                      motion_model=None):
    """物块区到托盘区取样后立即取回，保留独立的跨区绝对偏移观测。"""
    pick_pose = _pose_for_detection(
        current_block, "block", offsets, block_offset, pick_z)
    place_pose = _pose_for_tray_target(tray_target, offsets, tray_offset, place_z)
    command_rotation_deg = select_square_rotation(
        rotation_deg, config.TASK2_ROTATION_DIRECTION)
    command_place_pose, predicted_residual = compensate_place_pose(
        place_pose, command_rotation_deg, motion_model)
    _transfer(robot, pick_pose, command_place_pose, command_rotation_deg,
              "%s：物块区到%s托盘" % (name, tray_target["color"]))

    placed_image = _move_and_capture(
        robot, vision, config.TASK2_TRAY_VIEW_POSE, run_dir,
        name + "_tray_after", "tray")
    placed = _detect_colored_square(
        placed_image, BLOCK_COLOR, transformer,
        expected_pixel=tray_target["pixel"])
    _save_detection(run_dir, name + "_tray_after", placed)
    residual = (np.asarray(placed["world_xy"]) -
                np.asarray(tray_target["world_xy"]))

    retrieve_pose, return_pose = _return_transfer_poses(
        placed, block_return_target, offsets, tray_offset, block_offset,
        pick_z, place_z)
    _transfer(robot, retrieve_pose, return_pose, -command_rotation_deg,
              "%s：取回物块区" % name)
    returned = _capture_block_detection(
        robot, vision, transformer, run_dir, name + "_block_return",
        expected_pixel=block_return_target["pixel"])
    return_residual = (np.asarray(returned["world_xy"]) -
                       np.asarray(block_return_target["world_xy"]))
    return_pixel_shift = (np.asarray(returned["pixel"]) -
                          np.asarray(block_return_target["pixel"]))
    return placed, returned, {
        "target_color": tray_target["color"],
        "requested_rotation_deg": float(rotation_deg),
        "command_rotation_deg": float(command_rotation_deg),
        "target_robot_xy": list(map(float, place_pose[:2])),
        "command_place_xy": list(map(float, command_place_pose[:2])),
        "predicted_motion_residual_mm": predicted_residual,
        "target_world_xy": list(map(float, tray_target["world_xy"])),
        "observed_block_world_xy": list(map(float, placed["world_xy"])),
        "placement_residual_mm": residual.tolist(),
        "block_return_residual_mm": return_residual.tolist(),
        "block_return_residual_norm_mm": float(np.linalg.norm(return_residual)),
        "block_return_pixel_shift": return_pixel_shift.tolist(),
        "block_return_pixel_shift_norm": float(np.linalg.norm(return_pixel_shift)),
        "height_parallax_not_separated": True,
    }


def main():
    global CALIBRATION_BOARD_GUARD
    parser = argparse.ArgumentParser(
        description="任务二双区人工粗对准与可变采样半闭环偏移标定"
    )
    parser.add_argument(
        "--manual-only", action="store_true",
        help="只建立人工方块/托盘固定XY偏移，并将综合动作补偿置零")
    parser.add_argument(
        "--board-guard", choices=("strict", "fallback", "off"),
        default="fallback",
        help="标定专用白板审计；fallback在边界不可靠时保留HSV单目标审计")
    parser.add_argument(
        "--compensation-model", choices=("adaptive", "constant"),
        default="adaptive",
        help="adaptive拟合空间/旋转模型；constant仅取固定偏心并将其余补偿归零")
    args = parser.parse_args()
    CALIBRATION_BOARD_GUARD = args.board_guard
    sampling_plan = _sampling_plan()
    apply_aubo_pose_records()
    compatibility = require_shared_calibration_view_compatibility()
    print("共用矩阵位姿检查通过：Z差=%.3f mm，RZ差=%.6f rad。" %
          (compatibility["z_diff_mm"], compatibility["rz_diff_rad"]))
    load_task2_tuning()
    offset_path = Path(config.TASK2_OFFSET_FILE)
    original_bytes = offset_path.read_bytes() if offset_path.exists() else None
    previous_offsets = (json.loads(original_bytes.decode("utf-8"))
                        if original_bytes is not None else None)
    detector = ColorObjectDetector()
    block_transformer = detector.get_transformer("方块")
    tray_transformer = detector.get_transformer("托盘")
    if block_transformer is not tray_transformer:
        raise RuntimeError("内部错误：方块和托盘没有共用同一个坐标变换器。")
    matrix_hash = calibration_sha256(config.TASK2_CALIBRATION_FILE)
    matrix_quality = require_calibration_quality(
        config.TASK2_CALIBRATION_FILE, config.TASK2_OUTPUT_DIR, "方块/托盘共用")
    offsets, fresh_offset_seed = _fresh_offsets_from_current_calibration(
        matrix_hash, tray_transformer, matrix_quality)
    print("本轮偏移从零建立：物块区、托盘区均由本次人工粗对准确定；"
          "旧偏移和相对九点报告不参与绝对偏移计算。")
    print("自动采样方案：%s（平移%d次、旋转%d次、托盘修正%d次、验证%d次）。" %
          (sampling_plan["profile"],
           len(sampling_plan["translation_moves_mm"]),
           len(sampling_plan["rotation_moves_deg"]),
           len(sampling_plan["tray_calibration_trials"]),
           len(sampling_plan["tray_verification_trials"])))

    block_height = float(config.TASK2_BLOCK_HEIGHT_MM[BLOCK_COLOR])
    z_delta = block_height - float(config.TASK2_REFERENCE_BLOCK_HEIGHT_MM)
    # 两个Z按动作语义使用，而不是按区域使用：无论物块在哪个区域，
    # 吸取始终用pick_z，释放始终用place_z。
    pick_z = float(config.TASK2_BLOCK_PICK_Z) + z_delta
    place_z = float(config.TASK2_TRAY_PLACE_Z) + z_delta
    run_dir = (Path(config.TASK2_OUTPUT_DIR) / "closed_loop_offset_calibration" /
               datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
    run_dir.mkdir(parents=True, exist_ok=False)

    robot, vision = Robot(), Camera()
    if not robot.available:
        raise RuntimeError("AUBO未连接，无法执行半闭环标定。")
    completed = False
    report = {
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "mode": "dual_manual_anchor_guarded_variable_sampling_closed_loop",
        "requested_mode": "manual_only" if args.manual_only else "advanced",
        "requested_compensation_model": args.compensation_model,
        "calibration_board_guard": args.board_guard,
        "block_color": BLOCK_COLOR,
        "tray_reference_color": TRAY_REFERENCE_COLOR,
        "sampling_plan": sampling_plan,
        "parameters": {
            "max_manual_seed_mm": MAX_MANUAL_SEED_MM,
            "max_block_correction_mm": MAX_BLOCK_CORRECTION_MM,
            "max_translation_residual_mm": MAX_TRANSLATION_RESIDUAL_MM,
            "max_tray_verification_residual_mm": MAX_TRAY_VERIFICATION_RESIDUAL_MM,
            "max_scene_shift_px": MAX_SCENE_SHIFT_PX,
        },
        "matrix_quality": matrix_quality,
        "matrix_sha256": matrix_hash,
        "shared_view_compatibility": compatibility,
        "previous_offset_file_present": previous_offsets is not None,
        "previous_offsets_used_in_calculation": False,
        "previous_block_xy_offset": (previous_offsets.get("block_xy_offset")
                                     if previous_offsets else None),
        "previous_tray_xy_offset": (previous_offsets.get("tray_xy_offset")
                                    if previous_offsets else None),
        "fresh_offset_seed": fresh_offset_seed,
        "initial_block_xy_offset": list(offsets["block_xy_offset"]),
        "initial_tray_xy_offset": list(offsets["tray_xy_offset"]),
        "formal_offset_modified": False,
        "translation_trials": [],
        "block_trials": [],
        "tray_trials": [],
        "tray_verification_trials": [],
    }
    try:
        print("请清空物块区，只保留一个%s方块并放在画面中部；"
              "采样方案涉及的托盘必须为空。" % BLOCK_COLOR)
        print("先依次完成人工物块、托盘粗对准；两次都完成前不会执行抓放。")
        coarse_block_offset, block_anchor, block_anchor_record = _manual_block_anchor(
            robot, vision, block_transformer, offsets, run_dir, pick_z,
            board_guard=args.board_guard)
        report["manual_block_alignment"] = block_anchor_record
        report["completed_stage"] = "manual_block_anchor"
        block_checkpoint_path = run_dir / "01_manual_block_checkpoint.json"
        _write_json(block_checkpoint_path, report)
        print("物块区人工粗对准完成：XY偏移=%s mm。" % coarse_block_offset)
        print("物块锚点已落盘：" + str(block_checkpoint_path.resolve()))

        coarse_tray_offset, tray_anchor, tray_anchor_record = _manual_tray_anchor(
            robot, vision, detector, offsets, run_dir, place_z)
        report["manual_tray_alignment"] = tray_anchor_record
        print("托盘区人工粗对准完成：XY偏移=%s mm。" % coarse_tray_offset)
        report["coarse_block_xy_offset"] = list(coarse_block_offset)
        report["coarse_tray_xy_offset"] = list(coarse_tray_offset)
        report["completed_stage"] = "manual_dual_anchor"
        report_path = run_dir / "closed_loop_report.json"
        manual_candidate_path = run_dir / "task2_offsets_v2_manual_candidate.json"
        manual_candidate = _build_manual_candidate(
            offsets, coarse_block_offset, coarse_tray_offset,
            block_anchor_record, tray_anchor_record, report_path)
        _write_json(manual_candidate_path, manual_candidate)
        load_task2_runtime_state(manual_candidate_path)
        report["manual_candidate_offset_file"] = str(manual_candidate_path.resolve())
        report["manual_candidate_motion_compensation_zero"] = True
        _write_json(report_path, report)
        print("人工双锚点候选已落盘（综合动作补偿为0）：" +
              str(manual_candidate_path.resolve()))
        manual_decision = "manual" if args.manual_only else input(
            "人工锚点已完成：回车继续%s自动采样；输入manual以零补偿候选结束；"
            "输入q保留候选并退出：" % args.compensation_model
        ).strip().lower()
        if manual_decision not in ("", "manual", "q"):
            raise RuntimeError("人工阶段选择只能是回车、manual或q。")
        if manual_decision in ("manual", "q"):
            report["manual_stage_decision"] = manual_decision
            report["completed_stage"] = "manual_fallback_selected"
            _write_json(report_path, report)
            print("人工保底阶段完成：不会执行自动抓放采样。")
            print("方块固定XY偏移：%s；托盘固定XY偏移：%s" %
                  (coarse_block_offset, coarse_tray_offset))
            if manual_decision == "manual":
                _offer_activation(manual_candidate_path, offset_path, original_bytes,
                                  report, report_path)
            else:
                print("已保留人工候选，未进入启用确认，正式偏移未修改。")
            completed = True
            return
        report["manual_stage_decision"] = "continue_" + args.compensation_model
        _write_json(report_path, report)
        print("两个绝对锚点均已建立，开始自动采样。")

        current_block = _capture_block_detection(
            robot, vision, block_transformer, run_dir, "05_block_auto_start",
            expected_pixel=block_anchor["pixel"], enforce_scene_stability=True)
        for index, move in enumerate(sampling_plan["translation_moves_mm"], start=1):
            pick_pose = _pose_for_detection(
                current_block, "block", offsets, coarse_block_offset, pick_z)
            place_pose = list(pick_pose)
            place_pose[0] += move[0]
            place_pose[1] += move[1]
            _transfer(robot, pick_pose, place_pose, 0.0,
                      "物块区平移采样%d" % index)
            after = _capture_block_detection(
                robot, vision, block_transformer, run_dir,
                "06_translation_%02d" % index)
            observed = (np.asarray(after["world_xy"]) -
                        np.asarray(current_block["world_xy"]))
            residual = observed - np.asarray(move, dtype=float)
            residual_norm = float(np.linalg.norm(residual))
            trial = {
                "command_translation_mm": list(move),
                "observed_translation_mm": observed.tolist(),
                "residual_mm": residual.tolist(),
                "residual_norm_mm": residual_norm,
                "limit_mm": MAX_TRANSLATION_RESIDUAL_MM,
                "passed": residual_norm <= MAX_TRANSLATION_RESIDUAL_MM,
            }
            report["translation_trials"].append(trial)
            print("平移采样%d：指令=%s，观测=%s，残差=%.3f mm" %
                  (index, move, observed.tolist(), residual_norm))
            if residual_norm > MAX_TRANSLATION_RESIDUAL_MM:
                raise RuntimeError(
                    "纯平移验矩阵失败：第%d组残差%.3f mm超过%.3f mm；"
                    "禁止将该误差当作旋转偏心。" %
                    (index, residual_norm, MAX_TRANSLATION_RESIDUAL_MM))
            current_block = after
        translation_norms = [
            item["residual_norm_mm"] for item in report["translation_trials"]]
        report["translation_summary"] = {
            "sample_count": len(translation_norms),
            "rms_residual_norm_mm": float(
                np.sqrt(np.mean(np.square(translation_norms)))),
            "max_residual_norm_mm": float(max(translation_norms)),
        }

        for index, rotation_deg in enumerate(
                sampling_plan["rotation_moves_deg"], start=1):
            before = current_block
            pick_pose = _pose_for_detection(
                before, "block", offsets, coarse_block_offset, pick_z)
            command_rotation_deg = _transfer(
                robot, pick_pose, list(pick_pose), rotation_deg,
                "物块区旋转采样%d" % index)
            after = _capture_block_detection(
                robot, vision, block_transformer, run_dir,
                "07_rotation_%02d" % index)
            center_residual = (np.asarray(after["world_xy"]) -
                               np.asarray(before["world_xy"]))
            observed_rotation = normalize_square_angle(
                after["robot_angle_deg"] - before["robot_angle_deg"])
            report["block_trials"].append({
                "command_translation_mm": [0.0, 0.0],
                "observed_translation_mm": center_residual.tolist(),
                "center_residual_mm": center_residual.tolist(),
                "requested_rotation_deg": rotation_deg,
                "command_rotation_deg": command_rotation_deg,
                "observed_rotation_deg": observed_rotation,
                "angle_error_deg": normalize_square_angle(
                    observed_rotation - command_rotation_deg),
            })
            current_block = after
        _diagnostic_block_offset, block_fit = proposed_block_offset(
            [0.0, 0.0], coarse_block_offset, report["block_trials"],
            MAX_BLOCK_CORRECTION_MM)
        candidate_block_offset = list(coarse_block_offset)
        block_fit["coordinate_offset_applied"] = False
        block_fit["diagnostic_offset_if_legacy_applied"] = _diagnostic_block_offset
        report["block_fit"] = block_fit
        block_motion_model = {
            "model_version": 1,
            "mode": "rotation_only",
            "reference_robot_xy": list(map(float, current_block["world_xy"])),
            "constant_bias_mm": [0.0, 0.0],
            "spatial_residual_matrix": [[0.0, 0.0], [0.0, 0.0]],
            "rotation_center_bias_mm": list(map(
                float, block_fit["fitted_reference_pose_center_bias_mm"])),
            "sample_count": len(report["block_trials"]),
            "fit_rms_mm": float(block_fit["rotation_center_fit_rms_mm"]),
        }
        if args.compensation_model == "constant":
            block_motion_model = _zero_motion_model(current_block["world_xy"])

        block_return_target = current_block
        needed_colors = []
        for trial in (sampling_plan["tray_calibration_trials"] +
                      sampling_plan["tray_verification_trials"]):
            if trial["target_color"] not in needed_colors:
                needed_colors.append(trial["target_color"])
        tray_reference_image = _move_and_capture(
            robot, vision, config.TASK2_TRAY_VIEW_POSE, run_dir,
            "08_trays_before", "tray")
        tray_targets = {
            color: _tray_target(detector, tray_reference_image, color, run_dir,
                                "08_tray_%02d" % index)
            for index, color in enumerate(needed_colors, start=1)
        }
        current_block = _capture_block_detection(
            robot, vision, block_transformer, run_dir, "09_block_before_tray_trials",
            expected_pixel=block_return_target["pixel"], enforce_scene_stability=True)

        for index, trial_plan in enumerate(
                sampling_plan["tray_calibration_trials"], start=1):
            _placed, current_block, trial = _cross_zone_trial(
                robot, vision, tray_transformer, offsets, run_dir,
                "10_tray_calibration_%02d" % index,
                current_block, block_return_target, candidate_block_offset,
                coarse_tray_offset, tray_targets[trial_plan["target_color"]],
                pick_z, place_z, trial_plan["rotation_deg"])
            report["tray_trials"].append(trial)

        candidate_tray_offset = list(coarse_tray_offset)
        tray_fit = (fit_constant_motion_compensation(report["tray_trials"])
                    if args.compensation_model == "constant"
                    else fit_motion_compensation(report["tray_trials"]))
        report["tray_fit"] = tray_fit

        for index, trial_plan in enumerate(
                sampling_plan["tray_verification_trials"], start=1):
            _placed, current_block, trial = _cross_zone_trial(
                robot, vision, tray_transformer, offsets, run_dir,
                "11_tray_verify_%02d" % index,
                current_block, block_return_target, candidate_block_offset,
                candidate_tray_offset, tray_targets[trial_plan["target_color"]],
                pick_z, place_z, trial_plan["rotation_deg"], tray_fit)
            residual_norm = float(np.linalg.norm(trial["placement_residual_mm"]))
            trial["residual_norm_mm"] = residual_norm
            trial["limit_mm"] = MAX_TRAY_VERIFICATION_RESIDUAL_MM
            trial["passed"] = residual_norm <= MAX_TRAY_VERIFICATION_RESIDUAL_MM
            report["tray_verification_trials"].append(trial)
            if not trial["passed"]:
                raise RuntimeError(
                    "托盘修正后验证失败：第%d组残差%.3f mm超过%.3f mm。" %
                    (index, residual_norm, MAX_TRAY_VERIFICATION_RESIDUAL_MM))

        final_return_residual = (
            np.asarray(current_block["world_xy"]) -
            np.asarray(block_return_target["world_xy"]))

        candidate = dict(offsets)
        candidate["updated_at"] = datetime.now().isoformat(timespec="seconds")
        candidate["block_xy_offset"] = candidate_block_offset
        candidate["tray_xy_offset"] = candidate_tray_offset
        candidate["motion_compensation"] = {
            "model_version": 1,
            "models": {"tray": tray_fit, "block": block_motion_model}}
        report["candidate_motion_compensation"] = candidate["motion_compensation"]
        candidate["block_reference_color"] = BLOCK_COLOR
        candidate["block_pixel_center"] = block_anchor_record["pixel"]
        candidate["block_predicted_xy"] = block_anchor_record["predicted_xy_without_offset"]
        candidate["block_actual_xy"] = block_anchor_record["actual_tcp_xy"]
        candidate["tray_reference_color"] = TRAY_REFERENCE_COLOR
        candidate["tray_pixel_center"] = tray_anchor_record["pixel"]
        candidate["tray_predicted_xy"] = tray_anchor_record["predicted_xy_without_offset"]
        candidate["tray_actual_xy"] = tray_anchor_record["actual_tcp_xy"]
        report["candidate_block_xy_offset"] = candidate_block_offset
        report["candidate_tray_xy_offset"] = candidate_tray_offset
        report["block_return_validation_residual_mm"] = final_return_residual.tolist()
        report["warning"] = (
            "XY偏移仅由两次人工锚定建立；自动抓放残差进入动作补偿模型，"
            "不再污染坐标偏移。托盘闭环仍包含高度视差、释放滑动和检测误差；"
            "验证样本不参与拟合。"
        )
        candidate["closed_loop_calibration"] = {
            "created_at": report["created_at"],
            "report": str((run_dir / "closed_loop_report.json").resolve()),
            "sampling_profile": sampling_plan["profile"],
            "compensation_model": args.compensation_model,
            "manual_block_alignment": report["manual_block_alignment"],
            "manual_tray_alignment": report["manual_tray_alignment"],
            "block_fit": block_fit,
            "translation_summary": report["translation_summary"],
            "tray_fit": tray_fit,
            "motion_compensation": candidate["motion_compensation"],
            "tray_trials": report["tray_trials"],
            "tray_verification_trials": report["tray_verification_trials"],
            "block_return_validation_residual_mm": final_return_residual.tolist(),
        }
        candidate_path = run_dir / "task2_offsets_v2_candidate.json"
        report_path = run_dir / "closed_loop_report.json"
        _write_json(candidate_path, candidate)
        report["candidate_offset_file"] = str(candidate_path.resolve())
        _write_json(report_path, report)
        load_task2_runtime_state(candidate_path)

        print("\n半闭环候选已生成：" + str(candidate_path.resolve()))
        print("物块偏移：人工粗对准%s -> 候选%s" %
              (coarse_block_offset, candidate_block_offset))
        print("托盘偏移：人工粗对准%s -> 候选%s" %
              (coarse_tray_offset, candidate_tray_offset))
        print("最后一次取回物块区残差：%s mm" % final_return_residual.tolist())
        print("报告：" + str(report_path.resolve()))
        _print_offset_activation_summary(offset_path, candidate_path, report)
        _offer_activation(candidate_path, offset_path, original_bytes,
                          report, report_path)
        completed = True
    except Exception as exc:
        report["failure"] = {"type": type(exc).__name__, "message": str(exc)}
        failure_path = run_dir / "closed_loop_failure_report.json"
        _write_json(failure_path, report)
        print("半闭环已中止，失败报告：" + str(failure_path.resolve()))
        raise
    finally:
        cv2.destroyAllWindows()
        if completed and robot.available:
            try:
                robot.move_to_safe(config.TASK2_BLOCK_VIEW_POSE)
            except Exception as exc:
                print("警告：返回方块拍照位异常：" + str(exc))
        robot.disconnect()


if __name__ == "__main__":
    main()
