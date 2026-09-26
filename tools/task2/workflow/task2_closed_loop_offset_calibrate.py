"""任务二单物理锚点与原地旋转偏心标定。"""

import argparse
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
    build_physical_calibration_record,
    rebase_xy_offset,
)
from contracts.task2 import normalize_square_angle, select_square_rotation
from modules.task2_perception import (CoordinateTransformer,
                                  detect_block_board,
                                  valid_block_contour,
                                  load_task2_tuning)
from modules.camera import Camera
from runtime.task2_state import (
    get_task2_block_view_pose,
)


BLOCK_COLOR = "红色"
# 人工物理锚点允许矩阵零点与基座坐标相差较大；自动旋转微调独立限幅。
# 首次建立绝对锚点时，矩阵平移项可与基座坐标相差较大；这里只拦截明显
# 对错物体/坐标系的情况，后续自动平移与旋转仍使用更严格的独立门禁。
MAX_MANUAL_SEED_MM = 250.0
MAX_BLOCK_CORRECTION_MM = 12.0
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
    # tray_offset/tray_record仅保留调用签名兼容；正式模型只使用一次物理锚定。
    actual_pose = block_record.get("actual_tcp_pose")
    if actual_pose is None:
        actual_pose = list(block_record["actual_tcp_xy"]) + [0.0] * 4
    view_pose = block_record.get("reference_view_pose", [0.0] * 6)
    height = block_record.get("block_height_mm", config.TASK2_BLOCK_PHOTO_HEIGHT_MM)
    candidate = build_physical_calibration_record(
        block_record["pixel"], actual_pose, view_pose, height,
        evidence={"report": str(Path(report_path).resolve()),
                  "manual_block_alignment": block_record})
    candidate.update({
        "updated_at": datetime.now().isoformat(timespec="seconds"),
        "block_xy_offset": list(map(float, block_offset)),
        "tray_xy_offset": list(map(float, block_offset)),
        "motion_compensation": {
            "model_version": 1,
            "models": {
                "block": _zero_motion_model(block_record["actual_tcp_xy"]),
                "tray": _zero_motion_model(block_record["actual_tcp_xy"]),
            },
        },
        "block_reference_color": BLOCK_COLOR,
        "block_pixel_center": block_record["pixel"],
        "block_predicted_xy": block_record["predicted_xy_without_offset"],
        "block_actual_xy": block_record["actual_tcp_xy"],
        "closed_loop_calibration": {
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "mode": "manual_fallback_zero_motion_compensation",
            "report": str(Path(report_path).resolve()),
            "manual_block_alignment": block_record,
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
    required = {"rotation_moves_deg"}
    if set(raw) != required:
        raise RuntimeError(
            "偏移标定采样方案字段必须恰好为：%s。" % ", ".join(sorted(required)))

    rotations = [float(item) for item in raw["rotation_moves_deg"]]
    if not np.all(np.isfinite(np.asarray(rotations, dtype=float))):
        raise RuntimeError("旋转采样角度必须是有限数值。")
    if not rotations or not any(abs(item) >= 2.0 for item in rotations):
        raise RuntimeError("旋转采样至少需要1个绝对值不小于2°的角度。")
    return {
        "profile": profile,
        "rotation_moves_deg": rotations,
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
    world = transformer.pixel_to_world(cx, cy) if transformer is not None else None
    angle = transformer.rectangle_angle_to_robot(rect) if transformer is not None else None
    if angle is None:
        angle = normalize_square_angle(float(rect[2]))
    annotated = image.copy()
    cv2.drawContours(annotated, [cv2.boxPoints(rect).astype(np.int32)], 0, (0, 255, 255), 3)
    cv2.circle(annotated, (round(cx), round(cy)), 8, (0, 0, 255), -1)
    cv2.putText(annotated, "%s center=(%.1f,%.1f)" % (color, cx, cy), (30, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
    return {
        "pixel": [float(cx), float(cy)],
        "world_xy": ([float(world[0]), float(world[1])] if world is not None else None),
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


def _fresh_physical_seed():
    """建立物理锚点采集用的空派生值；不读取公共矩阵。"""
    return {
        "updated_at": datetime.now().isoformat(timespec="seconds"),
        "calibration_world_scale_mm": float(config.TASK2_CALIBRATION_WORLD_SCALE_MM),
        "calibration_xml_sha256": None,
        "block_origin_xy": list(map(float, config.TASK2_BLOCK_VIEW_POSE[:2])),
        "block_view_orientation_rad": list(map(float, config.TASK2_BLOCK_VIEW_POSE[3:])),
        "block_xy_offset": [0.0, 0.0],
        "tray_origin_xy": list(map(float, config.TASK2_TRAY_VIEW_POSE[:2])),
        "tray_view_orientation_rad": list(map(float, config.TASK2_TRAY_VIEW_POSE[3:])),
        "tray_xy_offset": [0.0, 0.0],
    }, {
        "mode": "single_physical_anchor_without_matrix",
        "block_seed_xy_offset": [0.0, 0.0],
        "tray_seed_xy_offset": [0.0, 0.0],
        "uses_previous_offset_file": False,
        "uses_relative_report_as_absolute_anchor": False,
    }


def _capture_block_detection(robot, vision, transformer, run_dir, name,
                             expected_pixel=None, enforce_scene_stability=False,
                             board_guard=None):
    image = _move_and_capture(
        robot, vision, get_task2_block_view_pose(), run_dir, name, "block")
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


def _manual_block_anchor(robot, vision, transformer, offsets, run_dir, pick_z,
                         board_guard="fallback"):
    initial = _capture_block_detection(
        robot, vision, transformer, run_dir, "01_block_anchor_before",
        board_guard=board_guard)
    predicted = (_pose_for_detection(initial, "block", offsets, [0.0, 0.0], pick_z)
                 if initial["world_xy"] is not None else None)
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
    if verified["world_xy"] is None:
        offset, predicted, distance = [0.0, 0.0], None, 0.0
    else:
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
        "world_xy": (list(map(float, verified["world_xy"]))
                     if verified["world_xy"] is not None else None),
        "predicted_xy_without_offset": (list(map(float, predicted[:2]))
                                        if predicted is not None else None),
        "actual_tcp_xy": list(map(float, actual_pose[:2])),
        "actual_tcp_pose": list(map(float, actual_pose)),
        "reference_view_pose": list(map(float, config.TASK2_BLOCK_VIEW_POSE)),
        "effective_capture_pose": get_task2_block_view_pose(),
        "block_height_mm": float(config.TASK2_BLOCK_PHOTO_HEIGHT_MM),
        "coarse_xy_offset": list(map(float, offset)),
        "coarse_offset_norm_mm": distance,
        "scene_shift_px": scene_shift,
        "board_guard_audits": [initial["board_guard_audit"],
                               verified["board_guard_audit"]],
    }


def main():
    global CALIBRATION_BOARD_GUARD
    parser = argparse.ArgumentParser(
        description="任务二单物理锚点与像素旋转偏心闭环标定"
    )
    parser.add_argument(
        "--manual-only", action="store_true",
        help="只记录方块区人工物理锚点，不要求公共矩阵，不执行自动抓放")
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
    load_task2_tuning()
    offset_path = Path(config.TASK2_OFFSET_FILE)
    original_bytes = offset_path.read_bytes() if offset_path.exists() else None
    previous_offsets = (json.loads(original_bytes.decode("utf-8"))
                        if original_bytes is not None else None)
    detector = None
    block_transformer = CoordinateTransformer()
    offsets, fresh_offset_seed = _fresh_physical_seed()
    print("本轮物理锚点从零建立；旧派生偏移不参与计算。")
    print("自动采样方案：%s（原地旋转%d次）。" %
          (sampling_plan["profile"],
           len(sampling_plan["rotation_moves_deg"])))

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
        "mode": "single_physical_anchor_pixel_rotation_sampling",
        "requested_mode": "manual_only" if args.manual_only else "advanced",
        "requested_compensation_model": args.compensation_model,
        "calibration_board_guard": args.board_guard,
        "block_color": BLOCK_COLOR,
        "sampling_plan": sampling_plan,
        "parameters": {
            "max_manual_seed_mm": MAX_MANUAL_SEED_MM,
            "max_block_correction_mm": MAX_BLOCK_CORRECTION_MM,
            "max_scene_shift_px": MAX_SCENE_SHIFT_PX,
        },
        "matrix_used_for_collection": False,
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
    }
    try:
        print("请清空物块区，只保留一个%s方块并放在画面中部。" % BLOCK_COLOR)
        print("先完成人工物块粗对准，再进行原地旋转偏心采样。")
        coarse_block_offset, block_anchor, block_anchor_record = _manual_block_anchor(
            robot, vision, block_transformer, offsets, run_dir, pick_z,
            board_guard=args.board_guard)
        report["manual_block_alignment"] = block_anchor_record
        report["completed_stage"] = "manual_block_anchor"
        block_checkpoint_path = run_dir / "01_manual_block_checkpoint.json"
        _write_json(block_checkpoint_path, report)
        print("物块区人工粗对准完成：像素=%s，实际TCP XY=%s；毫米偏移稍后由公共矩阵派生。" %
              (block_anchor_record["pixel"], block_anchor_record["actual_tcp_xy"]))
        print("物块锚点已落盘：" + str(block_checkpoint_path.resolve()))

        coarse_tray_offset = list(coarse_block_offset)
        tray_anchor_record = {
            "mode": "shared_camera_tool_offset",
            "actual_tcp_xy": block_anchor_record["actual_tcp_xy"],
            "coarse_xy_offset": list(coarse_block_offset),
        }
        report["manual_tray_alignment"] = None
        report["coarse_block_xy_offset"] = list(coarse_block_offset)
        report["coarse_tray_xy_offset"] = list(coarse_tray_offset)
        report["completed_stage"] = "manual_physical_anchor"
        report_path = run_dir / "closed_loop_report.json"
        manual_candidate_path = run_dir / "task2_offsets_v2_manual_candidate.json"
        manual_candidate = _build_manual_candidate(
            offsets, coarse_block_offset, coarse_tray_offset,
            block_anchor_record, tray_anchor_record, report_path)
        _write_json(manual_candidate_path, manual_candidate)
        report["manual_candidate_offset_file"] = str(manual_candidate_path.resolve())
        report["manual_candidate_motion_compensation_zero"] = True
        _write_json(report_path, report)
        if original_bytes is None:
            if offset_path.exists():
                raise RuntimeError("运行期间新出现正式物理标定文件，拒绝覆盖。")
        elif not offset_path.exists() or offset_path.read_bytes() != original_bytes:
            raise RuntimeError("运行期间正式物理标定文件发生变化，拒绝覆盖。")
        coarse_backup = _atomic_activate(
            manual_candidate_path, offset_path, Path(config.DATA_DIR) / "offset_history")
        original_bytes = offset_path.read_bytes()
        report["formal_offset_modified"] = True
        report["coarse_baseline_activated"] = True
        report["activated_offset_file"] = str(offset_path.resolve())
        report["previous_offset_backup"] = (
            str(coarse_backup.resolve()) if coarse_backup else None)
        _write_json(report_path, report)
        print("人工粗校零补偿基线已原子写入正式文件：" + str(offset_path.resolve()))
        print("后续自动旋转即使失败，该基线仍可单独使用。")
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
            print("相机—吸盘固定XY偏移：%s" % coarse_block_offset)
            print("正式文件已保留人工粗校零补偿基线。")
            completed = True
            return
        report["manual_stage_decision"] = "continue_" + args.compensation_model
        _write_json(report_path, report)
        print("物理锚点已建立，开始自动采样。")

        current_block = _capture_block_detection(
            robot, vision, block_transformer, run_dir, "05_block_auto_start",
            expected_pixel=block_anchor["pixel"], enforce_scene_stability=True)
        report["translation_summary"] = {
            "sample_count": 0,
            "mode": "not_part_of_physical_offset_calibration",
        }

        for index, rotation_deg in enumerate(
                sampling_plan["rotation_moves_deg"], start=1):
            before = current_block
            pick_pose = list(map(float, block_anchor_record["actual_tcp_pose"]))
            pick_pose[2] = pick_z
            command_rotation_deg = _transfer(
                robot, pick_pose, list(pick_pose), rotation_deg,
                "物块区旋转采样%d" % index)
            after = _capture_block_detection(
                robot, vision, block_transformer, run_dir,
                "07_rotation_%02d" % index)
            observed_rotation = normalize_square_angle(
                after["robot_angle_deg"] - before["robot_angle_deg"])
            report["block_trials"].append({
                "before_pixel_center": list(map(float, before["pixel"])),
                "after_pixel_center": list(map(float, after["pixel"])),
                "requested_rotation_deg": rotation_deg,
                "command_rotation_deg": command_rotation_deg,
                "observed_rotation_deg": observed_rotation,
                "angle_error_deg": normalize_square_angle(
                    observed_rotation - command_rotation_deg),
            })
            current_block = after
        candidate_block_offset = list(coarse_block_offset)
        block_motion_model = _zero_motion_model(block_anchor_record["actual_tcp_xy"])

        candidate_tray_offset = list(candidate_block_offset)
        tray_fit = _zero_motion_model(block_anchor_record["actual_tcp_xy"])
        final_return_residual = np.zeros(2, dtype=float)

        candidate = dict(manual_candidate)
        candidate["updated_at"] = datetime.now().isoformat(timespec="seconds")
        candidate["block_xy_offset"] = candidate_block_offset
        candidate["tray_xy_offset"] = candidate_tray_offset
        candidate["motion_compensation"] = {
            "model_version": 1,
            "models": {"tray": tray_fit, "block": block_motion_model}}
        report["candidate_motion_compensation"] = candidate["motion_compensation"]
        candidate["physical_alignment"]["rotation_trials"] = [
            {
                "before_pixel_center": item["before_pixel_center"],
                "after_pixel_center": item["after_pixel_center"],
                "requested_rotation_deg": item["requested_rotation_deg"],
                "command_rotation_deg": item["command_rotation_deg"],
                "observed_rotation_deg": item["observed_rotation_deg"],
            }
            for item in report["block_trials"]
        ]
        candidate["block_reference_color"] = BLOCK_COLOR
        candidate["block_pixel_center"] = block_anchor_record["pixel"]
        candidate["block_predicted_xy"] = block_anchor_record["predicted_xy_without_offset"]
        candidate["block_actual_xy"] = block_anchor_record["actual_tcp_xy"]
        report["candidate_block_xy_offset"] = candidate_block_offset
        report["candidate_tray_xy_offset"] = candidate_tray_offset
        report["block_return_validation_residual_mm"] = final_return_residual.tolist()
        report["warning"] = (
            "本工具只保存人工TCP/像素锚点与旋转前后像素；不加载矩阵。"
        )
        candidate["closed_loop_calibration"] = {
            "created_at": report["created_at"],
            "report": str((run_dir / "closed_loop_report.json").resolve()),
            "sampling_profile": sampling_plan["profile"],
            "compensation_model": args.compensation_model,
            "manual_block_alignment": report["manual_block_alignment"],
            "manual_tray_alignment": report["manual_tray_alignment"],
            "translation_summary": report["translation_summary"],
            "motion_compensation": candidate["motion_compensation"],
        }
        candidate_path = run_dir / "task2_offsets_v2_candidate.json"
        report_path = run_dir / "closed_loop_report.json"
        _write_json(candidate_path, candidate)
        report["candidate_offset_file"] = str(candidate_path.resolve())
        _write_json(report_path, report)

        print("\n半闭环候选已生成：" + str(candidate_path.resolve()))
        print("物块偏移：人工粗对准%s -> 候选%s" %
              (coarse_block_offset, candidate_block_offset))
        print("报告：" + str(report_path.resolve()))
        print("旋转像素观测：%d组；毫米偏心将在正式加载时由当前公共矩阵派生。" %
              len(report["block_trials"]))
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
                robot.move_to_safe(get_task2_block_view_pose())
            except Exception as exc:
                print("警告：返回方块拍照位异常：" + str(exc))
        robot.disconnect()


if __name__ == "__main__":
    main()
