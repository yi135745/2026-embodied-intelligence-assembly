"""一次九点、最多54组托盘中心，生成方块/托盘共用标定XML。"""

import argparse
import json
import sys
import time
from collections import defaultdict
from datetime import datetime
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[3]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import cv2
import numpy as np

import config
from runtime.site_data import load_aubo_pose_records
from modules.robot import Robot
from modules.task2_calibration import (
    GRID_OFFSETS_MM,
    activate_calibration,
    fit_relative_homography,
    print_activation_summary,
    write_vm_xml,
)
from modules.task2_perception import (ColorObjectDetector,
                                  detect_tray_landmarks_file,
                                  detect_tray_landmarks_tracked,
                                  load_task2_tuning)
from modules.camera import Camera
from runtime.task2_state import require_shared_calibration_view_compatibility


# 首帧从记录的托盘拍照中心建立完整六槽位，之后再走周围八点。
TRAY_GRID_OFFSETS = ((0, 0),) + tuple(item for item in GRID_OFFSETS_MM if item != (0, 0))


def _capture(vision, path):
    return vision.capture(
        output_name=path,
        exposure_time=config.TASK2_TRAY_EXPOSURE_TIME,
        gain=config.TASK2_TRAY_GAIN,
    )


def _pose_rms(samples):
    grouped = defaultdict(list)
    for sample in samples:
        grouped[int(sample["point_index"])].append(np.asarray(sample["world_error_mm"], dtype=float))
    return {
        str(index): float(np.sqrt(np.mean(np.sum(np.asarray(errors) ** 2, axis=1))))
        for index, errors in sorted(grouped.items())
    }


def _usable_samples(samples):
    """丢弃观测少于5次的托盘；至少保留跨两行、两列的3组。"""
    counts = defaultdict(int)
    for sample in samples:
        counts[sample["region"]] += 1
    usable_regions = {region for region, count in counts.items() if count >= 5}
    if len(usable_regions) < 3:
        raise ValueError("有效托盘组不足3个（每组至少需要5次观测）：%s" % dict(counts))
    rows = {"top" if region.startswith("top_") else "bottom"
            for region in usable_regions}
    columns = {region.rsplit("_", 1)[-1] for region in usable_regions}
    if len(rows) < 2 or len(columns) < 2:
        raise ValueError("有效托盘必须至少覆盖上下两行和两个不同列，不能集中在同一排/同一侧。")
    selected = [dict(item) for item in samples if item["region"] in usable_regions]
    pose_counts = defaultdict(int)
    for sample in selected:
        pose_counts[int(sample["point_index"])] += 1
    if len(pose_counts) != 9 or any(count < 3 for count in pose_counts.values()):
        raise ValueError("九个相机位姿各自至少需要3个有效托盘中心：%s" % dict(pose_counts))
    excluded = [dict(item) for item in samples if item["region"] not in usable_regions]
    return selected, excluded, dict(counts), dict(pose_counts)


def _build_candidate(samples, template, run_dir, view_pose, source_report=None):
    """纯离线拟合有效观测并写兼容VisionMaster的XML，不触碰正式文件。"""
    raw_samples = [dict(item) for item in samples]
    samples, excluded_samples, raw_region_counts, pose_counts = _usable_samples(raw_samples)
    region_count = len({item["region"] for item in samples})
    matrix, world_points, _fit_errors, details = fit_relative_homography(
        samples, cross_validate=region_count >= 4)
    details = dict(details)
    tray_positions = details.pop("region_estimated_block_xy")
    image_points = np.asarray([item["image_point"] for item in samples], dtype=np.float64)
    xml_path = run_dir / "visionmaster_task2_tray_candidate.xml"
    world_errors, image_errors, world_rms, pixel_rms = write_vm_xml(
        Path(template), xml_path, image_points, world_points, matrix
    )
    for sample, world, world_error, image_error in zip(
            samples, world_points, world_errors, image_errors):
        sample["world_point_mm"] = [float(value) for value in world]
        sample["world_error_mm"] = [float(value) for value in world_error]
        sample["pixel_error"] = [float(value) for value in image_error]
    color_sources = {"color_bootstrap", "hsv_near_predicted_slot"}
    color_sample_count = sum(
        item.get("detection_source") in color_sources for item in samples)
    report = {
        "created_at": datetime.now().isoformat(timespec="seconds"),
        **details,
        "mode": "single_9_pose_up_to_6_trays_geometry_hsv_spatial_fallback",
        "formal_project_files_modified": False,
        "tray_view_pose_mm_rad": view_pose,
        "camera_pose_count": len({item["point_index"] for item in samples}),
        "tray_landmark_count": len({item["region"] for item in samples}),
        "maximum_sample_count": 54,
        "raw_sample_count": len(raw_samples),
        "sample_count": len(samples),
        "excluded_sample_count": len(excluded_samples),
        "raw_region_sample_counts": raw_region_counts,
        "per_camera_pose_sample_counts": pose_counts,
        "matrix_pixel_to_world_mm": matrix.tolist(),
        "world_rms_mm": world_rms,
        "pixel_rms": pixel_rms,
        "per_camera_pose_world_rms_mm": _pose_rms(samples),
        "tray_estimated_xy": tray_positions,
        "candidate_xml": str(xml_path.resolve()),
        "source_report_for_offline_replay": str(source_report) if source_report else None,
        "color_identity_used_for_correspondence": False,
        "color_used_for_calibration": color_sample_count > 0,
        "color_spatial_supplement_sample_count": color_sample_count,
        "color_bootstrap_sample_count": sum(
            item.get("detection_source") == "color_bootstrap" for item in samples),
        "hsv_spatial_fallback_sample_count": sum(
            item.get("detection_source") == "hsv_near_predicted_slot" for item in samples),
        "requires_physical_alignment_combination": True,
        "six_observations_at_one_pose_share_camera_pose_error": True,
        "samples": samples,
        "excluded_samples": excluded_samples,
    }
    report_path = run_dir / "tray_calibration_report.json"
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print("\n托盘九点候选已生成；未做独立落点验收。")
    print("相机位姿：%d；托盘组：%d；样本：%d" %
          (report["camera_pose_count"], report["tray_landmark_count"], len(samples)))
    print("组内世界RMS：%.6f mm；像素RMS：%.6f px" % (world_rms, pixel_rms))
    if "leave_one_region_out_relative_rms_mm" in details:
        print("留出整个托盘的相对运动RMS：" +
              str(details["leave_one_region_out_relative_rms_mm"]))
    else:
        print("仅3个有效托盘组，无法留出整个托盘交叉验证；候选降级为低置信度。")
    print("各相机位姿世界RMS：" + str(report["per_camera_pose_world_rms_mm"]))
    print("候选XML：" + str(xml_path.resolve()))
    print("完整报告：" + str(report_path.resolve()))
    return xml_path, report_path, report


def main():
    parser = argparse.ArgumentParser(
        description="托盘区一次自动九点；每帧至少3个中心，最多54组，生成共用XML"
    )
    parser.add_argument("--template", default=config.TASK2_CALIBRATION_TEMPLATE_FILE,
                        help="独立只读XML模板")
    parser.add_argument("--from-report", help="离线重算已有托盘报告；不连接机器人、不激活XML")
    args = parser.parse_args()

    run_dir = (Path(config.TASK2_OUTPUT_DIR) / "tray_auto_calibration" /
               datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
    run_dir.mkdir(parents=True, exist_ok=False)
    if args.from_report:
        source_path = Path(args.from_report).resolve()
        source = json.loads(source_path.read_text(encoding="utf-8"))
        _build_candidate(source["samples"], args.template, run_dir,
                         source.get("tray_view_pose_mm_rad"), source_report=source_path)
        print("离线重算完成；未修改正式托盘XML或XY偏移。")
        return

    records = load_aubo_pose_records()
    view_pose = records.get("TASK2_TRAY_VIEW_POSE")
    block_pose = records.get("TASK2_BLOCK_VIEW_POSE")
    if view_pose is None:
        raise RuntimeError("aubo_poses.json缺少TASK2_TRAY_VIEW_POSE。")
    compatibility = require_shared_calibration_view_compatibility(block_pose, view_pose)
    print("共用矩阵位姿检查通过：Z差=%.3f mm，RZ差=%.6f rad。" %
          (compatibility["z_diff_mm"], compatibility["rz_diff_rad"]))
    step_mm = float(config.TASK2_TRAY_CALIBRATION_STEP_MM)
    if step_mm <= 0:
        raise RuntimeError("TASK2_TRAY_CALIBRATION_STEP_MM必须大于0。")

    robot = Robot()
    if not robot.available:
        raise RuntimeError("AUBO未连接，无法执行托盘九点标定。")
    vision = Camera()
    load_task2_tuning()
    color_detector = ColorObjectDetector()
    completed = False
    all_samples = []
    reference_landmarks = None
    try:
        print("程序将自动到托盘拍照位并连续完成九点；首帧建立六槽位，后续每帧至少3个真实中心。")
        print("托盘身份按空间槽位固定；几何优先，HSV只能在预测槽位附近补充中心，不按颜色建立对应。")
        if not robot.move_to_safe(view_pose):
            raise RuntimeError("自动移动到托盘拍照位失败。")
        time.sleep(max(0.0, float(config.TASK2_SETTLE_SECONDS)))
        center_camera_pose = robot.get_current_pose()

        for point_index, (grid_x, grid_y) in enumerate(TRAY_GRID_OFFSETS, start=1):
            command_pose = list(center_camera_pose)
            command_pose[0] += grid_x * step_mm
            command_pose[1] += grid_y * step_mm
            print("[%d/9] 相机XY偏移=[%.3f, %.3f] mm" %
                  (point_index, grid_x * step_mm, grid_y * step_mm))
            if not robot.move_to(command_pose):
                raise RuntimeError("托盘标定第%d点运动失败。" % point_index)
            time.sleep(max(0.0, float(config.TASK2_SETTLE_SECONDS)))
            actual_camera_pose = robot.get_current_pose()
            prefix = "%02d" % point_index
            raw_path = _capture(vision, run_dir / (prefix + "_raw.jpg"))
            supplemental_centers = []
            try:
                color_targets, _color_annotated = color_detector.detect(
                    raw_path, "托盘", include_robot_pose=False)
                supplemental_centers = [target.pixel_center for target in color_targets]
            except Exception as exc:
                print("第%d点颜色视觉补充不可用，仅保留几何中心：%s" %
                      (point_index, exc))
            if reference_landmarks is None:
                landmarks, annotated, edges, grid_score = detect_tray_landmarks_file(
                    raw_path, supplemental_centers)
                reference_landmarks = landmarks
                geometry_count = sum(
                    landmark.source == "geometry" for landmark in landmarks)
                tracking = {
                    "observed_count": 6,
                    "geometry_count": geometry_count,
                    "supplement_count": 6 - geometry_count,
                    "common_image_shift_px": [0.0, 0.0], "geometry_match_rms_px": 0.0,
                    "grid_score": grid_score,
                }
            else:
                landmarks, annotated, edges, tracking = detect_tray_landmarks_tracked(
                    cv2.imread(str(raw_path)), reference_landmarks, supplemental_centers)
                grid_score = None
            annotated_path = run_dir / (prefix + "_detected.jpg")
            cv2.imwrite(str(annotated_path), annotated)
            cv2.imwrite(str(run_dir / (prefix + "_edges.png")), edges)
            if len(landmarks) < 3:
                raise RuntimeError("第%d点只有%d个可靠托盘中心，少于最低3个。" %
                                   (point_index, len(landmarks)))
            for landmark in landmarks:
                all_samples.append({
                    "region": landmark.slot,
                    "region_label": landmark.slot,
                    "point_index": point_index,
                    "step_mm": step_mm,
                    "command_offset_mm": [grid_x * step_mm, grid_y * step_mm],
                    "camera_tcp_pose_mm_rad": actual_camera_pose,
                    "image_point": list(landmark.center),
                    "nested_square_support": landmark.support_count,
                    "median_square_side_px": landmark.median_side_px,
                    "detection_source": landmark.source,
                    "grid_score": grid_score,
                    "tracking": tracking,
                    "raw_image": str(Path(raw_path).resolve()),
                    "annotated_image": str(annotated_path.resolve()),
                })
            print("第%d点采用%d/6（几何%d，颜色视觉补充%d），累计%d/54。" %
                  (point_index, len(landmarks), tracking["geometry_count"],
                   tracking["supplement_count"], len(all_samples)))

        xml_path, report_path, report = _build_candidate(
            all_samples, args.template, run_dir, view_pose
        )
        report["shared_view_compatibility"] = compatibility
        report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
        decision = print_activation_summary(
            "方块/托盘共用九点矩阵", config.TASK2_CALIBRATION_FILE, xml_path,
            config.TASK2_OUTPUT_DIR, report,
            "物理锚点保持有效；运行时将用新矩阵重新组合，并须重做落点验收。")
        if decision["issues"]:
            print("候选未通过质量门禁，已禁止覆盖正式共用矩阵。")
            answer = ""
        else:
            answer = input(
                "确认上述信息并已核对9张标注图后，输入 yes 备份旧版并覆盖正式共用XML；其他输入保留候选："
            ).strip().lower()
        if answer == "yes":
            backup = activate_calibration(
                xml_path,
                active=config.TASK2_CALIBRATION_FILE,
                history_dir=config.TASK2_CALIBRATION_HISTORY_DIR,
                binding_file=config.TASK2_CALIBRATION_BINDING_FILE,
                quality_report=report,
                persistent_report_file=config.TASK2_CALIBRATION_REPORT_FILE,
            )
            report["formal_project_files_modified"] = True
            report["activated_xml"] = str(Path(config.TASK2_CALIBRATION_FILE).resolve())
            report["previous_xml_backup"] = str(backup) if backup else None
            report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
            print("正式方块/托盘共用标定已更新：" + report["activated_xml"])
            print("旧版备份：" + (str(backup) if backup else "无"))
            print("公共矩阵已更新；已记录的原始物理锚点可重新组合，无需按矩阵顺序重做人工粗校。")
        else:
            print("未覆盖正式共用标定XML。")
        completed = True
    finally:
        cv2.destroyAllWindows()
        if completed and robot.available:
            print("托盘标定结束，自动返回托盘拍照位。")
            try:
                if not robot.move_to_safe(view_pose):
                    print("警告：返回托盘拍照位失败，请检查机器人状态。")
            except Exception as exc:
                print("警告：返回托盘拍照位异常：" + str(exc))
        robot.disconnect()


if __name__ == "__main__":
    main()
