"""五个人工放置区域、每区局部九点的独立VM标定工具；结果只写output。"""

import argparse
import json
import time
from datetime import datetime
from pathlib import Path
from xml.etree import ElementTree

import cv2
import numpy as np

import config
from modules.pose_records import load_aubo_pose_records
from modules.robot import Robot
from modules.task2_vision import load_task2_tuning
from modules.vision import Vision
from task2_vm_auto_calibrate import (
    COLORS,
    GRID_OFFSETS_MM,
    _capture,
    _detect_single_block,
    _pose_text,
    _show,
)


REGIONS = (
    ("center", "画面中心", 15.0),
    ("left", "画面偏左但保留边距的位置", 8.0),
    ("right", "画面偏右但保留边距的位置", 8.0),
    ("top", "画面偏上但保留边距的位置", 8.0),
    ("bottom", "画面偏下但保留边距的位置", 8.0),
)


def _read_step(region_label, default_step):
    while True:
        value = input(
            "%s局部九点步长（mm，直接回车使用%.1f；物块靠边时请输入更小值）：" %
            (region_label, default_step)
        ).strip()
        if not value:
            return default_step
        try:
            step = float(value)
        except ValueError:
            print("请输入有效数字。")
            continue
        if 0 < step <= 15:
            return step
        print("步长必须大于0且不超过15mm。")


def _set_param(root, name, value):
    node = root.find(".//CalibParam[@ParamName='%s']/ParamValue" % name)
    if node is not None:
        node.text = str(value)


def _replace_points(root, name, points):
    node = root.find(".//CalibPointFListParam[@ParamName='%s']" % name)
    if node is None:
        raise RuntimeError("XML模板缺少%s。" % name)
    for child in list(node):
        node.remove(child)
    for x, y in points:
        point = ElementTree.SubElement(node, "PointF")
        ElementTree.SubElement(point, "X").text = "%.10g" % x
        ElementTree.SubElement(point, "Y").text = "%.10g" % y
        ElementTree.SubElement(point, "R").text = "0"


def _write_vm_xml(template_path, output_path, image_points, world_points, matrix):
    """写VisionMaster NPointCalib XML；点数使用全部有效样本，不固定为9。"""
    tree = ElementTree.parse(template_path)
    root = tree.getroot()
    projected_world = cv2.perspectiveTransform(
        np.asarray(image_points, dtype=np.float64).reshape(-1, 1, 2), matrix
    ).reshape(-1, 2)
    world_errors = projected_world - np.asarray(world_points, dtype=np.float64)
    inverse = np.linalg.inv(matrix)
    projected_image = cv2.perspectiveTransform(
        np.asarray(world_points, dtype=np.float64).reshape(-1, 1, 2), inverse
    ).reshape(-1, 2)
    image_errors = projected_image - np.asarray(image_points, dtype=np.float64)
    world_rms = float(np.sqrt(np.mean(np.sum(world_errors ** 2, axis=1))))
    pixel_rms = float(np.sqrt(np.mean(np.sum(image_errors ** 2, axis=1))))

    _set_param(root, "CreateCalibTime", datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
    _set_param(root, "CalibType", "NPointCalib")
    _set_param(root, "TransNum", len(image_points))
    _set_param(root, "RotNum", 0)
    _set_param(root, "CalibErrStatus", 0)
    _set_param(root, "TransError", "%.10g" % pixel_rms)
    _set_param(root, "TransWorldError", "%.10g" % world_rms)
    _set_param(root, "PixelPrecisionX", "%.10g" % float(np.sqrt(np.mean(image_errors[:, 0] ** 2))))
    _set_param(root, "PixelPrecisionY", "%.10g" % float(np.sqrt(np.mean(image_errors[:, 1] ** 2))))
    _set_param(root, "PixelPrecision", "%.10g" % pixel_rms)
    _replace_points(root, "ImagePointLst", image_points)
    _replace_points(root, "WorldPointLst", world_points)

    matrix_node = root.find(".//CalibFloatListParam[@ParamName='CalibMatrix']")
    if matrix_node is None:
        raise RuntimeError("XML模板缺少CalibMatrix。")
    for child in list(matrix_node):
        matrix_node.remove(child)
    normalized = matrix / matrix[2, 2]
    for value in normalized.reshape(-1):
        ElementTree.SubElement(matrix_node, "ParamValue").text = "%.12g" % float(value)
    ElementTree.indent(tree, space="    ")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output_path, encoding="UTF-8", xml_declaration=True)
    return world_errors, image_errors, world_rms, pixel_rms


def _confirm_detection(image_path, color, run_dir, prefix):
    original = cv2.imread(str(image_path))
    if original is None:
        raise RuntimeError("无法读取图片：" + str(image_path))
    _show(prefix + " raw", original)
    if input("检查物块位置、清晰度和画面边距；确认请回车，输入n取消：").strip().lower() == "n":
        raise RuntimeError("用户取消多位置标定。")
    pixel, area, angle, marked, mask = _detect_single_block(image_path, color)
    marked_path = run_dir / (prefix + "_initial_detected.jpg")
    cv2.imwrite(str(marked_path), marked)
    cv2.imwrite(str(run_dir / (prefix + "_initial_mask.png")), mask)
    _show(prefix + " detected", marked)
    if input("检查OpenCV轮廓和中心；确认该区域开始局部九点请回车，输入n取消：").strip().lower() == "n":
        raise RuntimeError("用户取消多位置标定。")
    cv2.destroyAllWindows()
    return pixel, area, angle


def main():
    parser = argparse.ArgumentParser(
        description="五个人工放置区域、每区局部九点，生成独立VisionMaster候选XML"
    )
    parser.add_argument("--color", choices=COLORS, default="紫色", help="参考物块颜色")
    parser.add_argument("--template", default=config.TASK2_CALIBRATION_FILE,
                        help="只读XML模板；不会覆盖")
    args = parser.parse_args()

    poses = load_aubo_pose_records()
    view_pose = poses.get("TASK2_BLOCK_VIEW_POSE")
    if view_pose is None:
        raise RuntimeError("aubo_poses.json缺少TASK2_BLOCK_VIEW_POSE。")
    load_task2_tuning()
    run_dir = (Path(config.TASK2_OUTPUT_DIR) / "vm_multi_position_calibration" /
               datetime.now().strftime("%Y%m%d_%H%M%S"))
    run_dir.mkdir(parents=True, exist_ok=True)

    robot = Robot()
    if not robot.available:
        raise RuntimeError("AUBO未连接，无法执行多位置标定。")
    vision = Vision()
    all_samples = []
    try:
        print("本工具不修改正式XML、config、偏移JSON或主程序。")
        print("五个区域依次为：中心、左、右、上、下；每区物块由人工重新放置。")
        for region_index, (region_name, region_label, default_step) in enumerate(REGIONS, start=1):
            input("\n[%d/5] 确认路径安全后按回车，机器人返回方块拍照位..." % region_index)
            if not robot.move_to_safe(view_pose):
                raise RuntimeError("返回方块拍照位失败。")
            time.sleep(max(0.0, float(config.TASK2_SETTLE_SECONDS)))
            center_camera_pose = robot.get_current_pose()
            input("请将同一个%s方块人工放到%s，保持其他同色物体离开画面；完成后按回车拍照..." %
                  (args.color, region_label))
            initial_path = _capture(vision, run_dir / ("%02d_%s_initial_raw.jpg" %
                                                       (region_index, region_name)))
            _confirm_detection(initial_path, args.color, run_dir,
                               "%02d_%s" % (region_index, region_name))
            step_mm = _read_step(region_label, default_step)
            region_samples = []

            for point_index, (grid_x, grid_y) in enumerate(GRID_OFFSETS_MM, start=1):
                command_pose = list(center_camera_pose)
                command_pose[0] += grid_x * step_mm
                command_pose[1] += grid_y * step_mm
                print("[%s %d/9] 相机XY偏移=[%.3f, %.3f] mm" %
                      (region_name, point_index, grid_x * step_mm, grid_y * step_mm))
                if not robot.move_to(command_pose):
                    raise RuntimeError("%s第%d点运动失败。" % (region_label, point_index))
                time.sleep(max(0.0, float(config.TASK2_SETTLE_SECONDS)))
                actual_camera_pose = robot.get_current_pose()
                prefix = "%02d_%s_%02d" % (region_index, region_name, point_index)
                raw_path = _capture(vision, run_dir / (prefix + "_raw.jpg"))
                pixel, area, angle, marked, mask = _detect_single_block(raw_path, args.color)
                marked_path = run_dir / (prefix + "_detected.jpg")
                cv2.imwrite(str(marked_path), marked)
                cv2.imwrite(str(run_dir / (prefix + "_mask.png")), mask)
                region_samples.append({
                    "region": region_name,
                    "region_label": region_label,
                    "point_index": point_index,
                    "step_mm": step_mm,
                    "command_offset_mm": [grid_x * step_mm, grid_y * step_mm],
                    "camera_tcp_pose_mm_rad": actual_camera_pose,
                    "image_point": list(pixel),
                    "area": area,
                    "angle_deg": angle,
                    "raw_image": str(Path(raw_path).resolve()),
                    "annotated_image": str(marked_path.resolve()),
                })

            if not robot.move_to(center_camera_pose):
                raise RuntimeError("局部九点结束后返回拍照中心失败。")
            input("保持方块不动，用示教器将吸盘移到该方块真实XY中心；完成后按回车读取TCP...")
            actual_block_pose = robot.get_current_pose()
            answer = input("读取物块位姿%s；确认采用请回车，输入n取消：" %
                           _pose_text(actual_block_pose)).strip().lower()
            if answer == "n":
                raise RuntimeError("用户取消%s数据。" % region_label)
            for sample in region_samples:
                camera_pose = sample["camera_tcp_pose_mm_rad"]
                # 固定物块基座XY减去拍照时相机TCP XY，得到当前像素对应的局部世界XY。
                sample["actual_block_pose_mm_rad"] = actual_block_pose
                sample["world_point_mm"] = [
                    actual_block_pose[0] - camera_pose[0],
                    actual_block_pose[1] - camera_pose[1],
                ]
            all_samples.extend(region_samples)
            print("%s完成，累计样本%d个。" % (region_label, len(all_samples)))

        image_points = np.asarray([item["image_point"] for item in all_samples], dtype=np.float64)
        world_points = np.asarray([item["world_point_mm"] for item in all_samples], dtype=np.float64)
        matrix, _ = cv2.findHomography(image_points, world_points, method=0)
        if matrix is None or not np.all(np.isfinite(matrix)) or abs(float(np.linalg.det(matrix))) < 1e-15:
            raise RuntimeError("多位置样本无法拟合有效3×3矩阵。")

        xml_path = run_dir / "visionmaster_task2_multi_position_candidate.xml"
        world_errors, image_errors, world_rms, pixel_rms = _write_vm_xml(
            Path(args.template), xml_path, image_points, world_points, matrix
        )
        for sample, world_error, image_error in zip(all_samples, world_errors, image_errors):
            sample["world_error_mm"] = [float(world_error[0]), float(world_error[1])]
            sample["pixel_error"] = [float(image_error[0]), float(image_error[1])]

        report = {
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "mode": "five_manual_regions_with_local_nine_point",
            "formal_project_files_modified": False,
            "color": args.color,
            "block_view_pose_mm_rad": view_pose,
            "sample_count": len(all_samples),
            "matrix_pixel_to_world_mm": matrix.tolist(),
            "world_rms_mm": world_rms,
            "pixel_rms": pixel_rms,
            "candidate_xml": str(xml_path.resolve()),
            "samples": all_samples,
        }
        report_path = run_dir / "multi_position_calibration_report.json"
        report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
        print("\n多位置候选标定完成，但未接入正式工程。")
        print("样本数：%d；世界RMS：%.6f mm；像素RMS：%.6f px" %
              (len(all_samples), world_rms, pixel_rms))
        print("候选XML：" + str(xml_path.resolve()))
        print("完整报告：" + str(report_path.resolve()))
    finally:
        cv2.destroyAllWindows()
        robot.disconnect()


if __name__ == "__main__":
    main()
