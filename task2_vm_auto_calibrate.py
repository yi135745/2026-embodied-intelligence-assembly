"""AUBO移动相机九点标定工具；生成VisionMaster兼容XML，不接入正式任务。"""

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


COLORS = ("红色", "橙色", "黄色", "绿色", "蓝色", "紫色")

# 蛇形顺序与现有VisionMaster九点文件的世界点排列一致，减少机器人空行程。
GRID_OFFSETS_MM = (
    (-1, -1), (0, -1), (1, -1),
    (1, 0), (0, 0), (-1, 0),
    (-1, 1), (0, 1), (1, 1),
)


def _pose_text(pose):
    return "[" + ", ".join("%.3f" % value for value in pose) + "]"


def _capture(vision, path):
    return vision.capture(
        output_name=path,
        exposure_time=config.TASK2_BLOCK_EXPOSURE_TIME,
        gain=config.TASK2_BLOCK_GAIN,
    )


def _detect_single_block(image_path, color):
    """只按指定颜色寻找最大方块，不启用六色联合识别或机器人坐标换算。"""
    image = cv2.imread(str(image_path))
    if image is None:
        raise RuntimeError("无法读取标定图片：" + str(image_path))
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    ranges = config.TASK2_BLOCK_HSV_RANGES.get(color)
    if not ranges:
        raise RuntimeError("没有配置%s方块的HSV范围。" % color)
    for lower, upper in ranges:
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))
    kernel_size = max(1, int(config.TASK2_MORPH_KERNEL))
    kernel = np.ones((kernel_size, kernel_size), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = [
        contour for contour in contours
        if config.TASK2_MIN_CONTOUR_AREA <= cv2.contourArea(contour) <= config.TASK2_MAX_CONTOUR_AREA
    ]
    if not candidates:
        raise RuntimeError("没有找到%s方块轮廓。" % color)
    contour = max(candidates, key=cv2.contourArea)
    rect = cv2.minAreaRect(contour)
    (cx, cy), _, angle = rect
    annotated = image.copy()
    box = cv2.boxPoints(rect).astype(np.int32)
    cv2.drawContours(annotated, [box], 0, (255, 255, 255), 3)
    cv2.circle(annotated, (round(cx), round(cy)), 8, (0, 0, 255), -1)
    cv2.putText(
        annotated, "center=(%.2f, %.2f) area=%.1f" % (cx, cy, cv2.contourArea(contour)),
        (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2,
    )
    return (float(cx), float(cy)), float(cv2.contourArea(contour)), float(angle), annotated, mask


def _show(title, image):
    """按屏幕可读尺寸显示，不改变保存的原始分辨率。"""
    height, width = image.shape[:2]
    scale = min(1.0, 1500.0 / width, 850.0 / height)
    preview = cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    cv2.namedWindow(title, cv2.WINDOW_NORMAL)
    cv2.imshow(title, preview)
    cv2.waitKey(150)


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
    _set_param(root, "TransNum", 9)
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


def main():
    parser = argparse.ArgumentParser(description="AUBO移动相机九点标定，输出VisionMaster兼容XML")
    parser.add_argument("--color", choices=COLORS, default="红色", help="固定参考方块颜色")
    parser.add_argument("--step-mm", type=float, default=15.0, help="九点相邻间距，默认15mm")
    parser.add_argument("--max-z-diff-mm", type=float, default=2.0,
                        help="方块/托盘拍照位最大Z差，默认2mm")
    parser.add_argument("--max-rz-diff-rad", type=float, default=0.02,
                        help="方块/托盘拍照位最大RZ差，默认0.02rad")
    parser.add_argument("--template", default=config.TASK2_CALIBRATION_FILE,
                        help="VisionMaster XML模板；只读取，不覆盖")
    args = parser.parse_args()
    if args.step_mm <= 0:
        raise RuntimeError("九点间距必须大于0。")

    records = load_aubo_pose_records()
    block_pose = records.get("TASK2_BLOCK_VIEW_POSE")
    tray_pose = records.get("TASK2_TRAY_VIEW_POSE")
    if block_pose is None or tray_pose is None:
        raise RuntimeError("aubo_poses.json缺少方块或托盘拍照位。")
    z_diff = abs(float(block_pose[2]) - float(tray_pose[2]))
    rz_diff = abs(float(block_pose[5]) - float(tray_pose[5]))
    calibration_pose = list(block_pose)
    calibration_pose[2] = (float(block_pose[2]) + float(tray_pose[2])) / 2.0
    calibration_pose[5] = (float(block_pose[5]) + float(tray_pose[5])) / 2.0
    print("方块拍照位：" + _pose_text(block_pose))
    print("托盘拍照位：" + _pose_text(tray_pose))
    print("拍照位Z差：%.4f mm；RZ差：%.6f rad" % (z_diff, rz_diff))
    if z_diff > args.max_z_diff_mm:
        raise RuntimeError("拍照位Z差超过%.3f mm，停止标定。" % args.max_z_diff_mm)
    if rz_diff > args.max_rz_diff_rad:
        raise RuntimeError("拍照位RZ差超过%.6f rad，停止标定。" % args.max_rz_diff_rad)
    print("标定中心位姿使用方块拍照位XY/RX/RY，并取两拍照位Z、RZ均值：" +
          _pose_text(calibration_pose))

    load_task2_tuning()
    run_dir = Path(config.TASK2_OUTPUT_DIR) / "vm_auto_calibration" / datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir.mkdir(parents=True, exist_ok=True)
    robot = Robot()
    if not robot.available:
        raise RuntimeError("AUBO未连接，无法执行九点标定。")
    vision = Vision()
    center_pose = None
    samples = []
    try:
        input("确认从当前位置到方块拍照位路径安全，固定方块已放好且不会移动；按回车自动前往...")
        if not robot.move_to_safe(calibration_pose):
            raise RuntimeError("移动到方块拍照位失败。")
        time.sleep(max(0.0, float(config.TASK2_SETTLE_SECONDS)))
        center_pose = robot.get_current_pose()

        check_path = _capture(vision, run_dir / "00_initial_raw.jpg")
        original = cv2.imread(str(check_path))
        _show("Initial raw image", original)
        if input("请检查原图中的方块放置和拍摄情况；确认继续请直接回车，输入n取消：").strip().lower() == "n":
            raise RuntimeError("用户取消标定。")
        center, area, angle, annotated, mask = _detect_single_block(check_path, args.color)
        cv2.imwrite(str(run_dir / "00_initial_detected.jpg"), annotated)
        cv2.imwrite(str(run_dir / "00_initial_mask.png"), mask)
        _show("Detected block center", annotated)
        if input("请检查OpenCV轮廓和中心标记；确认开始自主九点移动请直接回车，输入n取消：").strip().lower() == "n":
            raise RuntimeError("用户取消标定。")
        cv2.destroyAllWindows()

        for index, (grid_x, grid_y) in enumerate(GRID_OFFSETS_MM, start=1):
            command_pose = list(center_pose)
            command_pose[0] += grid_x * args.step_mm
            command_pose[1] += grid_y * args.step_mm
            print("\n[%d/9] XY偏移=[%.3f, %.3f] mm，目标位姿=%s" %
                  (index, grid_x * args.step_mm, grid_y * args.step_mm, _pose_text(command_pose)))
            if not robot.move_to(command_pose):
                raise RuntimeError("第%d个九点位置运动失败。" % index)
            time.sleep(max(0.0, float(config.TASK2_SETTLE_SECONDS)))
            actual_pose = robot.get_current_pose()
            raw_path = _capture(vision, run_dir / ("%02d_raw.jpg" % index))
            pixel, area, angle, marked, mask = _detect_single_block(raw_path, args.color)
            marked_path = run_dir / ("%02d_detected.jpg" % index)
            cv2.imwrite(str(marked_path), marked)
            cv2.imwrite(str(run_dir / ("%02d_mask.png" % index)), mask)
            # 固定物块相对于相机的平面位移，与相机TCP位移方向相反。
            world = [center_pose[0] - actual_pose[0], center_pose[1] - actual_pose[1]]
            sample = {
                "index": index,
                "command_offset_mm": [grid_x * args.step_mm, grid_y * args.step_mm],
                "actual_tcp_pose_mm_rad": actual_pose,
                "image_point": list(pixel),
                "world_point_mm": world,
                "area": area,
                "angle_deg": angle,
                "raw_image": str(Path(raw_path).resolve()),
                "annotated_image": str(marked_path.resolve()),
            }
            samples.append(sample)
            print("像素中心=[%.3f, %.3f]，VM世界点=[%.3f, %.3f] mm，面积=%.1f" %
                  (pixel[0], pixel[1], world[0], world[1], area))

        image_points = np.asarray([item["image_point"] for item in samples], dtype=np.float64)
        world_points = np.asarray([item["world_point_mm"] for item in samples], dtype=np.float64)
        matrix, _ = cv2.findHomography(image_points, world_points, method=0)
        if matrix is None or not np.all(np.isfinite(matrix)) or abs(float(np.linalg.det(matrix))) < 1e-15:
            raise RuntimeError("九点无法拟合有效的3×3标定矩阵。")
        xml_path = run_dir / "visionmaster_task2_calibration_candidate.xml"
        world_errors, image_errors, world_rms, pixel_rms = _write_vm_xml(
            Path(args.template), xml_path, image_points, world_points, matrix
        )
        for item, world_error, image_error in zip(samples, world_errors, image_errors):
            item["world_error_mm"] = [float(world_error[0]), float(world_error[1])]
            item["pixel_error"] = [float(image_error[0]), float(image_error[1])]
        report = {
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "mode": "eye_in_hand_fixed_block_nine_point",
            "color": args.color,
            "step_mm": args.step_mm,
            "block_view_pose_mm_rad": block_pose,
            "calibration_center_command_pose_mm_rad": calibration_pose,
            "center_actual_pose_mm_rad": center_pose,
            "tray_view_pose_mm_rad": tray_pose,
            "view_z_diff_mm": z_diff,
            "view_rz_diff_rad": rz_diff,
            "matrix_pixel_to_world_mm": matrix.tolist(),
            "world_rms_mm": world_rms,
            "pixel_rms": pixel_rms,
            "candidate_xml": str(xml_path.resolve()),
            "samples": samples,
        }
        report_path = run_dir / "calibration_report.json"
        report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
        print("\n九点标定完成，但未替换正式XML。")
        print("世界坐标RMS误差：%.6f mm；像素RMS误差：%.6f px" % (world_rms, pixel_rms))
        print("候选XML：" + str(xml_path.resolve()))
        print("完整报告：" + str(report_path.resolve()))
    finally:
        cv2.destroyAllWindows()
        if center_pose is not None and robot.available:
            print("标定结束，尝试返回中心拍照位：" + _pose_text(center_pose))
            if not robot.move_to(center_pose):
                print("警告：返回中心拍照位失败，请使用示教器确认机械臂状态。")
        robot.disconnect()


if __name__ == "__main__":
    main()
