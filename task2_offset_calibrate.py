"""连续完成任务二方块区与托盘区偏移标定，实际位姿由AUBO SDK读取。"""

import argparse
import json
import time
from datetime import datetime
from pathlib import Path

import cv2

import config
from modules.robot import Robot
from modules.task2_vision import ColorObjectDetector, load_task2_tuning
from modules.vision import Vision


COLORS = ("红色", "橙色", "黄色", "绿色", "蓝色", "紫色")


def _origin_for_scene(scene):
    specific = (config.TASK2_BLOCK_CALIBRATION_ORIGIN_POSE if scene == "block"
                else config.TASK2_TRAY_CALIBRATION_ORIGIN_POSE)
    return specific or config.TASK2_CALIBRATION_ORIGIN_POSE


def _pose_text(pose):
    return "[" + ", ".join("%.3f" % value for value in pose) + "]"


def _calibrate_scene(scene, color, vision, robot, detector, output_dir, previous_low_pose=None):
    kind = "方块" if scene == "block" else "托盘"
    origin = _origin_for_scene(scene)
    view_pose = config.TASK2_BLOCK_VIEW_POSE if scene == "block" else config.TASK2_TRAY_VIEW_POSE
    if origin is None or view_pose is None:
        raise RuntimeError("请先在config.py填写%s区域标定原点和拍照位。" % kind)

    print("\n========== %s区域偏移标定 ==========" % kind)
    print("程序将自动移动到%s拍照位：%s" % (kind, _pose_text(view_pose)))
    if previous_low_pose is None:
        input("确认机械臂到拍照位的路径安全、目标已放好后，按回车自动移动...")
    else:
        input("确认目标保持不动且周围安全后，按回车先原地抬升再自动移动到拍照位...")
        safe_pose = list(previous_low_pose)
        safe_pose[2] += float(config.TASK2_LIFT_DISTANCE_MM)
        print("先从上一低位原地抬升到：" + _pose_text(safe_pose))
        if not robot.move_to(safe_pose):
            raise RuntimeError("从上一标定低位原地抬升失败。")
    if not robot.move_to_safe(view_pose):
        raise RuntimeError("自动移动到%s拍照位失败。" % kind)
    time.sleep(max(0.0, float(config.TASK2_SETTLE_SECONDS)))
    print("已到达%s拍照位，开始拍照。" % kind)

    prefix = scene.upper()
    image_path = output_dir / ("%s_reference.jpg" % scene)
    image_path = vision.capture(
        output_name=image_path,
        exposure_time=getattr(config, "TASK2_%s_EXPOSURE_TIME" % prefix),
        gain=getattr(config, "TASK2_%s_GAIN" % prefix),
    )
    targets, annotated = detector.detect(image_path, kind, output_dir, scene)
    annotated_path = output_dir / (scene + "_offset_detected.jpg")
    cv2.imwrite(str(annotated_path), annotated)
    target = next((item for item in targets if item.color == color), None)
    if target is None:
        raise RuntimeError("没有识别到%s%s，请先完成颜色调参。" % (color, kind))

    world_xy = detector.transformer.pixel_to_world(*target.pixel_center)
    predicted_x = float(origin[0]) + world_xy[0]
    predicted_y = float(origin[1]) + world_xy[1]
    print("参考目标：%s%s" % (color, kind))
    print("像素中心：[%.3f, %.3f]" % target.pixel_center)
    print("矩阵相对XY：[%.3f, %.3f] mm" % world_xy)
    print("未补偿预测XY：[%.3f, %.3f] mm" % (predicted_x, predicted_y))
    print("标注图：" + str(annotated_path.resolve()))

    input("保持目标不动，用示教器将吸盘移到目标真实中心和正确Z高度，确认后按回车读取基座TCP...")
    actual_pose = robot.get_current_pose()
    answer = input("读取位姿%s，确认采用请直接回车；输入n取消：" % _pose_text(actual_pose)).strip().lower()
    if answer == "n":
        raise RuntimeError("用户取消%s区域标定。" % kind)

    offset = [actual_pose[0] - predicted_x, actual_pose[1] - predicted_y]
    print("%s区域：XY偏移=[%.3f, %.3f] mm，Z=%.3f mm" %
          (kind, offset[0], offset[1], actual_pose[2]))
    return {
        "%s_reference_color" % scene: color,
        "%s_pixel_center" % scene: list(target.pixel_center),
        "%s_predicted_xy" % scene: [predicted_x, predicted_y],
        "%s_actual_pose" % scene: actual_pose,
        "%s_actual_xy" % scene: actual_pose[:2],
        "%s_xy_offset" % scene: offset,
        "block_pick_z" if scene == "block" else "tray_place_z": actual_pose[2],
    }


def main():
    parser = argparse.ArgumentParser(description="连续标定方块区和托盘区，AUBO SDK自动读取实际基座坐标")
    parser.add_argument("--block-color", choices=COLORS, default="黄色")
    parser.add_argument("--tray-color", choices=COLORS, default="黄色")
    args = parser.parse_args()

    load_task2_tuning()
    output_dir = Path(config.TASK2_OUTPUT_DIR) / "offset_calibration"
    output_dir.mkdir(parents=True, exist_ok=True)
    robot = Robot()
    if not robot.available:
        raise RuntimeError("AUBO未连接，无法自动读取基座TCP位姿。")
    vision = Vision()
    detector = ColorObjectDetector()
    try:
        data = {"updated_at": datetime.now().isoformat(timespec="seconds")}
        block_data = _calibrate_scene("block", args.block_color, vision, robot, detector, output_dir)
        data.update(block_data)
        tray_data = _calibrate_scene(
            "tray", args.tray_color, vision, robot, detector, output_dir,
            previous_low_pose=block_data["block_actual_pose"],
        )
        data.update(tray_data)

        offset_path = Path(config.TASK2_OFFSET_FILE)
        print("\n即将写入双区域偏移：")
        print(json.dumps(data, ensure_ascii=False, indent=2))
        if input("两组数据确认无误后按回车写入JSON；输入n取消：").strip().lower() == "n":
            raise RuntimeError("用户取消写入，原偏移文件保持不变。")
        offset_path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
        print("双区域偏移标定完成，已写入：" + str(offset_path.resolve()))
    finally:
        robot.disconnect()


if __name__ == "__main__":
    main()
