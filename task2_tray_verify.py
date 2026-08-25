"""正式任务前单独检测并人工验收托盘；验收结果可由任务二直接复用。"""

import argparse
import json
from datetime import datetime
from pathlib import Path

import cv2

import config
from modules.robot import Robot
from modules.task2_vision import (ColorObjectDetector, load_task2_offsets,
                                  load_task2_tuning, validate_six_colors)
from modules.vision import Vision


def main():
    parser = argparse.ArgumentParser(description="托盘六色中心独立检测、人工确认和结果保存")
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--image", help="使用已有图片；默认调用相机拍照")
    parser.add_argument("--no-move-robot", action="store_true",
                        help="相机拍照前不自动移动；默认由AUBO移动到托盘拍照位")
    parser.add_argument("--no-show", action="store_true", help="不弹出OpenCV预览窗口")
    args = parser.parse_args()

    load_task2_tuning()
    load_task2_offsets()
    output_dir = Path(config.TASK2_OUTPUT_DIR) / "tray_verification"
    output_dir.mkdir(parents=True, exist_ok=True)
    robot = None
    try:
        should_move_robot = not args.image and not args.no_move_robot
        if should_move_robot:
            robot = Robot()
            if not robot.available:
                raise RuntimeError("AUBO未连接，无法移动到托盘拍照位。")
            input("确认到托盘拍照位的路径安全后按回车自动移动...")
            if not robot.move_to_safe(config.TASK2_TRAY_VIEW_POSE):
                raise RuntimeError("移动到托盘拍照位失败。")
            print("已到达托盘拍照位，开始拍照。")

        if args.image:
            image_path = Path(args.image)
        else:
            image_path = Vision().capture(
                output_name=output_dir / "verified_tray_raw.jpg",
                exposure_time=config.TASK2_TRAY_EXPOSURE_TIME,
                gain=config.TASK2_TRAY_GAIN,
            )

        targets, annotated = ColorObjectDetector().detect(image_path, "托盘", output_dir, "tray_verify")
        validate_six_colors(targets, "托盘")
        annotated_path = output_dir / "verified_tray_detected.jpg"
        cv2.imwrite(str(annotated_path), annotated)
        print("\n托盘识别结果：")
        for target in sorted(targets, key=lambda item: item.color):
            print("%s：像素=(%.1f, %.1f)，机器人XY=(%.3f, %.3f)，面积=%.0f" %
                  (target.color, target.pixel_center[0], target.pixel_center[1],
                   target.robot_pose[0], target.robot_pose[1], target.area))
        print("标注图：" + str(annotated_path.resolve()))

        if not args.no_show:
            cv2.namedWindow("Task2 Tray Verification", cv2.WINDOW_NORMAL)
            cv2.imshow("Task2 Tray Verification", annotated)
            cv2.waitKey(1)
        answer = input("肉眼确认六个框和中心均正确：输入yes保存，其他输入取消：").strip().lower()
        if not args.no_show:
            cv2.destroyAllWindows()
        if answer != "yes":
            print("已取消，正式任务不会更新托盘验收结果。")
            return

        data = {
            "verified_at": datetime.now().isoformat(timespec="seconds"),
            "source_image": str(Path(image_path).resolve()),
            "annotated_image": str(annotated_path.resolve()),
            "tray_view_pose": config.TASK2_TRAY_VIEW_POSE,
            "targets": [item.to_dict() for item in targets],
        }
        verified_path = Path(config.TASK2_VERIFIED_TRAY_FILE)
        verified_path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
        print("托盘验收结果已保存：" + str(verified_path.resolve()))
        print("正式任务将直接复用；托盘一旦移动，请重新运行本程序。")
    finally:
        if robot is not None:
            robot.disconnect()


if __name__ == "__main__":
    main()
