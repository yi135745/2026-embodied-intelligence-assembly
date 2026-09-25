"""任务二分区视觉独立测试。

示例：
  python tools/task2/diagnostics/task2_vision_test.py
  python tools/task2/diagnostics/task2_vision_test.py --scene block --camera
  python tools/task2/diagnostics/task2_vision_test.py --scene tray --image output/task2/task2_trays.jpg
  python tools/task2/diagnostics/task2_vision_test.py --scene card --camera

每次运行都会在 output/task2/diagnostics/<时间_区域>/ 保存原图、标注图、
六张颜色mask、检测JSON和summary.txt；即使颜色不完整也会保存。
"""

import argparse
import json
import shutil
import sys
from datetime import datetime
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[3]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import cv2
import numpy as np

import config
from modules.task2_perception import ColorObjectDetector, load_task2_tuning
from modules.camera import Camera
from runtime.site_data import apply_aubo_pose_records
from tools.task2._support import move_to_scene_view


def _scene_config(scene):
    prefix = scene.upper()
    return (
        getattr(config, "TASK2_%s_EXPOSURE_TIME" % prefix),
        getattr(config, "TASK2_%s_GAIN" % prefix),
    )


def _color_statistics(image_path, targets):
    """取每个目标中心圆形内区，避开高光边缘，供多次实拍比较HSV漂移。"""
    image = cv2.imread(str(image_path))
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
    result = {}
    for target in targets:
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        radius = max(8, round(.18 * np.sqrt(target.area)))
        center = tuple(round(value) for value in target.pixel_center)
        cv2.circle(mask, center, radius, 255, -1)
        hsv_pixels, lab_pixels = hsv[mask > 0], lab[mask > 0]
        result[target.color] = {
            "center": list(target.pixel_center),
            "sample_radius_px": radius,
            "hsv_p10": np.percentile(hsv_pixels, 10, axis=0).round(1).tolist(),
            "hsv_median": np.median(hsv_pixels, axis=0).round(1).tolist(),
            "hsv_p90": np.percentile(hsv_pixels, 90, axis=0).round(1).tolist(),
            "lab_median": np.median(lab_pixels, axis=0).round(1).tolist(),
        }
    return result


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="单独检查任务卡、方块区或托盘区视觉效果")
    parser.add_argument("--scene", choices=("card", "block", "tray"), default="block")
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--camera", action="store_true")
    source.add_argument("--image")
    parser.add_argument("--no-move", action="store_true", help="相机模式下保持机器人当前位置")
    parser.add_argument("--show", action="store_true", help="弹窗显示原图/标注图，按任意键关闭")
    args = parser.parse_args(argv)
    if args.image and args.no_move:
        parser.error("图片模式不需要 --no-move")
    return args


def main():
    args = parse_args()
    use_camera = not args.image

    load_task2_tuning()
    if use_camera and not args.no_move:
        apply_aubo_pose_records()
        move_to_scene_view(args.scene)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = Path(config.TASK2_OUTPUT_DIR) / "diagnostics" / (stamp + "_" + args.scene)
    output_dir.mkdir(parents=True, exist_ok=True)
    raw_path = output_dir / (args.scene + "_raw.jpg")

    if use_camera:
        exposure, gain = _scene_config(args.scene)
        image_path = Camera().capture(output_name=raw_path, exposure_time=exposure, gain=gain)
    else:
        image_path = Path(args.image)
        shutil.copy2(image_path, raw_path)
        image_path = raw_path

    if args.scene == "card":
        summary = "任务卡只检查原图清晰度，不执行HSV识别。\n原图：%s\n" % raw_path
        (output_dir / "summary.txt").write_text(summary, encoding="utf-8")
        print(summary)
        if args.show:
            cv2.imshow("task card", cv2.imread(str(raw_path)))
            cv2.waitKey(0)
        return

    kind = "方块" if args.scene == "block" else "托盘"
    targets, annotated = ColorObjectDetector().detect(
        image_path, kind, output_dir, args.scene, include_robot_pose=False,
        strict_board=(kind == "方块")
    )
    annotated_path = output_dir / (args.scene + "_detected.jpg")
    cv2.imwrite(str(annotated_path), annotated)
    found = {item.color for item in targets}
    expected = set(config.TASK2_BLOCK_COLORS if kind == "方块" else config.TASK2_TRAY_COLORS)
    if kind == "方块":
        expected -= set(config.TASK2_DISABLED_BLOCK_COLORS)
    result = [item.to_dict() for item in targets]
    (output_dir / "detections.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    statistics = _color_statistics(image_path, targets)
    (output_dir / "color_statistics.json").write_text(
        json.dumps(statistics, ensure_ascii=False, indent=2), encoding="utf-8")
    summary = (
        "区域：%s\n识别到：%s\n缺少：%s\n原图：%s\n标注图：%s\n颜色统计：%s\n"
        % (kind, sorted(found), sorted(expected - found), raw_path, annotated_path,
           output_dir / "color_statistics.json")
    )
    (output_dir / "summary.txt").write_text(summary, encoding="utf-8")
    print(summary)
    if args.show:
        left = cv2.resize(cv2.imread(str(raw_path)), (640, 480))
        right = cv2.resize(annotated, (640, 480))
        cv2.imshow("raw | detected", cv2.hconcat([left, right]))
        cv2.waitKey(0)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
