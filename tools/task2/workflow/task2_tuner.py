"""任务二独立视觉调参工具。

示例：
  python tools/task2/workflow/task2_tuner.py --image <现场方块或托盘照片>
  python tools/task2/workflow/task2_tuner.py
  python tools/task2/workflow/task2_tuner.py --scene tray

窗口直接显示方块或托盘正式检测器结果。
按 s 显式保存，按 v 在正式检测完整时人工确认并保存审计；关闭窗口或按 q/ESC 直接退出且不保存。方块按1~9、托盘按1~6切换颜色，
按 [ / ] 切换同色HSV区间，按 n 新增、x 删除区间；相机模式按 c 重新拍摄。
"""

import argparse
import json
import math
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[3]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import cv2
import numpy as np

import config
from tools.task2._support import move_to_scene_view
from modules.camera import Camera
from modules.task2_perception import (
    ColorObjectDetector,
    load_task2_tuning,
    save_task2_tuning_patch,
    validate_colors,
)


WINDOW = "Task2 tuner - formal detection result"


def _nothing(_value):
    pass


def _set_range(color, ranges):
    lower, upper = ranges[color][0]
    for name, value in zip(("H low", "S low", "V low", "H high", "S high", "V high"), lower + upper):
        cv2.setTrackbarPos(name, WINDOW, int(value))


def _get_range():
    values = [cv2.getTrackbarPos(name, WINDOW) for name in
              ("H low", "S low", "V low", "H high", "S high", "V high")]
    return values[:3], values[3:]


def _commit_range(saved_ranges, color, range_index, low, high):
    if any(int(low[i]) > int(high[i]) for i in range(3)):
        raise ValueError("HSV下限不能大于上限。")
    saved_ranges[color][range_index] = [list(map(int, low)), list(map(int, high))]


def _apply_preview_config(scene, saved_ranges, morph, min_area, max_area):
    converted = {
        color: [(tuple(item[0]), tuple(item[1])) for item in ranges]
        for color, ranges in saved_ranges.items()
    }
    if scene == "block":
        config.TASK2_BLOCK_HSV_RANGES = converted
    else:
        config.TASK2_TRAY_HSV_RANGES = converted
    config.TASK2_MORPH_KERNEL = int(morph)
    config.TASK2_MIN_CONTOUR_AREA = int(min_area)
    config.TASK2_MAX_CONTOUR_AREA = int(max_area)


def _detection_preview(detector, image_path, scene, output_dir):
    kind = "方块" if scene == "block" else "托盘"
    try:
        targets, annotated = detector.detect(
            image_path, kind, include_robot_pose=False,
            strict_board=(scene == "block"))
        return targets, annotated, None
    except Exception as exc:
        image = cv2.imread(str(image_path))
        return [], image, "%s: %s" % (type(exc).__name__, exc)


def _save_human_review(output_dir, image_path, scene, targets, annotated,
                       saved_ranges):
    kind = "方块" if scene == "block" else "托盘"
    validate_colors(targets, kind)
    output_dir.mkdir(parents=True, exist_ok=True)
    annotated_path = output_dir / ("%s_human_verified.jpg" % scene)
    cv2.imwrite(str(annotated_path), annotated)
    review = {
        "confirmed_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "scene": scene,
        "source_image": str(Path(image_path).resolve()),
        "annotated_image": str(annotated_path.resolve()),
        "detected_colors": [target.color for target in targets],
        "detections": [target.to_dict() for target in targets],
        "hsv_ranges": saved_ranges,
        "operator_visual_confirmation": True,
    }
    review_path = output_dir / ("%s_human_review.json" % scene)
    review_path.write_text(json.dumps(review, ensure_ascii=False, indent=2),
                           encoding="utf-8")
    return review, review_path


def _sample_confirmed_hsv_prototypes(image, targets):
    """从人工确认目标的内部圆形区域取中位数，避开边缘、高光和背景。"""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    result = {}
    for target in targets:
        radius = max(3, round(math.sqrt(max(1.0, target.area)) * 0.20))
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        center = tuple(round(value) for value in target.pixel_center)
        cv2.circle(mask, center, radius, 255, -1)
        pixels = hsv[mask > 0]
        if len(pixels):
            result[target.color] = [np.median(pixels, axis=0).round(1).tolist()]
    return result


def _swap_red_pink_prototypes(prototypes):
    """人工确认身份颠倒时交换分类原型；HSV候选范围保持不变。"""
    if "红色" not in prototypes or "粉色" not in prototypes:
        raise ValueError("红色或粉色分类原型缺失，无法对调。")
    swapped = dict(prototypes)
    swapped["红色"], swapped["粉色"] = prototypes["粉色"], prototypes["红色"]
    return swapped


def _capture(vision, scene, exposure, gain):
    path = Path(config.TASK2_OUTPUT_DIR) / ("tuning_%s.jpg" % scene)
    return vision.capture(output_name=path, exposure_time=float(exposure), gain=float(gain))


def _build_tuning_payload(scene, saved_ranges, morph, min_area, max_area,
                          exposure, gain):
    """构造可安全缓存的保存快照，避免窗口关闭后再读取轨道条。"""
    payload = {
        "%s_hsv_ranges" % scene: json.loads(json.dumps(saved_ranges)),
        "morph_kernel": int(morph),
        "min_contour_area": int(min_area),
        "max_contour_area": int(max_area),
        "capture": {
            "%s_exposure_time" % scene: float(exposure),
            "%s_gain" % scene: float(gain),
        },
    }
    if scene == "block":
        payload["block_color_prototypes_hsv"] = {
            color: [list(item) for item in values]
            for color, values in config.TASK2_BLOCK_COLOR_PROTOTYPES_HSV.items()
        }
    return payload


def _window_open():
    """窗口被 X 关闭时停止读取轨道条，避免 OpenCV NULL window 异常。"""
    try:
        return cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) >= 1
    except cv2.error:
        return False


def _move_to_block_view():
    """兼容旧调用；无确认提示，调试入口默认执行。"""
    move_to_scene_view("block")


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="任务二方块/托盘HSV、曝光和增益人工调参")
    parser.add_argument("--image", help="使用本地图片")
    parser.add_argument("--camera", action="store_true", help="使用海康相机（已是默认模式）")
    parser.add_argument("--scene", choices=("block", "tray"), default="block")
    parser.add_argument("--move-to-block", action="store_true",
                        help=argparse.SUPPRESS)
    parser.add_argument("--no-move", action="store_true", help="相机模式下保持机器人当前位置")
    args = parser.parse_args(argv)
    if args.image and args.camera:
        parser.error("--image 与 --camera 不能同时指定")
    if args.move_to_block and (args.image or args.scene != "block"):
        parser.error("--move-to-block 仅支持物块相机模式")
    if args.move_to_block and args.no_move:
        parser.error("--move-to-block 与 --no-move 不能同时指定")
    if args.image and args.no_move:
        parser.error("图片模式不需要 --no-move")
    return args


def main():
    args = parse_args()
    use_camera = not args.image
    if use_camera and not args.no_move:
        move_to_scene_view(args.scene)

    load_task2_tuning()
    ranges = config.TASK2_BLOCK_HSV_RANGES if args.scene == "block" else config.TASK2_TRAY_HSV_RANGES
    colors = list(ranges)
    saved_ranges = {color: [[list(lo), list(hi)] for lo, hi in values]
                    for color, values in ranges.items()}

    scene_exposure = getattr(config, "TASK2_%s_EXPOSURE_TIME" % args.scene.upper())
    scene_gain = getattr(config, "TASK2_%s_GAIN" % args.scene.upper())
    initial_exposure = config.MVS_EXPOSURE_TIME if scene_exposure is None else scene_exposure
    initial_gain = config.MVS_GAIN if scene_gain is None else scene_gain
    initial_exposure = max(0, int(initial_exposure))
    initial_gain_x10 = max(0, int(initial_gain * 10))

    color_index = 0
    range_index = 0
    vision = Camera() if use_camera else None
    detector = ColorObjectDetector()
    review_dir = Path(config.TASK2_OUTPUT_DIR) / "tuning_review"
    # 首帧采集结束后再显示窗口，避免相机阻塞时留下空白/无响应窗口。
    image_path = (_capture(vision, args.scene, initial_exposure, initial_gain_x10 / 10.0)
                  if use_camera else Path(args.image))
    image = cv2.imread(str(image_path))
    if image is None:
        raise RuntimeError("无法读取图片：" + str(image_path))

    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW, 1100, 900)
    maxima = (179, 255, 255, 179, 255, 255)
    for name, maximum in zip(("H low", "S low", "V low", "H high", "S high", "V high"), maxima):
        cv2.createTrackbar(name, WINDOW, 0, maximum, _nothing)
    cv2.createTrackbar("Morph", WINDOW, int(config.TASK2_MORPH_KERNEL), 31, _nothing)
    cv2.createTrackbar("Min area", WINDOW, int(config.TASK2_MIN_CONTOUR_AREA), 50000, _nothing)
    cv2.createTrackbar("Max area/100", WINDOW, max(1, int(config.TASK2_MAX_CONTOUR_AREA / 100)), 10000, _nothing)
    cv2.createTrackbar("Exposure us", WINDOW, initial_exposure, 100000, _nothing)
    cv2.createTrackbar("Gain x10", WINDOW, initial_gain_x10, 1000, _nothing)
    _set_range(colors[color_index], {
        colors[color_index]: saved_ranges[colors[color_index]][range_index:range_index + 1]})
    last_capture_settings = (initial_exposure, initial_gain_x10)
    settings_changed_at = float("inf")
    preview_signature = None
    last_preview_payload = None
    targets, detected, detection_error = [], image.copy(), "尚未运行正式检测"
    cv2.imshow(WINDOW, cv2.resize(image, (640, 480)))
    cv2.waitKey(1)
    while _window_open():
        current_capture_settings = (
            cv2.getTrackbarPos("Exposure us", WINDOW),
            cv2.getTrackbarPos("Gain x10", WINDOW),
        )
        if current_capture_settings != last_capture_settings:
            last_capture_settings = current_capture_settings
            settings_changed_at = time.monotonic()
        # 停止拖动0.35秒后重新拍摄，避免每个滑条刻度都反复打开相机。
        if vision is not None and time.monotonic() - settings_changed_at >= 0.35:
            exposure, gain_x10 = current_capture_settings
            image_path = _capture(vision, args.scene, exposure, gain_x10 / 10.0)
            if not _window_open():
                break
            refreshed = cv2.imread(str(image_path))
            if refreshed is not None:
                image = refreshed
            settings_changed_at = float("inf")
        low, high = _get_range()
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array(low), np.array(high))
        morph = max(1, cv2.getTrackbarPos("Morph", WINDOW))
        kernel = np.ones((morph, morph), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        preview = cv2.bitwise_and(image, image, mask=mask)
        min_area = cv2.getTrackbarPos("Min area", WINDOW)
        max_area = cv2.getTrackbarPos("Max area/100", WINDOW) * 100
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        accepted = [c for c in contours if min_area <= cv2.contourArea(c) <= max_area]
        cv2.drawContours(preview, accepted, -1, (255, 255, 255), 2)
        try:
            _commit_range(saved_ranges, colors[color_index], range_index, low, high)
            _apply_preview_config(args.scene, saved_ranges, morph, min_area, max_area)
            last_preview_payload = _build_tuning_payload(
                args.scene, saved_ranges, morph, min_area, max_area,
                current_capture_settings[0], current_capture_settings[1] / 10.0)
            signature = (json.dumps(saved_ranges, sort_keys=True), morph, min_area,
                         max_area, Path(image_path).stat().st_mtime_ns)
            if signature != preview_signature:
                targets, detected, detection_error = _detection_preview(
                    detector, image_path, args.scene, review_dir)
                preview_signature = signature
                if detection_error:
                    print("[正式检测预览] 失败：" + detection_error)
                else:
                    print("[正式检测预览] " + ", ".join(
                        "%s@(%.0f,%.0f)" %
                        (target.color, target.pixel_center[0], target.pixel_center[1])
                        for target in targets))
        except ValueError as exc:
            targets, detected, detection_error = [], image.copy(), str(exc)
        label = "%d:%s range=%d/%d mask=%d detect=%d  r=swap s=save v=verify" % (
            color_index + 1, colors[color_index], range_index + 1,
            len(saved_ranges[colors[color_index]]), len(accepted), len(targets)
        )
        display = detected.copy()
        cv2.rectangle(display, (0, 0), (display.shape[1], 70), (0, 0, 0), -1)
        cv2.putText(display, label, (20, 45), cv2.FONT_HERSHEY_SIMPLEX,
                    0.85, (255, 255, 255), 2)
        if detection_error:
            cv2.putText(display, ("DETECT ERROR: " + detection_error)[:100], (20, 68),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 0, 255), 2)
        cv2.imshow(WINDOW, cv2.resize(display, (960, 720)))
        key = cv2.waitKey(50) & 0xFF
        if key in (ord("q"), 27):
            break
        if ord("1") <= key < ord("1") + len(colors):
            color_index = key - ord("1")
            range_index = 0
            _set_range(colors[color_index], {
                colors[color_index]: saved_ranges[colors[color_index]][:1]})
        elif key in (ord("["), ord("]")):
            count = len(saved_ranges[colors[color_index]])
            range_index = ((range_index - 1) if key == ord("[") else
                           (range_index + 1)) % count
            _set_range(colors[color_index], {
                colors[color_index]: saved_ranges[colors[color_index]][range_index:range_index + 1]})
        elif key == ord("n"):
            saved_ranges[colors[color_index]].append([list(low), list(high)])
            range_index = len(saved_ranges[colors[color_index]]) - 1
            print("已为%s新增第%d段HSV。" % (colors[color_index], range_index + 1))
        elif key == ord("x"):
            if len(saved_ranges[colors[color_index]]) <= 1:
                print("每种颜色至少保留一段HSV，不能删除。")
            else:
                saved_ranges[colors[color_index]].pop(range_index)
                range_index = min(range_index, len(saved_ranges[colors[color_index]]) - 1)
                _set_range(colors[color_index], {
                    colors[color_index]: saved_ranges[colors[color_index]][range_index:range_index + 1]})
        elif key == ord("r") and args.scene == "block":
            try:
                config.TASK2_BLOCK_COLOR_PROTOTYPES_HSV = _swap_red_pink_prototypes(
                    config.TASK2_BLOCK_COLOR_PROTOTYPES_HSV)
                preview_signature = None
                print("已人工对调红色/粉色分类原型；请核对当前正式检测窗口，"
                      "确认正确后按v固化现场原型。")
            except ValueError as exc:
                print("红粉对调失败：" + str(exc))
        elif key == ord("c") and vision is not None:
            image_path = _capture(vision, args.scene, cv2.getTrackbarPos("Exposure us", WINDOW),
                                  cv2.getTrackbarPos("Gain x10", WINDOW) / 10.0)
            image = cv2.imread(str(image_path))
            settings_changed_at = float("inf")
        elif key in (ord("s"), ord("v")):
            payload = dict(last_preview_payload)
            if key == ord("v"):
                try:
                    review, review_path = _save_human_review(
                        review_dir, image_path, args.scene, targets, detected,
                        saved_ranges)
                except Exception as exc:
                    print("人工确认失败，当前正式检测结果不完整：%s" % exc)
                    continue
                payload["human_reviews"] = {args.scene: review}
                if args.scene == "block":
                    prototypes = _sample_confirmed_hsv_prototypes(image, targets)
                    payload["block_color_prototypes_hsv"] = prototypes
                    config.TASK2_BLOCK_COLOR_PROTOTYPES_HSV = {
                        **config.TASK2_BLOCK_COLOR_PROTOTYPES_HSV,
                        **{color: tuple(tuple(item) for item in values)
                           for color, values in prototypes.items()},
                    }
                    preview_signature = None
                print("人工肉眼确认记录已保存：" + str(review_path.resolve()))
            save_tuning(payload)
            last_preview_payload = payload
            print("已保存：" + config.TASK2_TUNING_FILE)
    cv2.destroyAllWindows()


def save_tuning(payload):
    """保存一个分区时保留另一个分区的阈值与曝光。"""
    save_task2_tuning_patch(payload)


if __name__ == "__main__":
    main()
