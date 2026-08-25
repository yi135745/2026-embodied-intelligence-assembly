"""任务二独立视觉调参工具。

示例：
  python task2_tuner.py --image test/test.jpg
  python task2_tuner.py --camera --scene block

窗口中拖动HSV/形态学/面积滑条，按 s 保存 task2_tuning.json，按 q 退出；
相机模式按 c 用当前曝光和增益重新拍一张。数字键1~6切换六种颜色。
"""

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np

import config
from modules.vision import Vision


COLORS = list(config.TASK2_HSV_RANGES)
WINDOW = "Task2 tuner"


def _nothing(_value):
    pass


def _set_range(color):
    lower, upper = config.TASK2_HSV_RANGES[color][0]
    for name, value in zip(("H low", "S low", "V low", "H high", "S high", "V high"), lower + upper):
        cv2.setTrackbarPos(name, WINDOW, int(value))


def _get_range():
    values = [cv2.getTrackbarPos(name, WINDOW) for name in
              ("H low", "S low", "V low", "H high", "S high", "V high")]
    return values[:3], values[3:]


def _capture(vision, scene, exposure, gain):
    path = Path(config.TASK2_OUTPUT_DIR) / ("tuning_%s.jpg" % scene)
    return vision.capture(output_name=path, exposure_time=float(exposure), gain=float(gain))


def main():
    parser = argparse.ArgumentParser(description="任务二HSV、曝光和增益人工调参")
    parser.add_argument("--image", help="使用本地图片")
    parser.add_argument("--camera", action="store_true", help="使用海康相机，曝光/增益变化后自动刷新")
    parser.add_argument("--scene", choices=("card", "block", "tray"), default="block")
    args = parser.parse_args()
    if not args.image and not args.camera:
        parser.error("必须指定 --image 或 --camera")

    if args.scene == "block":
        config.TASK2_HSV_RANGES = config.TASK2_BLOCK_HSV_RANGES
    elif args.scene == "tray":
        config.TASK2_HSV_RANGES = config.TASK2_TRAY_HSV_RANGES

    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    maxima = (179, 255, 255, 179, 255, 255)
    for name, maximum in zip(("H low", "S low", "V low", "H high", "S high", "V high"), maxima):
        cv2.createTrackbar(name, WINDOW, 0, maximum, _nothing)
    cv2.createTrackbar("Morph", WINDOW, int(config.TASK2_MORPH_KERNEL), 31, _nothing)
    cv2.createTrackbar("Min area", WINDOW, int(config.TASK2_MIN_CONTOUR_AREA), 50000, _nothing)
    cv2.createTrackbar("Max area/100", WINDOW, max(1, int(config.TASK2_MAX_CONTOUR_AREA / 100)), 10000, _nothing)
    scene_exposure = getattr(config, "TASK2_%s_EXPOSURE_TIME" % args.scene.upper())
    scene_gain = getattr(config, "TASK2_%s_GAIN" % args.scene.upper())
    initial_exposure = config.MVS_EXPOSURE_TIME if scene_exposure is None else scene_exposure
    initial_gain = config.MVS_GAIN if scene_gain is None else scene_gain
    cv2.createTrackbar("Exposure us", WINDOW, max(0, int(initial_exposure)), 100000, _nothing)
    cv2.createTrackbar("Gain x10", WINDOW, max(0, int(initial_gain * 10)), 1000, _nothing)

    color_index = 0
    _set_range(COLORS[color_index])
    vision = Vision() if args.camera else None
    image_path = _capture(vision, args.scene, cv2.getTrackbarPos("Exposure us", WINDOW),
                          cv2.getTrackbarPos("Gain x10", WINDOW) / 10.0) if args.camera else Path(args.image)
    image = cv2.imread(str(image_path))
    if image is None:
        raise RuntimeError("无法读取图片：" + str(image_path))

    last_capture_settings = (
        cv2.getTrackbarPos("Exposure us", WINDOW),
        cv2.getTrackbarPos("Gain x10", WINDOW),
    )
    settings_changed_at = float("inf")
    saved_ranges = {color: [[list(lo), list(hi)] for lo, hi in ranges]
                    for color, ranges in config.TASK2_HSV_RANGES.items()}
    while True:
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
        label = "%d:%s accepted=%d  s=save c=refresh q=quit" % (
            color_index + 1, COLORS[color_index], len(accepted)
        )
        cv2.putText(preview, label, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.imshow(WINDOW, np.hstack((cv2.resize(image, (640, 480)), cv2.resize(preview, (640, 480)))))
        key = cv2.waitKey(50) & 0xFF
        if key in (ord("q"), 27):
            break
        if ord("1") <= key <= ord("6"):
            saved_ranges[COLORS[color_index]] = [[low, high]] + saved_ranges[COLORS[color_index]][1:]
            color_index = key - ord("1")
            _set_range(COLORS[color_index])
        elif key == ord("c") and vision is not None:
            image_path = _capture(vision, args.scene, cv2.getTrackbarPos("Exposure us", WINDOW),
                                  cv2.getTrackbarPos("Gain x10", WINDOW) / 10.0)
            image = cv2.imread(str(image_path))
            settings_changed_at = float("inf")
        elif key == ord("s"):
            # 只覆盖第一段；红色默认的170~179第二段会保留。
            saved_ranges[COLORS[color_index]] = [[low, high]] + saved_ranges[COLORS[color_index]][1:]
            exposure = cv2.getTrackbarPos("Exposure us", WINDOW)
            gain = cv2.getTrackbarPos("Gain x10", WINDOW) / 10.0
            payload = {
                "%s_hsv_ranges" % args.scene: saved_ranges,
                "morph_kernel": morph,
                "min_contour_area": cv2.getTrackbarPos("Min area", WINDOW),
                "max_contour_area": cv2.getTrackbarPos("Max area/100", WINDOW) * 100,
                "capture": {
                    "%s_exposure_time" % args.scene: exposure,
                    "%s_gain" % args.scene: gain,
                },
            }
            Path(config.TASK2_TUNING_FILE).write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
            print("已保存：" + config.TASK2_TUNING_FILE)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
