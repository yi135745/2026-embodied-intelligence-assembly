"""任务卡拍照参数调试：自动到任务卡位，只调整曝光与增益。"""

import argparse
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[3]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import cv2

import config
from modules.camera import Camera
from modules.task2_perception import load_task2_tuning, save_task2_tuning_patch
from tools.task2._support import move_to_scene_view


WINDOW = "Task2 card exposure/gain"


def _nothing(_value):
    pass


def _window_open():
    try:
        return cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) >= 1
    except cv2.error:
        return False


def _capture(camera, exposure, gain):
    path = Path(config.TASK2_OUTPUT_DIR) / "tuning_card.jpg"
    return camera.capture(
        output_name=path, exposure_time=float(exposure), gain=float(gain))


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="自动到任务卡拍照位，只调整并保存任务卡曝光/增益")
    parser.add_argument("--image", help="使用本地任务卡图片；仅预览，仍可保存滑条值")
    parser.add_argument("--no-move", action="store_true",
                        help="相机模式下保持机器人当前位置")
    args = parser.parse_args(argv)
    if args.image and args.no_move:
        parser.error("图片模式不需要--no-move")
    return args


def main():
    args = parse_args()
    use_camera = not args.image
    if use_camera and not args.no_move:
        move_to_scene_view("card")

    load_task2_tuning()
    configured_exposure = config.TASK2_CARD_EXPOSURE_TIME
    configured_gain = config.TASK2_CARD_GAIN
    initial_exposure = (config.MVS_EXPOSURE_TIME if configured_exposure is None
                        else configured_exposure)
    initial_gain = config.MVS_GAIN if configured_gain is None else configured_gain
    initial_exposure = max(0, int(initial_exposure))
    initial_gain_x10 = max(0, int(float(initial_gain) * 10))

    camera = Camera() if use_camera else None
    image_path = (_capture(camera, initial_exposure, initial_gain_x10 / 10.0)
                  if use_camera else Path(args.image))
    image = cv2.imread(str(image_path))
    if image is None:
        raise RuntimeError("无法读取任务卡图片：" + str(image_path))

    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW, 1100, 900)
    cv2.createTrackbar("Exposure us", WINDOW, initial_exposure, 100000, _nothing)
    cv2.createTrackbar("Gain x10", WINDOW, initial_gain_x10, 1000, _nothing)
    last_settings = (initial_exposure, initial_gain_x10)
    changed_at = float("inf")

    while _window_open():
        settings = (cv2.getTrackbarPos("Exposure us", WINDOW),
                    cv2.getTrackbarPos("Gain x10", WINDOW))
        if settings != last_settings:
            last_settings = settings
            changed_at = time.monotonic()
        if camera is not None and time.monotonic() - changed_at >= 0.35:
            image_path = _capture(camera, settings[0], settings[1] / 10.0)
            refreshed = cv2.imread(str(image_path))
            if refreshed is not None:
                image = refreshed
            changed_at = float("inf")

        display = image.copy()
        cv2.rectangle(display, (0, 0), (display.shape[1], 70), (0, 0, 0), -1)
        cv2.putText(display, "CARD exposure/gain  s=save c=capture q=quit",
                    (20, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.85,
                    (255, 255, 255), 2)
        cv2.imshow(WINDOW, cv2.resize(display, (960, 720)))
        key = cv2.waitKey(50) & 0xFF
        if key in (ord("q"), 27):
            break
        if key == ord("c") and camera is not None:
            image_path = _capture(camera, settings[0], settings[1] / 10.0)
            refreshed = cv2.imread(str(image_path))
            if refreshed is not None:
                image = refreshed
            changed_at = float("inf")
        elif key == ord("s"):
            saved = save_task2_tuning_patch({"capture": {
                "card_exposure_time": settings[0],
                "card_gain": settings[1] / 10.0,
            }})
            print("已保存任务卡曝光/增益：" + str(saved.resolve()))
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
