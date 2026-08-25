"""任务二分区视觉独立测试。

示例：
  python task2_vision_test.py --scene block --camera
  python task2_vision_test.py --scene tray --image output/task2/task2_trays.jpg
  python task2_vision_test.py --scene card --camera

每次运行都会在 output/task2/diagnostics/<时间_区域>/ 保存原图、标注图、
六张颜色mask、检测JSON和summary.txt；即使颜色不完整也会保存。
"""

import argparse
import json
import shutil
from datetime import datetime
from pathlib import Path

import cv2

import config
from modules.task2_vision import ColorObjectDetector, load_task2_offsets, load_task2_tuning
from modules.vision import Vision


def _scene_config(scene):
    prefix = scene.upper()
    return (
        getattr(config, "TASK2_%s_EXPOSURE_TIME" % prefix),
        getattr(config, "TASK2_%s_GAIN" % prefix),
    )


def main():
    parser = argparse.ArgumentParser(description="单独检查任务卡、方块区或托盘区视觉效果")
    parser.add_argument("--scene", choices=("card", "block", "tray"), required=True)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--camera", action="store_true")
    source.add_argument("--image")
    parser.add_argument("--show", action="store_true", help="弹窗显示原图/标注图，按任意键关闭")
    args = parser.parse_args()

    load_task2_tuning()
    load_task2_offsets()
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = Path(config.TASK2_OUTPUT_DIR) / "diagnostics" / (stamp + "_" + args.scene)
    output_dir.mkdir(parents=True, exist_ok=True)
    raw_path = output_dir / (args.scene + "_raw.jpg")

    if args.camera:
        exposure, gain = _scene_config(args.scene)
        image_path = Vision().capture(output_name=raw_path, exposure_time=exposure, gain=gain)
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
        image_path, kind, output_dir, args.scene
    )
    annotated_path = output_dir / (args.scene + "_detected.jpg")
    cv2.imwrite(str(annotated_path), annotated)
    found = {item.color for item in targets}
    expected = set(config.TASK2_HSV_RANGES)
    result = [item.to_dict() for item in targets]
    (output_dir / "detections.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    summary = (
        "区域：%s\n识别到：%s\n缺少：%s\n原图：%s\n标注图：%s\n"
        % (kind, sorted(found), sorted(expected - found), raw_path, annotated_path)
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
