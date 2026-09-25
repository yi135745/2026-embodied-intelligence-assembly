"""方块/托盘拍照高度对照；只读AUBO TCP、抓图，不发送任何运动命令。"""

import argparse
import io
import json
import math
import sys
from contextlib import redirect_stdout
from datetime import datetime
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[3]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import cv2
import numpy as np

import config
from runtime.site_data import apply_aubo_pose_records
from modules.robot import Robot
from modules.task2_perception import ColorObjectDetector, CoordinateTransformer, load_task2_tuning
from modules.camera import Camera


def pose_deviation(reference, actual, delta_z):
    """相对记录拍照位的XY/Z/姿态偏差；姿态按2π环绕。"""
    angle_errors = [abs((actual[i] - reference[i] + math.pi) % (2 * math.pi) - math.pi)
                    for i in range(3, 6)]
    return {
        "xy_mm": math.hypot(actual[0] - reference[0], actual[1] - reference[1]),
        "z_mm": actual[2] - (reference[2] + delta_z),
        "orientation_rad": max(angle_errors),
    }


def _light_stats(image):
    """固定画面区域的粗略亮度指标；不能代替原图和标注图人工检查。"""
    value = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)[:, :, 2]
    height, width = value.shape

    def patch(x0, x1, y0, y1):
        part = value[int(height * y0):int(height * y1),
                     int(width * x0):int(width * x1)]
        return float(np.median(part))

    return {
        "center_v_median": patch(.4, .6, .4, .6),
        "corner_v_medians": [patch(.1, .25, .1, .25), patch(.75, .9, .1, .25),
                             patch(.1, .25, .75, .9), patch(.75, .9, .75, .9)],
        "full_image_v_250_fraction": float(np.mean(value >= 250)),
    }


def analyze_image(image_path, kind, detector, output_dir, prefix):
    image = cv2.imread(str(image_path))
    if image is None:
        raise RuntimeError("无法读取高度测试图片：" + str(image_path))
    expected = set(config.TASK2_BLOCK_COLORS if kind == "方块" else config.TASK2_TRAY_COLORS)
    if kind == "方块":
        expected -= set(config.TASK2_DISABLED_BLOCK_COLORS)
    result = {
        "image_size_px": [int(image.shape[1]), int(image.shape[0])],
        "lighting_rough_only": _light_stats(image),
        "expected_colors": sorted(expected),
        "detections": [],
        "detection_error": None,
    }
    try:
        targets, annotated = detector.detect(
            image_path, kind, output_dir, prefix, include_robot_pose=False
        )
        result["detections"] = [
            {"color": target.color, "pixel_center": list(target.pixel_center),
             "area_px2": target.area}
            for target in targets
        ]
    except (ValueError, RuntimeError) as exc:
        result["detection_error"] = str(exc)
        annotated = image.copy()
    found = {item["color"] for item in result["detections"]}
    result["missing_colors"] = sorted(expected - found)
    result["extra_colors"] = sorted(found - expected)
    result["complete"] = (result["detection_error"] is None and found == expected)
    annotated_path = output_dir / (prefix + "_detected.jpg")
    if not cv2.imwrite(str(annotated_path), annotated):
        raise RuntimeError("无法保存标注图片：" + str(annotated_path))
    result["annotated_image"] = str(annotated_path.resolve())
    return result


def _write_report(output_dir, report):
    (output_dir / "report.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    baseline = report.get("baseline_actual_pose_mm_rad")
    lines = ["拍照高度对照（不移动机械臂、不改正式标定）",
             "区域：%s；记录拍照位Z：%.3f mm；本次实际基准Z：%s" % (
                 report["kind"], report["reference_pose_mm_rad"][2],
                 "%.3f mm" % baseline[2] if baseline else "尚未拍摄"),
             "亮度指标只是固定画面区域的粗略比较，必须查看原图和标注图。", ""]
    for sample in report["samples"]:
        if "analysis" not in sample:
            continue
        analysis = sample["analysis"]
        light = analysis["lighting_rough_only"]
        lines.append("+%.0f mm | 实际Z %.3f | 识别 %d/%d | 缺少 %s | 中心V %.0f | 四角V %s | V≥250 %.1f%%" % (
            sample["requested_delta_z_mm"], sample["actual_pose_mm_rad"][2],
            len(analysis["detections"]), len(analysis["expected_colors"]),
            analysis["missing_colors"], light["center_v_median"],
            [round(value) for value in light["corner_v_medians"]],
            100 * light["full_image_v_250_fraction"]))
        if analysis["detection_error"]:
            lines.append("  检测错误：" + analysis["detection_error"])
        lines.append("  原图：" + sample["raw_image"])
        lines.append("  标注图：" + analysis["annotated_image"])
    summary = "\n".join(lines) + "\n"
    (output_dir / "summary.txt").write_text(summary, encoding="utf-8")
    print(summary)


def main():
    parser = argparse.ArgumentParser(description="手动调整拍照高度，自动留存方块/托盘对照图片和日志；不移动机械臂")
    parser.add_argument("--scene", choices=("block", "tray"), required=True)
    parser.add_argument("--deltas", type=float, nargs="+", default=[0.0, 20.0, 40.0],
                        help="相对已记录拍照位的Z增量，默认0/20/40mm")
    parser.add_argument("--exposure-us", type=float, help="本次三档共用的固定曝光；不写入config")
    parser.add_argument("--gain", type=float, help="本次三档共用的固定增益；不写入config")
    args = parser.parse_args()
    if args.deltas[0] != 0 or any(not math.isfinite(x) or x < 0 for x in args.deltas):
        parser.error("高度列表必须从0开始，且各项为非负有限毫米值。")
    if config.DEBUG_IMAGE is not None:
        parser.error("DEBUG_IMAGE已配置静态图；请先清空，避免误以为调用了相机。")
    if args.exposure_us is not None and (not math.isfinite(args.exposure_us) or args.exposure_us <= 0):
        parser.error("--exposure-us必须是正数。")
    if args.gain is not None and (not math.isfinite(args.gain) or args.gain < 0):
        parser.error("--gain必须是非负数。")

    apply_aubo_pose_records()
    load_task2_tuning()
    prefix = args.scene.upper()
    kind = "方块" if args.scene == "block" else "托盘"
    reference = list(getattr(config, "TASK2_%s_VIEW_POSE" % prefix))
    exposure = getattr(config, "TASK2_%s_EXPOSURE_TIME" % prefix)
    gain = getattr(config, "TASK2_%s_GAIN" % prefix)
    if args.exposure_us is not None:
        exposure = args.exposure_us
    if args.gain is not None:
        gain = args.gain
    requested_exposure = config.MVS_EXPOSURE_TIME if exposure is None else exposure
    requested_gain = config.MVS_GAIN if gain is None else gain
    output_dir = (Path(config.TASK2_OUTPUT_DIR) / "height_probe" /
                  (datetime.now().strftime("%Y%m%d_%H%M%S_%f") + "_" + args.scene))
    output_dir.mkdir(parents=True, exist_ok=False)
    # 只比较检测轮廓/像素中心；旧矩阵在新高度失效，故不计算机器人XY/方向。
    transformer = CoordinateTransformer()
    transformer.matrix = np.eye(3)
    detector = ColorObjectDetector(transformer)
    report = {
        "scene": args.scene, "kind": kind,
        "reference_pose_mm_rad": reference,
        "baseline_actual_pose_mm_rad": None,
        "requested_exposure_us": requested_exposure,
        "requested_gain": requested_gain,
        "camera_values_read_back": False,
        "robot_motion_commands_sent": False,
        "formal_calibration_modified": False,
        "samples": [],
    }
    _write_report(output_dir, report)

    robot = Robot()
    if not robot.available:
        raise RuntimeError("无法读取机器人TCP；高度对照要求连接AUBO，但不会发送运动命令。")
    vision = Camera()
    try:
        print("请保持物块/托盘摆放、曝光、增益不变；只用示教器改变拍照位Z。")
        if requested_exposure <= 0 or requested_gain < 0:
            print("警告：曝光或增益未显式设置；相机可能使用原有/自动值，亮度变化不能单独归因于高度。")
        print("曝光/增益请求值：%s us / %s；当前相机接口不读回实际值。" %
              (requested_exposure, requested_gain))
        print("旧记录拍照位仅供参考：" + str(reference))
        print("首张照片以当前实际TCP为基准，不要求精确回到旧记录位置。")
        baseline = None
        for index, delta in enumerate(args.deltas, start=1):
            while True:
                target_z = (baseline[2] + delta) if baseline is not None else reference[2]
                answer = input("[%d/%d] %s；回车读取，q结束：" % (
                    index, len(args.deltas),
                    ("确认当前原高度，后续以此TCP为基准" if baseline is None
                     else "示教器仅调整Z至%.3f mm（+%.0f），保持首张XY/姿态不变" %
                     (target_z, delta)))).strip().lower()
                if answer == "q":
                    return
                actual = robot.get_current_pose()
                if baseline is None:
                    baseline = actual
                    report["baseline_actual_pose_mm_rad"] = actual
                    report["baseline_vs_recorded_pose"] = pose_deviation(reference, actual, 0)
                    deviation = pose_deviation(actual, actual, 0)
                    print("本次实际基准位姿：" + str(actual))
                    print("与旧记录位姿的差异已存档，不作为本轮拍照拦截条件。")
                    break
                deviation = pose_deviation(baseline, actual, delta)
                print("偏差：XY=%.3f mm；Z=%+.3f mm；姿态最大=%.4f rad" % (
                    deviation["xy_mm"], deviation["z_mm"], deviation["orientation_rad"]))
                if (deviation["xy_mm"] <= 2.0 and abs(deviation["z_mm"]) <= 2.0
                        and deviation["orientation_rad"] <= 0.01):
                    break
                print("位置偏差超出对照容差（XY/Z≤2mm，姿态≤0.01rad），不拍照。请调整后重试。")
            name = "%02d_zplus_%g" % (index, delta)
            raw_path = output_dir / (name + "_raw.jpg")
            capture_log = io.StringIO()
            try:
                with redirect_stdout(capture_log):
                    vision.capture(output_name=raw_path, exposure_time=exposure, gain=gain)
            finally:
                captured_text = capture_log.getvalue()
                print(captured_text, end="")
                (output_dir / (name + "_capture_log.txt")).write_text(captured_text, encoding="utf-8")
            sample = {
                "requested_delta_z_mm": delta,
                "actual_pose_mm_rad": actual,
                "pose_deviation": deviation,
                "raw_image": str(raw_path.resolve()),
            }
            report["samples"].append(sample)
            _write_report(output_dir, report)
            sample["analysis"] = analyze_image(raw_path, kind, detector, output_dir, name)
            _write_report(output_dir, report)
    finally:
        robot.disconnect()
        print("对照文件：" + str(output_dir.resolve()))


if __name__ == "__main__":
    main()
