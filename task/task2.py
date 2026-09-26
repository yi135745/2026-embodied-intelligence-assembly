"""任务二：任务卡、九色方块、六色托盘各拍一次，按序落盘与旋转叠放。"""

import json
import time
import traceback
from datetime import datetime
from pathlib import Path
from copy import deepcopy

import os
import sys

# 保证无论从项目根目录运行（python main.py）还是直接运行本文件（python task/task1.py），
# 都能 import 到根目录的 config 与 modules。
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import config
from modules.task2_perception import (
    ColorObjectDetector,
    load_task2_tuning,
    save_debug_image,
    validate_colors,
)
from runtime.site_data import apply_aubo_pose_records
from runtime.task2_state import get_task2_block_view_pose, load_task2_runtime_state
from modules.task2_planning import (
    apply_motion_compensation,
    build_plan,
    validate_actions,
)


def _capture(camera, output_dir, name, debug_image, exposure_time=None, gain=None):
    try:
        return camera.capture(output_name=output_dir / name, debug_image=debug_image,
                              exposure_time=exposure_time, gain=gain)
    except TypeError:  # 兼容离线测试中的简化相机对象
        return camera.capture(output_name=output_dir / name, debug_image=debug_image)


def _move_to_view(robot, pose, label):
    if pose is None:
        print(label + "未填写，跳过拍照位移动。")
        return
    if robot is None or not getattr(robot, "available", False):
        raise RuntimeError(label + "已配置，但机器人当前不可用。")
    if not robot.move_to_safe(pose):
        raise RuntimeError("安全移动到" + label + "失败。")
    time.sleep(max(0.0, float(config.TASK2_SETTLE_SECONDS)))


def _build_plan(steps, blocks, trays, motion_compensation=None):
    nominal = build_plan(steps, blocks, trays)
    return apply_motion_compensation(nominal, motion_compensation)


def task2_run(voice, camera, robot, interpreter):
    """先预检整份计划，成功执行一步才提交已放置状态；无硬件时可用三张调试图。"""
    output_dir = Path(config.TASK2_OUTPUT_DIR)
    output_dir.mkdir(parents=True, exist_ok=True)
    record = None
    log_path = None
    try:
        apply_aubo_pose_records()
        load_task2_tuning()
        runtime_state = load_task2_runtime_state()
        motion_compensation = runtime_state.motion_compensation
        detector = ColorObjectDetector(xy_calibration=runtime_state.xy_calibration)
        execute = bool(config.TASK2_EXECUTE_ROBOT)
        if execute and (robot is None or not getattr(robot, "available", False)):
            raise RuntimeError("机器人未连接；离线计划请明确设置TASK2_EXECUTE_ROBOT=False。")
        voice.speak("开始识别任务卡二")
        if execute:
            _move_to_view(robot, config.TASK2_CARD_VIEW_POSE, "任务卡拍照位")
        card = _capture(camera, output_dir, config.TASK2_CARD_CAPTURE_NAME, config.TASK2_CARD_DEBUG_IMAGE,
                        config.TASK2_CARD_EXPOSURE_TIME, config.TASK2_CARD_GAIN)
        try:
            steps = validate_actions(interpreter.parse_task2_card(card, output_dir=output_dir))
        except Exception:
            voice.speak("任务卡识别失败，请确认任务卡位置")
            raise
        voice.speak("；".join("第%d步，%s方块放到%s%s上" %
                    (x["step"], x["source_color"], x["target_color"],
                     "托盘" if x["target_type"] == "tray" else "方块") for x in steps))
        if execute:
            block_view_pose = (get_task2_block_view_pose()
                               if runtime_state.uses_physical_alignment
                               else config.TASK2_BLOCK_VIEW_POSE)
            _move_to_view(robot, block_view_pose, "方块拍照位")
        block_image = _capture(camera, output_dir, config.TASK2_BLOCK_CAPTURE_NAME, config.TASK2_BLOCK_DEBUG_IMAGE,
                               config.TASK2_BLOCK_EXPOSURE_TIME, config.TASK2_BLOCK_GAIN)
        blocks, block_debug = detector.detect(block_image, "方块", output_dir, "blocks",
                                               strict_board=True)
        save_debug_image(output_dir / "blocks_detected.jpg", block_debug)
        if execute:
            _move_to_view(robot, config.TASK2_TRAY_VIEW_POSE, "托盘拍照位")
        tray_image = _capture(camera, output_dir, config.TASK2_TRAY_CAPTURE_NAME, config.TASK2_TRAY_DEBUG_IMAGE,
                              config.TASK2_TRAY_EXPOSURE_TIME, config.TASK2_TRAY_GAIN)
        trays, tray_debug = detector.detect(tray_image, "托盘", output_dir, "trays")
        save_debug_image(output_dir / "trays_detected.jpg", tray_debug)
        if config.TASK2_REQUIRE_ALL_COLORS:
            validate_colors(blocks, "方块")
            validate_colors(trays, "托盘")
        record = {"timestamp": datetime.now().isoformat(timespec="seconds"), "steps": steps,
                  "plan": _build_plan(steps, blocks, trays, motion_compensation),
                  "status": "planned",
                  "placed_block_map": {}, "rotation_enabled": config.TASK2_ROTATION_ENABLED,
                  "block_height_mm": float(config.TASK2_BLOCK_HEIGHT_MM)}
        log_path = output_dir / ("task2_%s.json" % datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
        log_path.write_text(json.dumps(record, ensure_ascii=False, indent=2), encoding="utf-8")
        print("任务二计划已生成：" + str(log_path))
        if execute:
            if config.TASK2_REQUIRE_OFFSET_FILE and not Path(config.TASK2_OFFSET_FILE).exists():
                raise RuntimeError(
                    "缺少正式偏差文件%s，请先运行"
                    "tools/task2/workflow/task2_closed_loop_offset_calibrate.py。" %
                    config.TASK2_OFFSET_FILE)
            for item in record["plan"]:
                if item["target_type"] == "block" and item["target_color"] not in record["placed_block_map"]:
                    raise RuntimeError("目标方块尚未成功放置：" + item["target_color"])
                pick_pose = item["pick"].get(
                    "command_robot_pose", item["pick"]["robot_pose"])
                place_pose = item["place"].get(
                    "command_robot_pose", item["place"]["robot_pose"])
                if pick_pose is None or place_pose is None:
                    raise RuntimeError("第%d步缺少机器人坐标，请填写标定原点和抓放Z高度。" % item["step"])
                item["robot_status"] = "running"
                record["status"] = "running"
                log_path.write_text(json.dumps(record, ensure_ascii=False, indent=2), encoding="utf-8")
                if not robot.pick_and_place(pick_pose, place_pose):
                    item["robot_status"] = "failed"
                    log_path.write_text(json.dumps(record, ensure_ascii=False, indent=2), encoding="utf-8")
                    raise RuntimeError("第%d步机器人抓放失败。" % item["step"])
                item["robot_status"] = "completed"
                record["placed_block_map"][item["source_color"]] = deepcopy(item["placed_state"])
                log_path.write_text(json.dumps(record, ensure_ascii=False, indent=2), encoding="utf-8")
            record["status"] = "completed"
            log_path.write_text(json.dumps(record, ensure_ascii=False, indent=2), encoding="utf-8")
            voice.speak("任务已完成")
        else:
            voice.speak("任务二装配计划生成完成，当前为离线模式，未执行运动")
        return record
    except (Exception, SystemExit) as exc:
        if record is not None and log_path is not None:
            record["status"] = "failed"
            record["error"] = str(exc)
            for item in record["plan"]:
                if item["robot_status"] == "running":
                    item["robot_status"] = "failed"
            log_path.write_text(json.dumps(record, ensure_ascii=False, indent=2), encoding="utf-8")
        error_text = "%s\n\n%s" % (str(exc), traceback.format_exc())
        error_path = output_dir / ("task2_error_%s.log" % datetime.now().strftime("%Y%m%d_%H%M%S"))
        error_path.write_text(error_text, encoding="utf-8")
        print("任务二失败：" + str(exc))
        print("完整错误日志：" + str(error_path))
        voice.speak("任务二失败，原因是" + str(exc))
        return None


if __name__ == "__main__":
    from modules.interpreter import Interpreter
    from modules.robot import Robot
    from modules.camera import Camera
    from modules.voice import Voice

    standalone_robot = Robot()
    try:
        task2_run(Voice(), Camera(), standalone_robot, Interpreter())
    finally:
        standalone_robot.disconnect()
