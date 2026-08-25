"""任务二基础流程：任务卡解析、六色目标识别、装配计划生成。"""

import json
import time
import traceback
from datetime import datetime
from pathlib import Path

import cv2

import os
import sys

# 保证无论从项目根目录运行（python main.py）还是直接运行本文件（python task/task1.py），
# 都能 import 到根目录的 config 与 modules。
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import config
from modules.task2_vision import (ColorObjectDetector, VisionTarget, load_task2_offsets,
                                  load_task2_tuning, validate_six_colors)


def _capture(vision, output_dir, name, debug_image, exposure_time=None, gain=None):
    try:
        return vision.capture(output_name=output_dir / name, debug_image=debug_image,
                              exposure_time=exposure_time, gain=gain)
    except TypeError:  # 兼容离线测试中的简化相机对象
        return vision.capture(output_name=output_dir / name, debug_image=debug_image)


def _move_to_view(robot, pose, label):
    if pose is None:
        print(label + "未填写，跳过拍照位移动。")
        return
    if robot is None or not getattr(robot, "available", False):
        raise RuntimeError(label + "已配置，但机器人当前不可用。")
    if not robot.move_to(pose):
        raise RuntimeError("移动到" + label + "失败。")
    time.sleep(max(0.0, float(config.TASK2_SETTLE_SECONDS)))


def _build_plan(steps, blocks, trays):
    block_map = {item.color: item for item in blocks}
    tray_map = {item.color: item for item in trays}
    return [{"step": step["step"], "block_color": step["block_color"], "tray_color": step["tray_color"], "pick": block_map[step["block_color"]].to_dict(), "place": tray_map[step["tray_color"]].to_dict(), "robot_status": "pending_calibration_and_aubo"} for step in steps]


def _load_verified_trays():
    path = Path(config.TASK2_VERIFIED_TRAY_FILE)
    if not config.TASK2_USE_VERIFIED_TRAYS:
        return None
    if not path.exists():
        raise RuntimeError(
            "已配置TASK2_USE_VERIFIED_TRAYS=True，但找不到%s；"
            "请先运行task2_tray_verify.py人工验收，或在config.py改为False使用现场识别。" % path
        )
    data = json.loads(path.read_text(encoding="utf-8"))
    trays = [VisionTarget(**item) for item in data.get("targets", [])]
    validate_six_colors(trays, "人工验收托盘")
    if any(item.robot_pose is None for item in trays):
        raise RuntimeError("人工验收托盘文件中存在空机器人坐标，请重新验收。")
    print("已使用人工验收托盘：%s（%s）" % (path, data.get("verified_at", "时间未知")))
    print("警告：必须确保托盘自验收后没有移动。")
    return trays


def task2_run(voice, vision, robot, llm):
    """识别任务卡和目标，并在坐标完整时执行六步抓放。"""
    output_dir = Path(config.TASK2_OUTPUT_DIR)
    output_dir.mkdir(parents=True, exist_ok=True)
    load_task2_tuning()
    load_task2_offsets()
    detector = ColorObjectDetector()
    try:
        voice.speak("开始识别任务卡二")
        _move_to_view(robot, config.TASK2_CARD_VIEW_POSE, "任务卡拍照位")
        card = _capture(vision, output_dir, config.TASK2_CARD_CAPTURE_NAME, config.TASK2_CARD_DEBUG_IMAGE,
                        config.TASK2_CARD_EXPOSURE_TIME, config.TASK2_CARD_GAIN)
        try:
            steps = llm.parse_task2_card(card, output_dir=output_dir)
        except TypeError:
            steps = llm.parse_task2_card(card)
        voice.speak("；".join("第%d步，%s方块到%s托盘" % (x["step"], x["block_color"], x["tray_color"]) for x in steps))
        _move_to_view(robot, config.TASK2_BLOCK_VIEW_POSE, "方块拍照位")
        block_image = _capture(vision, output_dir, config.TASK2_BLOCK_CAPTURE_NAME, config.TASK2_BLOCK_DEBUG_IMAGE,
                               config.TASK2_BLOCK_EXPOSURE_TIME, config.TASK2_BLOCK_GAIN)
        blocks, block_debug = detector.detect(block_image, "方块", output_dir, "blocks")
        cv2.imwrite(str(output_dir / "blocks_detected.jpg"), block_debug)
        trays = _load_verified_trays()
        if trays is None:
            _move_to_view(robot, config.TASK2_TRAY_VIEW_POSE, "托盘拍照位")
            tray_image = _capture(vision, output_dir, config.TASK2_TRAY_CAPTURE_NAME, config.TASK2_TRAY_DEBUG_IMAGE,
                                  config.TASK2_TRAY_EXPOSURE_TIME, config.TASK2_TRAY_GAIN)
            trays, tray_debug = detector.detect(tray_image, "托盘", output_dir, "trays")
            cv2.imwrite(str(output_dir / "trays_detected.jpg"), tray_debug)
        if config.TASK2_REQUIRE_ALL_COLORS:
            validate_six_colors(blocks, "方块")
            validate_six_colors(trays, "托盘")
        record = {"timestamp": datetime.now().isoformat(timespec="seconds"), "steps": steps, "plan": _build_plan(steps, blocks, trays)}
        log_path = output_dir / ("task2_%s.json" % datetime.now().strftime("%Y%m%d_%H%M%S"))
        log_path.write_text(json.dumps(record, ensure_ascii=False, indent=2), encoding="utf-8")
        print("任务二计划已生成：" + str(log_path))
        can_execute = config.TASK2_EXECUTE_ROBOT and robot is not None and getattr(robot, "available", False)
        if can_execute:
            if config.TASK2_REQUIRE_OFFSET_FILE and not Path(config.TASK2_OFFSET_FILE).exists():
                raise RuntimeError("缺少新的偏差标定文件%s，请先运行task2_offset_calibrate.py。" % config.TASK2_OFFSET_FILE)
            for item in record["plan"]:
                pick_pose, place_pose = item["pick"]["robot_pose"], item["place"]["robot_pose"]
                if pick_pose is None or place_pose is None:
                    raise RuntimeError("第%d步缺少机器人坐标，请填写标定原点和抓放Z高度。" % item["step"])
                item["robot_status"] = "running"
                if not robot.pick_and_place(pick_pose, place_pose):
                    item["robot_status"] = "failed"
                    log_path.write_text(json.dumps(record, ensure_ascii=False, indent=2), encoding="utf-8")
                    raise RuntimeError("第%d步机器人抓放失败。" % item["step"])
                item["robot_status"] = "completed"
            log_path.write_text(json.dumps(record, ensure_ascii=False, indent=2), encoding="utf-8")
            voice.speak("任务已完成")
        else:
            voice.speak("任务二视觉识别与装配计划生成完成，机器人未连接，未执行运动")
        return record
    except (Exception, SystemExit) as exc:
        error_text = "%s\n\n%s" % (str(exc), traceback.format_exc())
        error_path = output_dir / ("task2_error_%s.log" % datetime.now().strftime("%Y%m%d_%H%M%S"))
        error_path.write_text(error_text, encoding="utf-8")
        print("任务二失败：" + str(exc))
        print("完整错误日志：" + str(error_path))
        voice.speak("任务二失败，原因是" + str(exc))
        return None


if __name__ == "__main__":
    from modules.llm import LLM
    from modules.robot import Robot
    from modules.vision import Vision
    from modules.voice import Voice

    standalone_robot = Robot()
    try:
        task2_run(Voice(), Vision(), standalone_robot, LLM())
    finally:
        standalone_robot.disconnect()
