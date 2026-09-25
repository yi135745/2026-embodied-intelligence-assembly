"""离线列出或恢复任务二标定 XML 的历史版本。"""

import argparse
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[3]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import config
from modules.task2_calibration import (
    activate_calibration,
    assess_calibration,
    list_calibration_versions,
    print_activation_summary,
    validate_calibration,
)


def main():
    parser = argparse.ArgumentParser(description="列出或恢复任务二标定 XML；不连接机器人")
    parser.add_argument("--activate", metavar="XML", help="启用候选或历史 XML，先备份当前正式版")
    args = parser.parse_args()
    active = config.TASK2_CALIBRATION_FILE
    binding = config.TASK2_CALIBRATION_BINDING_FILE
    if args.activate:
        candidate = Path(args.activate).resolve()
        validate_calibration(candidate)
        decision = print_activation_summary(
            "方块/托盘共用矩阵恢复", active, candidate, config.TASK2_OUTPUT_DIR,
            consequence="唯一XY偏移指纹立即失效，启用后需运行闭环偏移和纯平移验收。")
        if decision["state"] != "PASS":
            print("候选没有通过可追溯质量门禁，禁止启用。")
            return
        if input("确认上述信息后，输入 yes 覆盖正式共用标定 XML：").strip().lower() != "yes":
            print("已取消，正式标定保持不变。")
            return
        candidate_quality = assess_calibration(candidate, config.TASK2_OUTPUT_DIR)
        quality_report = ({
            "world_rms_mm": candidate_quality["world_rms_mm"],
            "pixel_rms": candidate_quality["pixel_rms"],
            "report_path": candidate_quality["source_report"],
        } if candidate_quality["quality_known"] else None)
        if quality_report and candidate_quality["source_report"]:
            try:
                full_report = json.loads(
                    Path(candidate_quality["source_report"]).read_text(encoding="utf-8"))
                if isinstance(full_report.get("samples"), list):
                    quality_report = full_report
            except (OSError, ValueError, json.JSONDecodeError):
                pass
        backup = activate_calibration(
            candidate, active=active, binding_file=binding,
            quality_report=quality_report,
            persistent_report_file=(config.TASK2_CALIBRATION_REPORT_FILE
                                    if quality_report and quality_report.get("samples") else None))
        print("已启用：" + active)
        print("旧版备份：" + (str(backup) if backup else "无（原正式文件不存在）"))
        return
    print("当前正式方块/托盘共用标定：%s" % active)
    versions = list_calibration_versions()
    if not versions:
        print("暂无历史备份。")
    for path in versions:
        print(path)


if __name__ == "__main__":
    main()
