"""只读审计任务二共用矩阵、版本指纹与XY偏移链路。"""

import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[3]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import config
from runtime.site_data import apply_aubo_pose_records
from modules.task2_calibration import assess_calibration, calibration_sha256
from runtime.task2_state import require_shared_calibration_view_compatibility


def _print_matrix(label, path):
    result = assess_calibration(path, config.TASK2_OUTPUT_DIR)
    state = "FAIL" if result["issues"] else ("PASS" if result["quality_known"] else "UNKNOWN")
    print("[%s] %s矩阵：%s" % (state, label, Path(path).resolve()))
    print("  指纹=%s，点数=%s" % (result["sha256"], result["point_count"]))
    if result["quality_known"]:
        print("  world RMS=%.3f mm，pixel RMS=%.3f px" %
              (float(result["world_rms_mm"]), float(result["pixel_rms"])))
        print("  来源=" + result["source_report"])
    if result["issues"]:
        print("  原因=" + "；".join(result["issues"]))
    return result


def main():
    apply_aubo_pose_records()
    matrix_path = Path(config.TASK2_CALIBRATION_FILE)
    matrix = _print_matrix("方块/托盘共用", matrix_path)
    compatibility = require_shared_calibration_view_compatibility()
    print("[PASS] 共用矩阵位姿：Z差=%.3f mm，RZ差=%.6f rad" %
          (compatibility["z_diff_mm"], compatibility["rz_diff_rad"]))
    offset_path = Path(config.TASK2_OFFSET_FILE)
    if not offset_path.exists():
        print("[PENDING] 尚无正式XY偏移；半自动闭环可从本轮数据首次创建：" +
              str(offset_path.resolve()))
        offset_match = False
    else:
        offsets = json.loads(offset_path.read_text(encoding="utf-8"))
        if offsets.get("physical_alignment") is not None:
            offset_match = True
            print("[PASS] 原始物理锚点可与当前公共矩阵重新组合")
        else:
            offset_match = (offsets.get("calibration_xml_sha256") ==
                            calibration_sha256(matrix_path))
            print("[%s] 旧版矩阵偏移指纹" % ("PASS" if offset_match else "STALE"))
    if matrix["issues"] or not matrix["quality_known"]:
        print("下一步：重做或恢复共用矩阵，禁止运行偏移闭环。")
    elif not offset_match:
        print("下一步：运行tools/task2/workflow/task2_closed_loop_offset_calibrate.py；"
              "旧偏移不参与计算，本轮将重新建立锚点并做真机验证。")
    else:
        print("下一步：矩阵和偏移版本一致，仍需通过纯平移/旋转/托盘复拍验收。")
    return 1 if matrix["issues"] or not matrix["quality_known"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
