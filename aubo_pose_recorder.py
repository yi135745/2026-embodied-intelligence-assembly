"""通过AUBO SDK连续记录四个基座TCP位姿，不执行任何自动运动。"""

import argparse
import json
from datetime import datetime
from pathlib import Path

import config
from modules.robot import Robot


DEFAULT_NAMES = (
    "TASK2_CARD_VIEW_POSE",
    "TASK2_BLOCK_VIEW_POSE",
    "TASK2_TRAY_VIEW_POSE",
    "ROBOT_TARGET",
)


def _format_pose(pose):
    return "[" + ", ".join("%.6f" % value for value in pose) + "]"


def main():
    parser = argparse.ArgumentParser(
        description="手动调整机械臂后按回车，连续读取并保存四个基座TCP位姿"
    )
    args = parser.parse_args()

    robot = Robot()
    if not robot.available:
        raise RuntimeError("AUBO未连接，无法读取基座TCP位姿。")

    records = []
    try:
        print("\n程序不会自动移动机械臂。")
        print("请用示教器调整位置；每次调整完成后回到本窗口按回车读取。")
        print("坐标单位：XYZ=mm，RX/RY/RZ=rad。")
        for index, name in enumerate(DEFAULT_NAMES, start=1):
            while True:
                input("\n[%d/4] 请将机械臂调整到 %s，完成后按回车读取..." % (index, name))
                pose = robot.get_current_pose()
                print("%s = %s" % (name, _format_pose(pose)))
                answer = input("直接回车确认；输入r重新读取；输入q取消全部记录：").strip().lower()
                if answer == "q":
                    print("已取消，本次不写入文件。")
                    return
                if answer != "r":
                    records.append({"index": index, "name": name, "pose_mm_rad": pose})
                    break

        timestamp = datetime.now()
        data = {
            "recorded_at": timestamp.isoformat(timespec="seconds"),
            "coordinate_system": "robot_base_tcp",
            "position_unit": "mm",
            "orientation_unit": "rad",
            "poses": records,
        }
        json_path = Path(config.AUBO_POSE_JSON_FILE)
        json_path.parent.mkdir(parents=True, exist_ok=True)
        content = json.dumps(data, ensure_ascii=False, indent=2)
        json_path.write_text(content, encoding="utf-8")
        config_lines = ["%s = %s" % (item["name"], _format_pose(item["pose_mm_rad"]))
                        for item in records]

        # 只保留固定文件data/aubo_poses.json，清理由旧版本记录器生成的副本。
        cleanup_roots = (json_path.parent, Path(config.PROJECT_ROOT) / "output" / "aubo_pose_records")
        for root in cleanup_roots:
            if not root.exists():
                continue
            for pattern in ("aubo_poses*.json", "aubo_poses*.txt"):
                for old_path in root.glob(pattern):
                    if old_path.resolve() != json_path.resolve() and old_path.is_file():
                        old_path.unlink()

        print("\n四个位姿记录完成：")
        for line in config_lines:
            print(line)
        print("唯一位姿JSON已更新：" + str(json_path.resolve()))
    finally:
        robot.disconnect()


if __name__ == "__main__":
    main()
