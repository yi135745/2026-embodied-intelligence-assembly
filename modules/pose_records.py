"""读取AUBO位姿记录JSON，并按受控名称覆盖config中的位姿。"""

import json
import math
from pathlib import Path

import config


REQUIRED_POSE_NAMES = (
    "TASK2_CARD_VIEW_POSE",
    "TASK2_BLOCK_VIEW_POSE",
    "TASK2_TRAY_VIEW_POSE",
    "ROBOT_TARGET",
)


def load_aubo_pose_records(path=None):
    """返回{name: [x,y,z,rx,ry,rz]}；XYZ毫米、姿态弧度。"""
    pose_path = Path(path or config.AUBO_POSE_JSON_FILE)
    if not pose_path.exists():
        raise RuntimeError("找不到AUBO唯一位姿文件：%s；请先运行aubo_pose_recorder.py。" % pose_path)
    data = json.loads(pose_path.read_text(encoding="utf-8"))
    if data.get("position_unit") != "mm" or data.get("orientation_unit") != "rad":
        raise RuntimeError("AUBO位姿JSON单位必须为XYZ=mm、姿态=rad。")
    result = {}
    for item in data.get("poses", []):
        name, pose = item.get("name"), item.get("pose_mm_rad")
        if not isinstance(name, str) or not isinstance(pose, list) or len(pose) != 6:
            raise RuntimeError("AUBO位姿JSON中存在无效记录：%r" % item)
        values = [float(value) for value in pose]
        if not all(math.isfinite(value) for value in values):
            raise RuntimeError("AUBO位姿%s包含NaN或无穷大。" % name)
        result[name] = values
    return result


def apply_aubo_pose_records(path=None):
    """强制从唯一JSON加载四个位姿，并覆盖运行时config。"""
    records = load_aubo_pose_records(path)
    missing = [name for name in REQUIRED_POSE_NAMES if name not in records]
    extra = [name for name in records if name not in REQUIRED_POSE_NAMES]
    if missing or extra:
        raise RuntimeError("AUBO位姿JSON名称不完整，缺少=%s，多出=%s。" % (missing, extra))
    applied = {}
    for name in REQUIRED_POSE_NAMES:
        pose = records[name]
        setattr(config, name, pose)
        applied[name] = pose
        print("已从AUBO位姿JSON加载%s：%s" % (name, pose))
    return applied
