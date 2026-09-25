"""标定库内部的 XML 校验、启用和历史版本保存。"""

import math
import hashlib
import json
import os
import shutil
import tempfile
from datetime import datetime
from pathlib import Path
from xml.etree import ElementTree

import config


def validate_calibration(path):
    """只接受点数、矩阵和元数据一致的正式标定结果。"""
    path = Path(path)
    root = ElementTree.parse(path).getroot()
    points = []
    for name in ("ImagePointLst", "WorldPointLst"):
        node = root.find(".//CalibPointFListParam[@ParamName='%s']" % name)
        if node is None:
            raise ValueError("标定文件缺少%s：%s" % (name, path))
        values = [(float(point.findtext("X")), float(point.findtext("Y")))
                  for point in node.findall("PointF")]
        if not all(math.isfinite(value) for pair in values for value in pair):
            raise ValueError("标定点含非有限数值：%s" % path)
        points.append(values)
    count_node = root.find(".//CalibParam[@ParamName='TransNum']/ParamValue")
    if count_node is None:
        raise ValueError("标定文件缺少TransNum：%s" % path)
    count = int(count_node.text)
    if count < 4 or len(points[0]) != count or len(points[1]) != count:
        raise ValueError("标定文件点数与TransNum不一致：%s" % path)
    matrix_node = root.find(".//CalibFloatListParam[@ParamName='CalibMatrix']")
    matrix = ([float(item.text) for item in matrix_node.findall("ParamValue")]
              if matrix_node is not None else [])
    if len(matrix) != 9 or not all(math.isfinite(value) for value in matrix):
        raise ValueError("标定文件缺少有效的3×3矩阵：%s" % path)
    determinant = (matrix[0] * (matrix[4] * matrix[8] - matrix[5] * matrix[7])
                   - matrix[1] * (matrix[3] * matrix[8] - matrix[5] * matrix[6])
                   + matrix[2] * (matrix[3] * matrix[7] - matrix[4] * matrix[6]))
    if abs(determinant) < 1e-15:
        raise ValueError("标定矩阵不可逆：%s" % path)
    return count


def _atomic_copy(source, destination):
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(
                mode="wb", prefix=".calibration_", suffix=".tmp",
                dir=destination.parent, delete=False) as output:
            temporary = Path(output.name)
            with Path(source).open("rb") as input_file:
                shutil.copyfileobj(input_file, output)
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary, destination)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()


def calibration_sha256(path):
    """标定XML的版本指纹，用于阻止新矩阵与旧XY偏移混用。"""
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _atomic_write_bytes(content, destination):
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(
                mode="wb", prefix=".calibration_binding_", suffix=".tmp",
                dir=destination.parent, delete=False) as output:
            temporary = Path(output.name)
            output.write(content)
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary, destination)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()


def activate_calibration(candidate, active=None, history_dir=None, binding_file=None,
                         quality_report=None, persistent_report_file=None):
    """校验候选，归档旧正式版，再原子替换正式 XML；返回旧版路径。

    ``persistent_report_file`` 用于把生成矩阵所需的完整观测报告固化到
    resources，避免清理 output 后闭环偏移失去零点样本。
    """
    candidate = Path(candidate).resolve()
    active = Path(active or config.TASK2_CALIBRATION_FILE).resolve()
    history_dir = Path(history_dir or config.TASK2_CALIBRATION_HISTORY_DIR).resolve()
    if binding_file is None and active == Path(config.TASK2_CALIBRATION_FILE).resolve():
        binding_file = config.TASK2_CALIBRATION_BINDING_FILE
    if candidate == active:
        raise ValueError("候选文件不能就是正式标定文件。")
    validate_calibration(candidate)
    candidate_hash = calibration_sha256(candidate)
    same_as_active = active.exists() and calibration_sha256(active) == candidate_hash
    backup = None
    if active.exists() and not same_as_active:
        validate_calibration(active)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        backup = history_dir / (active.stem + "_" + stamp + active.suffix)
        if backup.exists():
            raise FileExistsError("历史版本文件名冲突：%s" % backup)
        _atomic_copy(active, backup)
    if binding_file is not None:
        # 先写新指纹，再切换XML；中途失败时运行任务会因指纹不一致而停止。
        binding = {"active_xml_sha256": candidate_hash,
                   "activated_at": datetime.now().isoformat(timespec="seconds")}
        if quality_report is not None:
            try:
                world_rms = float(quality_report["world_rms_mm"])
                pixel_rms = float(quality_report["pixel_rms"])
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError("质量报告缺少有效的world_rms_mm/pixel_rms。") from exc
            if not math.isfinite(world_rms) or not math.isfinite(pixel_rms):
                raise ValueError("质量报告RMS必须是有限数值。")
            source_report = quality_report.get("report_path")
            if persistent_report_file is not None:
                persistent_report_file = Path(persistent_report_file).resolve()
                persistent_report = dict(quality_report)
                persistent_report["report_path"] = str(persistent_report_file)
                _atomic_write_bytes(
                    json.dumps(persistent_report, ensure_ascii=False, indent=2).encode("utf-8"),
                    persistent_report_file,
                )
                source_report = str(persistent_report_file)
            binding["quality"] = {
                "world_rms_mm": world_rms,
                "pixel_rms": pixel_rms,
                "point_count": int(validate_calibration(candidate)),
                "source_report": source_report,
            }
        _atomic_write_bytes(json.dumps(binding, ensure_ascii=False, indent=2).encode("utf-8"),
                            Path(binding_file))
    if not same_as_active:
        _atomic_copy(candidate, active)
    return backup


def list_calibration_versions(history_dir=None):
    history_dir = Path(history_dir or config.TASK2_CALIBRATION_HISTORY_DIR)
    return sorted(history_dir.glob("*.xml"), reverse=True) if history_dir.exists() else []
