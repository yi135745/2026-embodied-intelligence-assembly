"""任务二标定矩阵的来源报告追溯与质量门禁。"""

import json
from pathlib import Path

import config
from ._versions import calibration_sha256, validate_calibration


MAX_WORLD_RMS_MM = 1.0
MAX_PIXEL_RMS_PX = 10.0


def _bound_quality(xml_path):
    """读取随正式XML固化的质量摘要；output被清理后仍可追溯。"""
    xml_path = Path(xml_path).resolve()
    pairs = ((Path(config.TASK2_CALIBRATION_FILE).resolve(),
              Path(config.TASK2_CALIBRATION_BINDING_FILE)),)
    for active_path, binding_path in pairs:
        if xml_path != active_path or not binding_path.exists():
            continue
        try:
            binding = json.loads(binding_path.read_text(encoding="utf-8"))
            if binding.get("active_xml_sha256") != calibration_sha256(xml_path):
                return None, None
            quality = binding.get("quality")
            if not isinstance(quality, dict):
                return None, None
            if quality.get("world_rms_mm") is None or quality.get("pixel_rms") is None:
                return None, None
            return binding_path, quality
        except (OSError, ValueError, json.JSONDecodeError):
            return None, None
    return None, None


def quality_issues_from_report(report, max_world_rms_mm=MAX_WORLD_RMS_MM,
                               max_pixel_rms_px=MAX_PIXEL_RMS_PX):
    issues = []
    world_rms = report.get("world_rms_mm")
    pixel_rms = report.get("pixel_rms")
    if world_rms is None or pixel_rms is None:
        issues.append("来源报告缺少world_rms_mm或pixel_rms")
        return issues
    if float(world_rms) > float(max_world_rms_mm):
        issues.append("世界RMS %.3f mm > %.3f mm" %
                      (float(world_rms), float(max_world_rms_mm)))
    if float(pixel_rms) > float(max_pixel_rms_px):
        issues.append("像素RMS %.3f px > %.3f px" %
                      (float(pixel_rms), float(max_pixel_rms_px)))
    return issues


def find_source_report(xml_path, output_root):
    """按候选XML指纹找到生成当前矩阵的最新报告。"""
    target_hash = calibration_sha256(xml_path)
    matches = []
    root = Path(output_root)
    if not root.exists():
        return None, None
    for report_path in root.rglob("*report.json"):
        try:
            report = json.loads(report_path.read_text(encoding="utf-8"))
            candidate = report.get("candidate_xml")
            if not candidate:
                continue
            candidate_path = Path(candidate)
            if candidate_path.exists() and calibration_sha256(candidate_path) == target_hash:
                matches.append((report_path.stat().st_mtime, report_path, report))
        except (OSError, ValueError, KeyError, json.JSONDecodeError):
            continue
    if not matches:
        return None, None
    _mtime, report_path, report = max(matches, key=lambda item: item[0])
    return report_path, report


def assess_calibration(xml_path, output_root):
    xml_path = Path(xml_path)
    result = {
        "xml": str(xml_path.resolve()),
        "sha256": None,
        "point_count": None,
        "source_report": None,
        "world_rms_mm": None,
        "pixel_rms": None,
        "quality_known": False,
        "issues": [],
    }
    try:
        result["point_count"] = validate_calibration(xml_path)
        result["sha256"] = calibration_sha256(xml_path)
    except Exception as exc:
        result["issues"].append("标定XML无效：%s" % exc)
        return result
    # 正式矩阵优先使用随版本固化的绑定；候选矩阵再扫描 output 来源报告。
    report_path, report = _bound_quality(xml_path)
    if report is None:
        report_path, report = find_source_report(xml_path, output_root)
    if report is None:
        return result
    bound_source = report.get("source_report")
    if bound_source and Path(bound_source).exists():
        report_path = Path(bound_source)
    result["source_report"] = str(report_path.resolve())
    result["world_rms_mm"] = report.get("world_rms_mm")
    result["pixel_rms"] = report.get("pixel_rms")
    result["quality_known"] = True
    result["issues"].extend(quality_issues_from_report(report))
    return result


def require_calibration_quality(xml_path, output_root, label):
    result = assess_calibration(xml_path, output_root)
    if result["issues"]:
        raise RuntimeError(
            "%s标定质量门禁未通过：%s。来源报告：%s" %
            (label, "；".join(result["issues"]), result["source_report"] or "未找到"))
    if not result["quality_known"]:
        raise RuntimeError(
            "%s标定质量未知：来源报告和正式绑定摘要均不存在；"
            "为避免删除output后让坏矩阵绕过门禁，请恢复带质量记录的历史版或重做标定。" % label)
    print("%s矩阵质量通过：world RMS=%.3f mm，pixel RMS=%.3f px" %
          (label, float(result["world_rms_mm"]), float(result["pixel_rms"])))
    return result


def print_activation_summary(label, active_path, candidate_path, output_root,
                             report=None, consequence=None, dependencies=()):
    """在覆盖确认前统一输出当前版、候选版、质量和影响。"""
    active_path, candidate_path = Path(active_path), Path(candidate_path)
    candidate_count = validate_calibration(candidate_path)
    candidate_hash = calibration_sha256(candidate_path)
    active_hash = calibration_sha256(active_path) if active_path.exists() else None
    source_report = None
    if report is None:
        source_path, report = find_source_report(candidate_path, output_root)
        source_report = str(source_path.resolve()) if source_path else None
    else:
        source_report = report.get("report_path")
    issues = (quality_issues_from_report(report) if report is not None else [])
    state = "FAIL" if issues else ("PASS" if report is not None else "UNKNOWN")
    print("\n========== %s启用决策 ==========" % label)
    print("当前正式：%s" % active_path.resolve())
    print("当前指纹：%s" % (active_hash or "无"))
    print("候选文件：%s" % candidate_path.resolve())
    print("候选指纹：%s（%d对点）" % (candidate_hash, candidate_count))
    if report is not None:
        print("候选质量：world RMS=%.3f mm / 阈值%.3f mm；"
              "pixel RMS=%.3f px / 阈值%.3f px" %
              (float(report["world_rms_mm"]), MAX_WORLD_RMS_MM,
               float(report["pixel_rms"]), MAX_PIXEL_RMS_PX))
    else:
        print("候选质量：未找到可追溯报告，无法离线判定RMS。")
    print("质量结论：" + state)
    if source_report:
        print("来源报告：" + source_report)
    if issues:
        print("拒绝原因：" + "；".join(issues))
    if consequence:
        print("启用影响：" + consequence)
    blocked_dependencies = []
    for dependency_label, dependency_path in dependencies:
        dependency = assess_calibration(dependency_path, output_root)
        dependency_state = ("FAIL" if dependency["issues"] else
                            ("PASS" if dependency["quality_known"] else "UNKNOWN"))
        metrics = ""
        if dependency["quality_known"]:
            metrics = "（world RMS=%.3f mm，pixel RMS=%.3f px）" % (
                float(dependency["world_rms_mm"]), float(dependency["pixel_rms"]))
        print("关联前置：%s矩阵=%s%s" %
              (dependency_label, dependency_state, metrics))
        if dependency["issues"]:
            blocked_dependencies.append(dependency_label + "：" + "；".join(dependency["issues"]))
        elif not dependency["quality_known"]:
            blocked_dependencies.append(
                dependency_label + "：质量未知（无来源报告或正式绑定摘要）")
    if blocked_dependencies:
        print("整体闭环状态：BLOCKED（当前候选可独立保存，"
              "但不得进入XY/旋转/抓放闭环）")
        print("阻塞原因：" + " | ".join(blocked_dependencies))
    else:
        print("整体闭环前置：未发现关联矩阵质量阻塞。")
    print("备份策略：启用前先原子存档当前正式文件。")
    if issues:
        print("建议动作：不启用，检查原图/错检后重做标定。")
    elif blocked_dependencies:
        print("建议动作：候选自身通过，可启用保存；但修复上述关联前置前不得进入闭环。")
    elif report is None:
        print("建议动作：只在已人工核对来源且将立即做纯平移验收时启用。")
    else:
        print("建议动作：质量门禁通过，核对标注图后可启用。")
    return {"state": state, "issues": issues, "active_sha256": active_hash,
            "candidate_sha256": candidate_hash, "candidate_point_count": candidate_count,
            "source_report": source_report,
            "blocked_dependencies": blocked_dependencies}
