"""任务二标定功能库的唯一、按需加载的公共入口。

正式任务不导入本包；现场 workflow 通过这里完成拟合、质量评估、启用和恢复。
按需加载避免只读版本审计被迫初始化拟合依赖。
"""

from importlib import import_module

__all__ = [
    "GRID_OFFSETS_MM", "activate_calibration", "apply_tray_residual",
    "assess_calibration", "calibration_sha256", "fit_relative_homography",
    "fit_rotation_center_bias", "list_calibration_versions",
    "print_activation_summary", "proposed_block_offset", "proposed_tray_offset",
    "quality_issues_from_report", "rebase_xy_offset",
    "require_calibration_quality", "summarize_rotation_response",
    "validate_calibration", "write_vm_xml",
]

_EXPORT_MODULES = {
    "GRID_OFFSETS_MM": "_xml",
    "write_vm_xml": "_xml",
    "activate_calibration": "_versions",
    "calibration_sha256": "_versions",
    "list_calibration_versions": "_versions",
    "validate_calibration": "_versions",
    "assess_calibration": "_quality",
    "print_activation_summary": "_quality",
    "quality_issues_from_report": "_quality",
    "require_calibration_quality": "_quality",
    "fit_relative_homography": "_relative",
    "apply_tray_residual": "_closed_loop",
    "fit_rotation_center_bias": "_closed_loop",
    "proposed_block_offset": "_closed_loop",
    "proposed_tray_offset": "_closed_loop",
    "rebase_xy_offset": "_closed_loop",
    "summarize_rotation_response": "_closed_loop",
}


def __getattr__(name):
    module_name = _EXPORT_MODULES.get(name)
    if module_name is None:
        raise AttributeError(name)
    value = getattr(import_module("%s.%s" % (__name__, module_name)), name)
    globals()[name] = value
    return value
