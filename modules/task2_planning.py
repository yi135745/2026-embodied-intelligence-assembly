"""任务二协议、装配规划与命令补偿；纯计算，不访问硬件或其他功能库。"""

import math
from copy import deepcopy

import config
import numpy as np

from contracts.task2 import normalize_square_angle, select_square_rotation


def validate_actions(actions):
    """保留原文顺序，验证六次落盘和随后 1~3 次叠放。"""
    expected = config.TASK2_EXPECTED_STACK_COUNT
    if expected is not None and (type(expected) is not int or expected not in range(4)):
        raise ValueError("TASK2_EXPECTED_STACK_COUNT必须为None或0~3。")
    counts = (7, 8, 9) if expected is None else (6 + expected,)
    if not isinstance(actions, list) or len(actions) not in counts:
        raise ValueError("任务卡动作数不符合规则，允许总步数：%s。" % (counts,))
    used, covered, result = set(), set(), []
    fields = {"step", "source_color", "target_type", "target_color"}
    for index, action in enumerate(actions, 1):
        if not isinstance(action, dict) or set(action) != fields:
            raise ValueError("第%d步字段必须为step/source_color/target_type/target_color。" % index)
        if type(action["step"]) is not int or action["step"] != index:
            raise ValueError("第%d步编号不连续，禁止自动重排。" % index)
        source, kind, target = (action[key] for key in
                                ("source_color", "target_type", "target_color"))
        if not all(isinstance(value, str) for value in (source, kind, target)):
            raise ValueError("第%d步颜色与目标类型必须为字符串。" % index)
        if source in config.TASK2_DISABLED_BLOCK_COLORS:
            raise ValueError("第%d步需要已屏蔽的%s方块；缺件期间禁止执行该任务卡。" %
                             (index, source))
        if source not in config.TASK2_BLOCK_COLORS or source in used:
            raise ValueError("第%d步来源颜色无效或重复使用：%s。" % (index, source))
        if index <= 6:
            if (source not in config.TASK2_TRAY_COLORS or kind != "tray" or
                    target not in config.TASK2_TRAY_COLORS):
                raise ValueError("前六步必须为六色方块放到六色托盘。")
        else:
            if source not in config.TASK2_EXTRA_BLOCK_COLORS or kind != "block":
                raise ValueError("第%d步必须为青/粉/棕方块叠放。" % index)
            if target not in used or target in covered:
                raise ValueError("第%d步目标方块尚未放置或顶面已被占用：%s。" %
                                 (index, target))
            covered.add(target)
        used.add(source)
        result.append(dict(action))
    if {a["target_color"] for a in result[:6]} != set(config.TASK2_TRAY_COLORS):
        raise ValueError("前六步必须分别覆盖六色托盘，不能重复占用。")
    return result


def _finite(value, label):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(label + "必须为有限数值。")
    return value


def _height(color):
    height = _finite(config.TASK2_BLOCK_HEIGHT_MM[color], color + "高度")
    if height <= 0:
        raise ValueError(color + "高度必须大于0。")
    return height


def _pose(target):
    if target.robot_pose is None or len(target.robot_pose) != 6:
        raise ValueError(target.color + "缺少六维机器人位姿。")
    return [_finite(v, target.color + "位姿") for v in target.robot_pose]


def build_plan(actions, blocks, trays):
    """一次识别后预测所有终态；执行器仅在抓放成功后提交对应终态。"""
    actions = validate_actions(actions)
    block_map, tray_map = {}, {}
    for targets, mapping in ((blocks, block_map), (trays, tray_map)):
        for target in targets:
            if target.color in mapping:
                raise ValueError("视觉存在重复颜色：" + target.color)
            mapping[target.color] = target
    reference = _finite(config.TASK2_REFERENCE_BLOCK_HEIGHT_MM, "参考高度")
    if reference <= 0:
        raise ValueError("参考高度必须大于0。")
    predicted, plan = {}, []
    for action in actions:
        color = action["source_color"]
        if color not in block_map:
            raise ValueError("未识别到来源方块：" + color)
        source = block_map[color]
        pick_pose = _pose(source)
        height = _height(color)
        pick_pose[2] += height - reference
        source_angle = source.robot_angle_deg
        if action["target_type"] == "tray":
            target_color = action["target_color"]
            if target_color not in tray_map:
                raise ValueError("未识别到目标托盘：" + target_color)
            target = tray_map[target_color]
            place_pose = _pose(target)
            # TCP接触顶面的等效Z基准，不是物理托盘表面Z。
            support_z = place_pose[2] - reference
            target_angle = target.robot_angle_deg
        else:
            state = predicted[action["target_color"]]
            place_pose = list(state["pose"])
            support_z = state["top_tcp_z"]
            target_angle = state["angle_deg"]
        delta = 0.0
        if config.TASK2_ROTATION_ENABLED:
            if source_angle is None or target_angle is None:
                raise ValueError("缺少机器人坐标系方向角，无法对准：" + color)
            delta = select_square_rotation(
                _finite(target_angle, "目标角") - _finite(source_angle, "来源角"),
                config.TASK2_ROTATION_DIRECTION,
            )
        # 保持抓取时rx/ry；rz为ZYX欧拉角，绕基座Z增量直接加到rz。
        place_pose[3:] = pick_pose[3:]
        place_pose[5] = (pick_pose[5] + math.radians(delta) + math.pi) % (2 * math.pi) - math.pi
        place_pose[2] = support_z + height
        final_angle = None if source_angle is None else normalize_square_angle(source_angle + delta)
        state = {"pose": list(place_pose), "angle_deg": final_angle,
                 "top_tcp_z": place_pose[2], "height_mm": height,
                 "target_type": action["target_type"], "target_color": action["target_color"]}
        predicted[color] = state
        pick = source.to_dict()
        pick["robot_pose"] = pick_pose
        plan.append({**action, "pick": pick,
                     "place": {"robot_pose": place_pose,
                               "robot_angle_deg": final_angle},
                     "rotation_delta_deg": delta, "placed_state": deepcopy(state),
                     "robot_status": "pending"})
    return plan


def _design_rows(target_xy, rotation_deg, reference_xy, spatial):
    dx, dy = np.asarray(target_xy, dtype=float) - np.asarray(reference_xy, dtype=float)
    angle = math.radians(float(rotation_deg))
    a, s = 1.0 - math.cos(angle), math.sin(angle)
    if spatial:
        return ([1., 0., dx, dy, 0., 0., a, s],
                [0., 1., 0., 0., dx, dy, -s, a])
    return ([1., 0., a, s], [0., 1., -s, a])


def fit_motion_compensation(trials):
    """拟合执行误差；样本不足时显式降阶。"""
    if not trials:
        raise ValueError("执行补偿至少需要1组落点观测。")
    targets = np.asarray([item["target_robot_xy"] for item in trials], dtype=float)
    residuals = np.asarray([item["placement_residual_mm"] for item in trials], dtype=float)
    rotations = np.asarray([item["command_rotation_deg"] for item in trials], dtype=float)
    if targets.shape != residuals.shape or targets.shape[1:] != (2,):
        raise ValueError("执行补偿的目标和残差必须为二维坐标。")
    if not np.all(np.isfinite(targets)) or not np.all(np.isfinite(residuals)):
        raise ValueError("执行补偿观测包含非有限数值。")
    reference = np.mean(targets, axis=0)

    def solve(spatial):
        rows, observed = [], []
        for target, angle, residual in zip(targets, rotations, residuals):
            xrow, yrow = _design_rows(target, angle, reference, spatial)
            rows.extend((xrow, yrow))
            observed.extend(residual)
        design = np.asarray(rows, dtype=float)
        if np.linalg.matrix_rank(design) < design.shape[1]:
            return None
        parameters, *_ = np.linalg.lstsq(design, np.asarray(observed), rcond=None)
        fitted = design @ parameters
        errors = fitted - np.asarray(observed)
        return parameters, float(np.sqrt(np.mean(errors ** 2)))

    solved = solve(spatial=True)
    spatial = solved is not None
    if solved is None:
        solved = solve(spatial=False)
    if solved is None:
        center = np.median(residuals, axis=0)
        errors = residuals - center
        parameters = np.asarray([center[0], center[1], 0., 0.])
        fit_rms = float(np.sqrt(np.mean(errors ** 2)))
        mode = "constant"
    else:
        parameters, fit_rms = solved
        mode = "spatial_rotation" if spatial else "constant_rotation"
    if spatial:
        constant = parameters[:2]
        matrix = np.asarray([[parameters[2], parameters[3]],
                             [parameters[4], parameters[5]]])
        bias = parameters[6:8]
    else:
        constant = parameters[:2]
        matrix = np.zeros((2, 2), dtype=float)
        bias = parameters[2:4]
    return {
        "model_version": 1,
        "mode": mode,
        "reference_robot_xy": reference.tolist(),
        "constant_bias_mm": constant.tolist(),
        "spatial_residual_matrix": matrix.tolist(),
        "rotation_center_bias_mm": bias.tolist(),
        "sample_count": int(len(trials)),
        "fit_rms_mm": fit_rms,
    }


def fit_constant_motion_compensation(trials):
    """只拟合固定二维执行偏差；空间矩阵和旋转偏心明确归零。"""
    if not trials:
        raise ValueError("固定执行补偿至少需要1组落点观测。")
    targets = np.asarray([item["target_robot_xy"] for item in trials], dtype=float)
    residuals = np.asarray([item["placement_residual_mm"] for item in trials], dtype=float)
    if targets.shape != residuals.shape or targets.shape[1:] != (2,):
        raise ValueError("固定执行补偿的目标和残差必须为二维坐标。")
    if not np.all(np.isfinite(targets)) or not np.all(np.isfinite(residuals)):
        raise ValueError("固定执行补偿观测包含非有限数值。")
    reference = np.mean(targets, axis=0)
    constant = np.median(residuals, axis=0)
    errors = residuals - constant
    return {
        "model_version": 1,
        "mode": "constant_median",
        "reference_robot_xy": reference.tolist(),
        "constant_bias_mm": constant.tolist(),
        "spatial_residual_matrix": [[0.0, 0.0], [0.0, 0.0]],
        "rotation_center_bias_mm": [0.0, 0.0],
        "sample_count": int(len(trials)),
        "fit_rms_mm": float(np.sqrt(np.mean(errors ** 2))),
    }


def predict_motion_residual(model, target_xy, rotation_deg):
    if not model:
        return np.zeros(2, dtype=float)
    reference = np.asarray(model["reference_robot_xy"], dtype=float)
    constant = np.asarray(model["constant_bias_mm"], dtype=float)
    matrix = np.asarray(model["spatial_residual_matrix"], dtype=float)
    bias = np.asarray(model["rotation_center_bias_mm"], dtype=float)
    if (reference.shape != (2,) or constant.shape != (2,) or
            matrix.shape != (2, 2) or bias.shape != (2,)):
        raise ValueError("执行补偿模型字段维度无效。")
    angle = math.radians(float(rotation_deg))
    rotation = np.asarray([[math.cos(angle), -math.sin(angle)],
                           [math.sin(angle), math.cos(angle)]])
    return (constant + matrix @ (np.asarray(target_xy, dtype=float) - reference) +
            (np.eye(2) - rotation) @ bias)


def compensate_place_pose(place_pose, rotation_deg, model):
    """返回实际 TCP 命令；输入标称位姿不修改。"""
    command = list(map(float, place_pose))
    correction = predict_motion_residual(model, command[:2], rotation_deg)
    command[0] -= float(correction[0])
    command[1] -= float(correction[1])
    return command, correction.tolist()


def apply_motion_compensation(plan, compensation=None):
    """把标称计划转换为执行计划；不修改输入和 placed_state。"""
    result = deepcopy(plan)
    models = (compensation or {}).get("models", {})
    for item in result:
        pick_pose = item["pick"]["robot_pose"]
        place_pose = item["place"]["robot_pose"]
        model = models.get(item["target_type"])
        command_place, residual = compensate_place_pose(
            place_pose, item["rotation_delta_deg"], model)
        item["pick"]["command_robot_pose"] = list(pick_pose)
        item["place"]["command_robot_pose"] = command_place
        item["place"]["predicted_motion_residual_mm"] = residual
    return result
