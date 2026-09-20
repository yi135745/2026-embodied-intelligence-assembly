"""国赛指令校验和装配计划；纯计算，不访问硬件。角度统一为机器人基座XY方向。"""

import math
from copy import deepcopy

import config


def normalize_square_angle(angle_deg):
    """正方形90度等价，选择[-45,45)内的最小转角。"""
    return (float(angle_deg) + 45.0) % 90.0 - 45.0


def validate_actions(actions):
    """保留原文顺序，验证六次落盘和随后1~3次叠放；不补全模型输出。"""
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
        source, kind, target = (action[key] for key in ("source_color", "target_type", "target_color"))
        if not all(isinstance(value, str) for value in (source, kind, target)):
            raise ValueError("第%d步颜色与目标类型必须为字符串。" % index)
        if source in config.TASK2_DISABLED_BLOCK_COLORS:
            raise ValueError("第%d步需要已屏蔽的%s方块；缺件期间禁止执行该任务卡。" % (index, source))
        if source not in config.TASK2_BLOCK_COLORS or source in used:
            raise ValueError("第%d步来源颜色无效或重复使用：%s。" % (index, source))
        if index <= 6:
            if source not in config.TASK2_TRAY_COLORS or kind != "tray" or target not in config.TASK2_TRAY_COLORS:
                raise ValueError("前六步必须为六色方块放到六色托盘。")
        else:
            if source not in config.TASK2_EXTRA_BLOCK_COLORS or kind != "block":
                raise ValueError("第%d步必须为青/粉/棕方块叠放。" % index)
            if target not in used or target in covered:
                raise ValueError("第%d步目标方块尚未放置或顶面已被占用：%s。" % (index, target))
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
            delta = normalize_square_angle(_finite(target_angle, "目标角") - _finite(source_angle, "来源角"))
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
                     "place": {"robot_pose": place_pose, "robot_angle_deg": final_angle},
                     "rotation_delta_deg": delta, "placed_state": deepcopy(state),
                     "robot_status": "pending"})
    return plan
