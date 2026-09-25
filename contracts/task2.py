"""任务二跨功能库共享的数据与数值契约。"""

from dataclasses import asdict, dataclass
import math
from typing import List, Optional, Tuple


@dataclass
class VisionTarget:
    kind: str
    color: str
    pixel_center: Tuple[float, float]
    area: float
    angle_deg: float
    robot_pose: Optional[List[float]] = None
    robot_angle_deg: Optional[float] = None

    def to_dict(self) -> dict:
        return asdict(self)


def normalize_square_angle(angle_deg):
    """正方形 90 度等价，选择 [-45, 45) 内的规范角。"""
    return (float(angle_deg) + 45.0) % 90.0 - 45.0


def select_square_rotation(angle_deg, direction="shortest"):
    """按调用方明确给出的工位方向约束选择 RZ 增量。"""
    delta = normalize_square_angle(angle_deg)
    mode = str(direction).lower()
    if mode == "shortest":
        return delta
    if mode == "positive":
        return delta + 90.0 if delta < -1e-9 else delta
    if mode == "negative":
        return delta - 90.0 if delta > 1e-9 else delta
    raise ValueError("旋转方向必须是 shortest、positive 或 negative。")


def validate_motion_compensation(data):
    """验证持久化执行补偿字段；不负责拟合或应用补偿。"""
    if data is None:
        return {"model_version": 1, "models": {}}
    if not isinstance(data, dict) or int(data.get("model_version", 0)) != 1:
        raise ValueError("motion_compensation必须是version=1的对象。")
    models = data.get("models", {})
    if not isinstance(models, dict):
        raise ValueError("motion_compensation.models必须是对象。")
    for target_type, model in models.items():
        if target_type not in ("tray", "block"):
            raise ValueError("执行补偿目标类型只能是tray或block。")

        def finite_vector(value, length):
            return (isinstance(value, (list, tuple)) and len(value) == length and
                    all(isinstance(item, (int, float)) and math.isfinite(float(item))
                        for item in value))

        reference = model["reference_robot_xy"]
        constant = model["constant_bias_mm"]
        matrix = model["spatial_residual_matrix"]
        bias = model["rotation_center_bias_mm"]
        matrix_valid = (isinstance(matrix, (list, tuple)) and len(matrix) == 2 and
                        all(finite_vector(row, 2) for row in matrix))
        if (not finite_vector(reference, 2) or not finite_vector(constant, 2) or
                not matrix_valid or not finite_vector(bias, 2)):
            raise ValueError("执行补偿模型字段维度无效。")
    return data
