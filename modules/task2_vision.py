"""任务二 OpenCV 视觉：图像预处理、六色目标检测和坐标接口。"""

from dataclasses import asdict, dataclass
import itertools
import json
from pathlib import Path
from typing import List, Optional, Tuple
from xml.etree import ElementTree

import cv2
import numpy as np

import config


_TASK2_XY_CALIBRATION = None


def load_task2_tuning(path=None) -> bool:
    """加载调参工具生成的JSON；文件不存在时继续使用config默认值。"""
    tuning_path = Path(path or config.TASK2_TUNING_FILE)
    if not tuning_path.exists():
        return False
    data = json.loads(tuning_path.read_text(encoding="utf-8"))
    for json_key, config_key in (("hsv_ranges", "TASK2_HSV_RANGES"),
                                 ("block_hsv_ranges", "TASK2_BLOCK_HSV_RANGES"),
                                 ("tray_hsv_ranges", "TASK2_TRAY_HSV_RANGES")):
        hsv_ranges = data.get(json_key)
        if not hsv_ranges:
            continue
        setattr(config, config_key, {
            color: [(tuple(item[0]), tuple(item[1])) for item in ranges]
            for color, ranges in hsv_ranges.items()
        })
    for name, value in data.get("capture", {}).items():
        key = "TASK2_%s" % name.upper()
        if hasattr(config, key):
            setattr(config, key, value)
    for name, key in (("morph_kernel", "TASK2_MORPH_KERNEL"),
                      ("min_contour_area", "TASK2_MIN_CONTOUR_AREA"),
                      ("max_contour_area", "TASK2_MAX_CONTOUR_AREA")):
        if name in data:
            setattr(config, key, data[name])
    print("已加载任务二调参文件：" + str(tuning_path))
    return True


def load_task2_offsets(path=None) -> bool:
    """加载唯一XY标定JSON；不读取、不修改Z或姿态。"""
    global _TASK2_XY_CALIBRATION
    offset_path = Path(path or config.TASK2_OFFSET_FILE)
    if not offset_path.exists():
        _TASK2_XY_CALIBRATION = None
        return False
    data = json.loads(offset_path.read_text(encoding="utf-8"))
    saved_scale = data.get("calibration_world_scale_mm")
    current_scale = float(config.TASK2_CALIBRATION_WORLD_SCALE_MM)
    if saved_scale is None:
        raise RuntimeError(
            "任务二XY标定JSON未记录九点矩阵单位倍率，可能由旧配置生成；"
            "请重新运行task2_offset_calibrate.py。"
        )
    if abs(float(saved_scale) - current_scale) > 1e-9:
        raise RuntimeError(
            "任务二XY标定JSON的九点矩阵倍率为%s，当前配置为%s；请重新标定。" %
            (saved_scale, current_scale)
        )
    required = ("block_origin_xy", "block_xy_offset", "block_view_orientation_rad",
                "tray_origin_xy", "tray_xy_offset", "tray_view_orientation_rad")
    missing = [key for key in required if key not in data]
    if missing:
        raise RuntimeError("任务二XY标定JSON缺少字段：%s" % missing)
    calibration = {}
    for key in required:
        value = data[key]
        expected_length = 3 if key.endswith("orientation_rad") else 2
        if not isinstance(value, list) or len(value) != expected_length:
            raise RuntimeError("任务二XY标定字段%s必须是%d个数。" % (key, expected_length))
        calibration[key] = [float(value[0]), float(value[1])]
        if expected_length == 3:
            calibration[key].append(float(value[2]))
    for prefix, view_pose in (("block", config.TASK2_BLOCK_VIEW_POSE),
                              ("tray", config.TASK2_TRAY_VIEW_POSE)):
        if view_pose is None:
            raise RuntimeError("尚未从aubo_poses.json加载%s拍照位。" % prefix)
        saved_xy = calibration[prefix + "_origin_xy"]
        saved_orientation = calibration[prefix + "_view_orientation_rad"]
        xy_error = max(abs(saved_xy[i] - float(view_pose[i])) for i in range(2))
        angle_error = max(abs(saved_orientation[i] - float(view_pose[i + 3])) for i in range(3))
        if xy_error > 0.5 or angle_error > 0.005:
            raise RuntimeError(
                "%s拍照位已改变，但XY标定JSON仍属于旧位姿；请重新运行task2_offset_calibrate.py。" % prefix
            )
    _TASK2_XY_CALIBRATION = calibration
    print("已加载任务二XY标定文件（Z仍取config）：" + str(offset_path))
    return True


@dataclass
class VisionTarget:
    kind: str
    color: str
    pixel_center: Tuple[float, float]
    area: float
    angle_deg: float
    robot_pose: Optional[List[float]] = None

    def to_dict(self) -> dict:
        return asdict(self)


class CoordinateTransformer:
    """读取 VisionMaster 九点标定 XML，把像素映射到机器人平面坐标。"""

    def __init__(self, calibration_file=None):
        self.calibration_file = Path(calibration_file) if calibration_file else None
        self.matrix = self._load_matrix() if self.calibration_file else None

    def _load_matrix(self):
        if not self.calibration_file.exists():
            raise RuntimeError("找不到VisionMaster标定文件：" + str(self.calibration_file))
        root = ElementTree.parse(self.calibration_file).getroot()
        node = root.find(".//CalibFloatListParam[@ParamName='CalibMatrix']")
        values = [float(item.text) for item in node.findall("ParamValue")] if node is not None else []
        if len(values) != 9:
            raise RuntimeError("VisionMaster标定文件中CalibMatrix必须包含9个数值。")
        return np.asarray(values, dtype=np.float64).reshape(3, 3)

    @property
    def ready(self) -> bool:
        return self.matrix is not None and load_task2_offsets()

    def pixel_to_world(self, pixel_x: float, pixel_y: float) -> Optional[Tuple[float, float]]:
        if self.matrix is None:
            return None
        mapped = self.matrix @ np.asarray([pixel_x, pixel_y, 1.0], dtype=np.float64)
        if abs(mapped[2]) < 1e-12:
            raise RuntimeError("九点标定矩阵产生无效齐次坐标。")
        scale = float(config.TASK2_CALIBRATION_WORLD_SCALE_MM)
        return float(mapped[0] / mapped[2]) * scale, float(mapped[1] / mapped[2]) * scale

    def pixel_to_robot(self, pixel_x: float, pixel_y: float, kind: str) -> Optional[List[float]]:
        """返回机器人位姿，XYZ为mm，姿态为rad；未填写人工基准时返回None。"""
        world = self.pixel_to_world(pixel_x, pixel_y)
        global _TASK2_XY_CALIBRATION
        if _TASK2_XY_CALIBRATION is None:
            load_task2_offsets()
        if world is None or _TASK2_XY_CALIBRATION is None:
            return None
        prefix = "block" if kind == "方块" else "tray"
        origin = _TASK2_XY_CALIBRATION[prefix + "_origin_xy"]
        offset = _TASK2_XY_CALIBRATION[prefix + "_xy_offset"]
        z = config.TASK2_BLOCK_PICK_Z if kind == "方块" else config.TASK2_TRAY_PLACE_Z
        view_pose = config.TASK2_BLOCK_VIEW_POSE if kind == "方块" else config.TASK2_TRAY_VIEW_POSE
        if z is None:
            return None
        if view_pose is None:
            return None
        orientation = list(view_pose[3:])
        return [origin[0] + world[0] + offset[0], origin[1] + world[1] + offset[1], float(z)] + orientation


class ColorObjectDetector:
    """使用 HSV 阈值和轮廓检测六色方块或托盘区域。"""

    def __init__(self, transformer=None):
        self.transformer = transformer or CoordinateTransformer(config.TASK2_CALIBRATION_FILE)

    def detect(self, image_path, kind: str, debug_dir=None, debug_prefix=None,
               include_robot_pose: bool = True):
        """检测目标；偏移标定时可跳过依赖旧偏移JSON的机器人位姿换算。"""
        if kind not in ("方块", "托盘"):
            raise ValueError("kind 必须是方块或托盘")
        image = cv2.imread(str(image_path))
        if image is None:
            raise RuntimeError("无法读取视觉图片：" + str(image_path))
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        annotated = image.copy()
        targets = []
        debug_path = Path(debug_dir) if debug_dir else None
        if debug_path:
            debug_path.mkdir(parents=True, exist_ok=True)
        size = max(1, int(config.TASK2_MORPH_KERNEL))
        kernel = np.ones((size, size), np.uint8)
        hsv_ranges = config.TASK2_BLOCK_HSV_RANGES if kind == "方块" else config.TASK2_TRAY_HSV_RANGES
        color_masks = {}
        for color_index, (color, ranges) in enumerate(hsv_ranges.items(), start=1):
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lower, upper in ranges:
                mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            color_masks[color] = mask
            if debug_path:
                prefix = debug_prefix or kind
                cv2.imwrite(str(debug_path / ("%s_%s_mask.png" % (prefix, color))), mask)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            candidates = [c for c in contours if config.TASK2_MIN_CONTOUR_AREA <= cv2.contourArea(c) <= config.TASK2_MAX_CONTOUR_AREA]
            if not candidates:
                continue
            contour = max(candidates, key=cv2.contourArea)
            rect = cv2.minAreaRect(contour)
            (cx, cy), _, angle = rect
            robot_pose = self.transformer.pixel_to_robot(cx, cy, kind) if include_robot_pose else None
            target = VisionTarget(kind, color, (float(cx), float(cy)),
                                  float(cv2.contourArea(contour)), float(angle), robot_pose)
            targets.append(target)
            box = cv2.boxPoints(rect).astype(np.int32)
            cv2.drawContours(annotated, [box], 0, (255, 255, 255), 2)
            cv2.circle(annotated, (round(cx), round(cy)), 5, (0, 0, 0), -1)
            cv2.putText(annotated, "C%d" % color_index, (max(0, round(cx) - 35), max(25, round(cy) - 15)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        fallback = None
        if getattr(config, "TASK2_COLOR_FALLBACK_ENABLED", True):
            fallback = self._combined_color_fallback(
                image, hsv, kind, hsv_ranges, color_masks, kernel, include_robot_pose
            )
        if fallback is not None:
            targets, annotated = fallback
            if debug_path:
                prefix = debug_prefix or kind
                cv2.imwrite(str(debug_path / ("%s_combined_detected.jpg" % prefix)), annotated)
            print("%s已使用HSV覆盖率 + HSV色相 + Lab颜色距离联合识别。" % kind)
        return targets, annotated

    @staticmethod
    def _needs_fallback(targets, expected_count):
        if not getattr(config, "TASK2_COLOR_FALLBACK_ENABLED", True):
            return False
        if len(targets) != expected_count:
            return True
        areas = np.asarray([target.area for target in targets], dtype=np.float64)
        return bool(areas.min() < np.median(areas) * float(config.TASK2_COLOR_BAD_AREA_RATIO))

    def _combined_color_fallback(self, image, hsv, kind, hsv_ranges, color_masks, kernel,
                                 include_robot_pose=True):
        """宽掩膜找六个物体，再用三种颜色证据进行六色一对一全局分配。"""
        broad = cv2.inRange(
            hsv,
            (0, int(config.TASK2_COLOR_FALLBACK_MIN_S), int(config.TASK2_COLOR_FALLBACK_MIN_V)),
            (179, 255, 255),
        )
        broad = cv2.morphologyEx(broad, cv2.MORPH_OPEN, kernel)
        broad = cv2.morphologyEx(broad, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(broad, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        filtered = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if not config.TASK2_MIN_CONTOUR_AREA <= area <= config.TASK2_MAX_CONTOUR_AREA:
                continue
            _, (width, height), _ = cv2.minAreaRect(contour)
            if min(width, height) <= 0:
                continue
            aspect_ratio = max(width, height) / min(width, height)
            rectangle_fill = area / (width * height)
            if (aspect_ratio <= float(config.TASK2_COLOR_MAX_ASPECT_RATIO) and
                    rectangle_fill >= float(config.TASK2_COLOR_MIN_RECT_FILL)):
                filtered.append(contour)
        contours = sorted(filtered, key=cv2.contourArea, reverse=True)
        while (len(contours) > len(hsv_ranges) and
               cv2.contourArea(contours[0]) >
               cv2.contourArea(contours[1]) * float(config.TASK2_COLOR_MAX_AREA_JUMP)):
            contours.pop(0)
        contours = contours[:max(len(hsv_ranges), int(config.TASK2_COLOR_MAX_CANDIDATES))]
        if len(contours) < len(hsv_ranges):
            return None

        colors = list(hsv_ranges)
        scores = np.zeros((len(contours), len(colors)), dtype=np.float64)
        measurements = []
        for row, contour in enumerate(contours):
            object_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            cv2.drawContours(object_mask, [contour], -1, 255, -1)
            # 腐蚀后只取物体内部，避开阴影、白边和高光边缘。
            inner = cv2.erode(object_mask, np.ones((11, 11), np.uint8))
            if cv2.countNonZero(inner) < 50:
                inner = object_mask
            pixels_hsv = hsv[inner > 0]
            pixels_lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)[inner > 0]
            median_hsv = np.median(pixels_hsv, axis=0)
            median_lab = np.median(pixels_lab, axis=0)
            measurements.append((median_hsv, median_lab))
            total = max(1, cv2.countNonZero(inner))
            for col, color in enumerate(colors):
                coverage = cv2.countNonZero(cv2.bitwise_and(color_masks[color], inner)) / total
                prototype_scores = []
                for lower, upper in hsv_ranges[color]:
                    prototype_hsv = np.asarray([(lower[i] + upper[i]) / 2.0 for i in range(3)])
                    hue_delta = abs(median_hsv[0] - prototype_hsv[0])
                    hue_distance = min(hue_delta, 180.0 - hue_delta) / 90.0
                    proto_bgr = cv2.cvtColor(
                        np.uint8([[[round(prototype_hsv[0]), round(prototype_hsv[1]), round(prototype_hsv[2])]]]),
                        cv2.COLOR_HSV2BGR,
                    )
                    proto_lab = cv2.cvtColor(proto_bgr, cv2.COLOR_BGR2LAB)[0, 0].astype(float)
                    lab_distance = np.linalg.norm(median_lab - proto_lab) / 255.0
                    prototype_scores.append(
                        float(config.TASK2_COLOR_HUE_WEIGHT) * hue_distance +
                        float(config.TASK2_COLOR_LAB_WEIGHT) * lab_distance
                    )
                scores[row, col] = (float(config.TASK2_COLOR_MASK_WEIGHT) * (1.0 - coverage) +
                                    min(prototype_scores))

        # assignment[color_index] = contour_index。允许从多于六个候选中选择，
        # 避免台外较大杂物仅凭面积挤掉真正物体。
        best_assignment = min(
            itertools.permutations(range(len(contours)), len(colors)),
            key=lambda assignment: sum(scores[row, col] for col, row in enumerate(assignment)),
        )
        annotated = image.copy()
        targets = []
        for color_index, contour_index in enumerate(best_assignment):
            contour = contours[contour_index]
            color = colors[color_index]
            rect = cv2.minAreaRect(contour)
            (cx, cy), _, angle = rect
            robot_pose = self.transformer.pixel_to_robot(cx, cy, kind) if include_robot_pose else None
            target = VisionTarget(kind, color, (float(cx), float(cy)), float(cv2.contourArea(contour)),
                                  float(angle), robot_pose)
            targets.append(target)
            box = cv2.boxPoints(rect).astype(np.int32)
            cv2.drawContours(annotated, [box], 0, (255, 255, 255), 2)
            cv2.circle(annotated, (round(cx), round(cy)), 5, (0, 0, 0), -1)
            cv2.putText(annotated, "C%d*" % (color_index + 1),
                        (max(0, round(cx) - 35), max(25, round(cy) - 15)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        return targets, annotated


def validate_six_colors(targets, kind: str) -> None:
    expected = set(config.TASK2_HSV_RANGES)
    actual = {item.color for item in targets}
    if actual != expected:
        raise RuntimeError("%s颜色识别不完整，缺少：%s，多出：%s" % (kind, sorted(expected-actual), sorted(actual-expected)))
