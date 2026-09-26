"""任务二 OpenCV 视觉：九色方块、六色托盘、中心和方向标定。"""

import math
import json
import os
from pathlib import Path
from typing import List, Optional, Tuple
from xml.etree import ElementTree

import cv2
import numpy as np
from scipy.optimize import linear_sum_assignment

import config
from contracts.task2 import (
    VisionTarget,
    normalize_square_angle,
)
from ._board import detect_block_board, valid_block_contour
from ._tray import (
    TrayLandmark,
    detect_tray_landmarks,
    detect_tray_landmarks_file,
    detect_tray_landmarks_tracked,
)


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
        # 省赛六色调参文件只覆盖已有颜色，不能抹掉国赛新增三色默认值。
        merged = dict(getattr(config, config_key))
        merged.update({
            color: [(tuple(item[0]), tuple(item[1])) for item in ranges]
            for color, ranges in hsv_ranges.items()
        })
        setattr(config, config_key, merged)
    for name, value in data.get("capture", {}).items():
        key = "TASK2_%s" % name.upper()
        if hasattr(config, key):
            setattr(config, key, value)
    prototypes = data.get("block_color_prototypes_hsv")
    if prototypes:
        merged = dict(getattr(config, "TASK2_BLOCK_COLOR_PROTOTYPES_HSV", {}))
        merged.update({
            color: tuple(tuple(map(float, item)) for item in values)
            for color, values in prototypes.items()
        })
        config.TASK2_BLOCK_COLOR_PROTOTYPES_HSV = merged
    for name, key in (("morph_kernel", "TASK2_MORPH_KERNEL"),
                      ("min_contour_area", "TASK2_MIN_CONTOUR_AREA"),
                      ("max_contour_area", "TASK2_MAX_CONTOUR_AREA")):
        if name in data:
            setattr(config, key, data[name])
    print("已加载任务二调参文件：" + str(tuning_path))
    return True


def save_task2_tuning_patch(payload, path=None) -> Path:
    """合并并原子保存分区调参，避免一个场景覆盖另外两个场景。"""
    target = Path(path or config.TASK2_TUNING_FILE)
    existing = json.loads(target.read_text(encoding="utf-8")) if target.exists() else {}
    capture = {**existing.get("capture", {}), **payload.get("capture", {})}
    reviews = {**existing.get("human_reviews", {}),
               **payload.get("human_reviews", {})}
    existing.update(payload)
    existing["capture"] = capture
    if reviews:
        existing["human_reviews"] = reviews
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_suffix(target.suffix + ".tmp")
    temporary.write_text(json.dumps(existing, ensure_ascii=False, indent=2),
                         encoding="utf-8")
    os.replace(temporary, target)
    return target


def save_debug_image(path, image) -> None:
    """由感知库保存 OpenCV 调试图，避免任务流程直接依赖 OpenCV。"""
    if not cv2.imwrite(str(path), image):
        raise RuntimeError("保存视觉调试图失败：" + str(path))


class CoordinateTransformer:
    """读取 VisionMaster 九点标定 XML，把像素映射到机器人平面坐标。"""

    def __init__(self, calibration_file=None, xy_calibration=None):
        self.calibration_file = Path(calibration_file) if calibration_file else None
        self.matrix = self._load_matrix() if self.calibration_file else None
        self.xy_calibration = xy_calibration

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
        return self.matrix is not None and self.xy_calibration is not None

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
        if world is None or self.xy_calibration is None:
            return None
        prefix = "block" if kind == "方块" else "tray"
        origin = self.xy_calibration[prefix + "_origin_xy"]
        offset = self.xy_calibration[prefix + "_xy_offset"]
        z = config.TASK2_BLOCK_PICK_Z if kind == "方块" else config.TASK2_TRAY_PLACE_Z
        view_pose = config.TASK2_BLOCK_VIEW_POSE if kind == "方块" else config.TASK2_TRAY_VIEW_POSE
        if z is None:
            return None
        if view_pose is None:
            return None
        orientation = list(view_pose[3:])
        return [origin[0] + world[0] + offset[0], origin[1] + world[1] + offset[1], float(z)] + orientation

    def rectangle_angle_to_robot(self, rect):
        """把正方形一条边的两端映射至机器人XY平面，消除图像Y翻转/透视。"""
        if self.matrix is None:
            return None
        corners = cv2.boxPoints(rect)
        a = self.pixel_to_world(*corners[0])
        b = self.pixel_to_world(*corners[1])
        dx, dy = b[0] - a[0], b[1] - a[1]
        if not all(math.isfinite(v) for v in (dx, dy)) or math.hypot(dx, dy) < 1e-9:
            raise ValueError("矩形方向标定结果无效。")
        return normalize_square_angle(math.degrees(math.atan2(dy, dx)))


class ColorObjectDetector:
    """使用HSV及全局匹配检测九色方块或六色托盘。"""

    def __init__(self, transformer=None, xy_calibration=None):
        # 方块与托盘强制引用同一个变换对象，避免调用方重新注入第二套矩阵。
        self.transformer = transformer or CoordinateTransformer(
            config.TASK2_CALIBRATION_FILE, xy_calibration=xy_calibration)
        self.tray_transformer = self.transformer

    def get_transformer(self, kind: str):
        return self.transformer if kind == "方块" else self.tray_transformer

    def detect(self, image_path, kind: str, debug_dir=None, debug_prefix=None,
               include_robot_pose: bool = True, strict_board: bool = False):
        """检测目标；偏移标定时可跳过依赖旧偏移JSON的机器人位姿换算。"""
        if kind not in ("方块", "托盘"):
            raise ValueError("kind 必须是方块或托盘")
        transformer = self.get_transformer(kind)
        image = cv2.imread(str(image_path))
        if image is None:
            raise RuntimeError("无法读取视觉图片：" + str(image_path))
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        annotated = image.copy()
        board_bounds = None
        board_guard_active = strict_board and kind == "方块"
        if board_guard_active:
            board_mode = str(config.TASK2_BLOCK_BOARD_GUARD_MODE).lower()
            if board_mode not in ("strict", "fallback", "off"):
                raise ValueError(
                    "TASK2_BLOCK_BOARD_GUARD_MODE必须是strict、fallback或off。")
            if board_mode != "off":
                try:
                    board_bounds = detect_block_board(
                        image, config.TASK2_BLOCK_BOARD_REGION_SCALE)
                except ValueError as exc:
                    if board_mode == "strict":
                        raise
                    print("警告：%s 已按fallback模式退回全图形状/颜色识别。" % exc)
        targets = []
        debug_path = Path(debug_dir) if debug_dir else None
        if debug_path:
            debug_path.mkdir(parents=True, exist_ok=True)
        size = max(1, int(config.TASK2_MORPH_KERNEL))
        kernel = np.ones((size, size), np.uint8)
        hsv_ranges = (config.TASK2_BLOCK_HSV_RANGES if kind == "方块"
                      else config.TASK2_TRAY_HSV_RANGES)
        if kind == "方块":
            disabled = set(config.TASK2_DISABLED_BLOCK_COLORS)
            hsv_ranges = {color: ranges for color, ranges in hsv_ranges.items()
                          if color not in disabled}
        color_masks = {}
        color_candidates = {}
        geometric_contours = []
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
            candidates = [c for c in contours
                          if config.TASK2_MIN_CONTOUR_AREA <= cv2.contourArea(c) <= config.TASK2_MAX_CONTOUR_AREA
                          and (board_bounds is None or valid_block_contour(c, board_bounds))]
            color_candidates[color] = candidates
            geometric_contours.extend(candidates)
            if not candidates:
                continue
            contour = self._select_color_contour(candidates, hsv, color, kind)
            rect = cv2.minAreaRect(contour)
            (cx, cy), _, angle = rect
            robot_pose = transformer.pixel_to_robot(cx, cy, kind) if include_robot_pose else None
            target = VisionTarget(kind, color, (float(cx), float(cy)),
                                  float(cv2.contourArea(contour)), float(angle), robot_pose,
                                  transformer.rectangle_angle_to_robot(rect))
            targets.append(target)
            if kind != "方块" or color not in ("红色", "粉色"):
                box = cv2.boxPoints(rect).astype(np.int32)
                cv2.drawContours(annotated, [box], 0, (0, 255, 255), 2)
                cv2.circle(annotated, (round(cx), round(cy)), 5, (0, 0, 0), -1)
                cv2.putText(annotated, "C%d" % color_index, (max(0, round(cx) - 35), max(25, round(cy) - 15)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        if kind == "方块" and {"红色", "粉色"}.issubset(hsv_ranges):
            targets, annotated = self._resolve_red_pink(
                hsv, targets, color_candidates, include_robot_pose, annotated)
        fallback = None
        # 严格物块模式不重新洗牌已可靠识别的颜色；形状候选只补剩余颜色。
        if board_guard_active and board_bounds is not None:
            targets, annotated = self._complete_block_targets(
                image, hsv, kind, targets, hsv_ranges, board_bounds, include_robot_pose,
                annotated)
        use_fallback = (False if board_guard_active and board_bounds is not None else
                        (kind == "方块" or self._needs_fallback(targets, len(hsv_ranges))))
        if use_fallback:
            if board_bounds is not None:
                geometric_contours.extend(self._edge_square_contours(image, board_bounds))
            fallback = self._combined_color_fallback(
                image, hsv, kind, hsv_ranges, color_masks, kernel, include_robot_pose,
                board_bounds, geometric_contours
            )
        if fallback is not None:
            targets, annotated = fallback
            if debug_path:
                prefix = debug_prefix or kind
                cv2.imwrite(str(debug_path / ("%s_combined_detected.jpg" % prefix)), annotated)
            print("%s已使用HSV覆盖率 + HSV色相 + Lab颜色距离联合识别。" % kind)
        if kind == "托盘" and targets:
            targets, annotated = self._refine_tray_centers(
                image, targets, include_robot_pose)
        if board_bounds is not None:
            x0, y0, x1, y1 = board_bounds
            cv2.rectangle(annotated, (x0, y0), (x1, y1), (0, 255, 255), 3)
            if len(targets) > 1:
                areas = np.asarray([target.area for target in targets], dtype=np.float64)
                median_area = float(np.median(areas))
                if areas.min() < .45 * median_area or areas.max() > 2.2 * median_area:
                    raise ValueError("方块轮廓大小明显不一致；拒绝将局部色斑当作整块抓放。")
        # 不允许HSV重叠将同一个轮廓同时识别成两种颜色。
        for index, target in enumerate(targets):
            for other in targets[:index]:
                distance = math.dist(target.pixel_center, other.pixel_center)
                if distance < 0.25 * math.sqrt(min(target.area, other.area)):
                    raise ValueError("颜色识别冲突：%s和%s指向同一物块，请调HSV。" % (target.color, other.color))
        return targets, annotated

    def _refine_tray_centers(self, image, targets, include_robot_pose):
        """颜色负责身份，2×3同心方框几何负责托盘最终中心。"""
        landmarks, annotated, _edges, _score = detect_tray_landmarks(
            image, [target.pixel_center for target in targets])
        color_centers = np.asarray(
            [target.pixel_center for target in targets], dtype=np.float64)
        geometry_centers = np.asarray(
            [landmark.center for landmark in landmarks], dtype=np.float64)
        distances = np.linalg.norm(
            color_centers[:, None, :] - geometry_centers[None, :, :], axis=2)
        rows, columns = linear_sum_assignment(distances)
        max_distance = 0.08 * min(image.shape[:2])
        if len(rows) != len(targets) or np.any(distances[rows, columns] > max_distance):
            raise RuntimeError("托盘颜色中心与2×3方框中心无法可靠对应；请检查曝光和HSV。")

        transformer = self.get_transformer("托盘")
        landmark_by_target = {int(row): landmarks[int(column)]
                              for row, column in zip(rows, columns)}
        refined = []
        for index, target in enumerate(targets):
            center = tuple(float(value) for value in landmark_by_target[index].center)
            robot_pose = (transformer.pixel_to_robot(*center, "托盘")
                          if include_robot_pose else None)
            refined.append(VisionTarget(
                target.kind, target.color, center, target.area, target.angle_deg,
                robot_pose, target.robot_angle_deg))
            point = (round(center[0]), round(center[1]))
            cv2.circle(annotated, point, 8, (0, 255, 255), 2)
            cv2.circle(annotated, point, 3, (0, 0, 0), -1)
        return refined, annotated

    @staticmethod
    def _needs_fallback(targets, expected_count):
        if not getattr(config, "TASK2_COLOR_FALLBACK_ENABLED", True):
            return False
        if len(targets) != expected_count:
            return True
        areas = np.asarray([target.area for target in targets], dtype=np.float64)
        return bool(areas.min() < np.median(areas) * float(config.TASK2_COLOR_BAD_AREA_RATIO))

    @staticmethod
    def _prototype_cost(median_hsv, prototypes):
        """红/粉共享色相时提高S/V权重；现场确认原型会覆盖默认原型。"""
        costs = []
        for prototype in prototypes:
            hue_delta = abs(float(median_hsv[0]) - float(prototype[0]))
            hue = min(hue_delta, 180.0 - hue_delta) / 90.0
            saturation = abs(float(median_hsv[1]) - float(prototype[1])) / 255.0
            value = abs(float(median_hsv[2]) - float(prototype[2])) / 255.0
            costs.append(0.15 * hue + 0.35 * saturation + 0.50 * value)
        return min(costs) if costs else float("inf")

    def _resolve_red_pink(self, hsv, targets, color_candidates,
                          include_robot_pose, annotated):
        """对色相重叠的红/粉候选做一次全局一对一判别，禁止同轮廓双占。"""
        colors = ("红色", "粉色")
        contours = self._deduplicate_contours(
            color_candidates.get("红色", []) + color_candidates.get("粉色", []))
        if not contours:
            return [target for target in targets if target.color not in colors], annotated
        scores = np.zeros((len(contours), len(colors)), dtype=np.float64)
        for row, contour in enumerate(contours):
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            cv2.drawContours(mask, [contour], -1, 255, -1)
            inner = cv2.erode(mask, np.ones((11, 11), np.uint8))
            if cv2.countNonZero(inner) < 50:
                inner = mask
            median = np.median(hsv[inner > 0], axis=0)
            for column, color in enumerate(colors):
                prototypes = getattr(
                    config, "TASK2_BLOCK_COLOR_PROTOTYPES_HSV", {}).get(color, ())
                scores[row, column] = self._prototype_cost(median, prototypes)

        rows, columns = linear_sum_assignment(scores)
        resolved = [target for target in targets if target.color not in colors]
        transformer = self.get_transformer("方块")
        for row, column in zip(rows, columns):
            if scores[row, column] > float(config.TASK2_COLOR_MAX_ASSIGNMENT_COST):
                continue
            color = colors[column]
            contour = contours[row]
            rect = cv2.minAreaRect(contour)
            (cx, cy), _, angle = rect
            robot_pose = (transformer.pixel_to_robot(cx, cy, "方块")
                          if include_robot_pose else None)
            resolved.append(VisionTarget(
                "方块", color, (float(cx), float(cy)),
                float(cv2.contourArea(contour)), float(angle), robot_pose,
                transformer.rectangle_angle_to_robot(rect)))
            box = cv2.boxPoints(rect).astype(np.int32)
            cv2.drawContours(annotated, [box], 0, (0, 255, 255), 3)
            cv2.circle(annotated, (round(cx), round(cy)), 5, (0, 0, 0), -1)
            cv2.putText(
                annotated, "RP:%s" % ("R" if color == "红色" else "P"),
                (max(0, round(cx) - 55), max(40, round(cy) + 40)),
                cv2.FONT_HERSHEY_SIMPLEX, 1.25, (0, 255, 255), 4)
        return resolved, annotated

    @staticmethod
    def _select_color_contour(candidates, hsv, color, kind):
        """颜色范围重叠时按实拍原型选块；无原型仍按最大完整轮廓。"""
        prototypes = (getattr(config, "TASK2_BLOCK_COLOR_PROTOTYPES_HSV", {}).get(color, ())
                      if kind == "方块" else ())
        if not prototypes or len(candidates) == 1:
            return max(candidates, key=cv2.contourArea)

        def score(contour):
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            cv2.drawContours(mask, [contour], -1, 255, -1)
            inner = cv2.erode(mask, np.ones((11, 11), np.uint8))
            if cv2.countNonZero(inner) == 0:
                inner = mask
            median = np.median(hsv[inner > 0], axis=0)
            costs = []
            for prototype in prototypes:
                hue = min(abs(median[0] - prototype[0]), 180 - abs(median[0] - prototype[0])) / 90
                saturation = abs(median[1] - prototype[1]) / 255
                value = abs(median[2] - prototype[2]) / 255
                costs.append(.35 * hue + .30 * saturation + .35 * value)
            return min(costs)

        return min(candidates, key=score)

    @staticmethod
    def _edge_square_contours(image, board_bounds):
        """补充低饱和深色块：只接受白板内、尺寸合理的闭合边缘方形。"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(cv2.GaussianBlur(gray, (5, 5), 0), 25, 90)
        edges = cv2.dilate(edges, np.ones((3, 3), np.uint8))
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return [contour for contour in contours if valid_block_contour(contour, board_bounds)]

    @staticmethod
    def _deduplicate_contours(contours):
        kept = []
        for contour in sorted(contours, key=cv2.contourArea, reverse=True):
            (cx, cy), (width, height), _ = cv2.minAreaRect(contour)
            if any(math.dist((cx, cy), center) < .35 * min(width, height)
                   for center, _existing in kept):
                continue
            kept.append(((cx, cy), contour))
        return [contour for _center, contour in kept]

    def _complete_block_targets(self, image, hsv, kind, targets, hsv_ranges, board_bounds,
                                include_robot_pose, annotated):
        """用独立的完整方形补充HSV漏检；已识别颜色及中心保持不变。"""
        missing = [color for color in hsv_ranges if color not in {item.color for item in targets}]
        if not missing:
            return targets, annotated
        contours = []
        for contour in self._edge_square_contours(image, board_bounds):
            rect = cv2.minAreaRect(contour)
            center = rect[0]
            if any(math.dist(center, target.pixel_center) < .35 * min(rect[1]) for target in targets):
                continue
            contours.append(contour)
        contours = self._deduplicate_contours(contours)
        if not contours:
            return targets, annotated

        lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
        scores = np.zeros((len(contours), len(missing)), dtype=np.float64)
        for row, contour in enumerate(contours):
            object_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            cv2.drawContours(object_mask, [contour], -1, 255, -1)
            inner = cv2.erode(object_mask, np.ones((15, 15), np.uint8))
            median_hsv = np.median(hsv[inner > 0], axis=0)
            median_lab = np.median(lab[inner > 0], axis=0)
            for col, color in enumerate(missing):
                prototypes = [tuple((lo[i] + hi[i]) / 2 for i in range(3))
                              for lo, hi in hsv_ranges[color]]
                prototypes.extend(
                    getattr(config, "TASK2_BLOCK_COLOR_PROTOTYPES_HSV", {}).get(color, ()))
                costs = []
                for prototype in prototypes:
                    prototype = np.asarray(prototype, dtype=np.float64)
                    hue_delta = abs(median_hsv[0] - prototype[0])
                    hue_distance = min(hue_delta, 180.0 - hue_delta) / 90.0
                    proto_bgr = cv2.cvtColor(
                        np.uint8([[[round(prototype[0]), round(prototype[1]), round(prototype[2])]]]),
                        cv2.COLOR_HSV2BGR)
                    proto_lab = cv2.cvtColor(proto_bgr, cv2.COLOR_BGR2LAB)[0, 0].astype(float)
                    costs.append(.55 * hue_distance + .45 * np.linalg.norm(median_lab - proto_lab) / 255.0)
                scores[row, col] = min(costs)

        rows, columns = linear_sum_assignment(scores)
        for row, col in zip(rows, columns):
            if scores[row, col] > float(config.TASK2_COLOR_MAX_ASSIGNMENT_COST):
                continue
            contour, color = contours[row], missing[col]
            rect = cv2.minAreaRect(contour)
            (cx, cy), _, angle = rect
            transformer = self.get_transformer(kind)
            robot_pose = transformer.pixel_to_robot(cx, cy, kind) if include_robot_pose else None
            target = VisionTarget(kind, color, (float(cx), float(cy)), float(cv2.contourArea(contour)),
                                  float(angle), robot_pose,
                                  transformer.rectangle_angle_to_robot(rect))
            targets.append(target)
            box = cv2.boxPoints(rect).astype(np.int32)
            cv2.drawContours(annotated, [box], 0, (0, 255, 255), 3)
            cv2.putText(annotated, "S:%s" % color, (max(0, round(cx) - 45), max(25, round(cy) - 15)),
                        cv2.FONT_HERSHEY_SIMPLEX, .65, (0, 255, 255), 2)
        return targets, annotated

    def _combined_color_fallback(self, image, hsv, kind, hsv_ranges, color_masks, kernel,
                                 include_robot_pose=True, board_bounds=None,
                                 geometric_contours=None):
        """宽掩膜找候选，匈牙利算法完成一对一分配，不进行九色全排列。"""
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
            (cx, cy), (width, height), _ = cv2.minAreaRect(contour)
            if min(width, height) <= 0:
                continue
            aspect_ratio = max(width, height) / min(width, height)
            rectangle_fill = area / (width * height)
            if (aspect_ratio <= float(config.TASK2_COLOR_MAX_ASPECT_RATIO) and
                    rectangle_fill >= float(config.TASK2_COLOR_MIN_RECT_FILL) and
                    (board_bounds is None or valid_block_contour(contour, board_bounds)) and
                    (kind != "托盘" or
                     (.08 * image.shape[1] <= cx <= .92 * image.shape[1] and
                      .08 * image.shape[0] <= cy <= .92 * image.shape[0]))):
                filtered.append(contour)
        # 方块的边缘候选用于补低饱和棕色；托盘不能把未经白板区域约束的
        # 单色小轮廓重新塞回联合候选，否则画面外反光会冒充缺失颜色。
        extra_contours = list(geometric_contours or []) if kind == "方块" else []
        contours = self._deduplicate_contours(filtered + extra_contours)
        while (len(contours) > len(hsv_ranges) and
               cv2.contourArea(contours[0]) >
               cv2.contourArea(contours[1]) * float(config.TASK2_COLOR_MAX_AREA_JUMP)):
            contours.pop(0)
        contours = contours[:max(len(hsv_ranges), int(config.TASK2_COLOR_MAX_CANDIDATES))]
        if len(contours) < max(1, len(hsv_ranges) - 2):
            return None

        colors = list(hsv_ranges)
        scores = np.zeros((len(contours), len(colors)), dtype=np.float64)
        lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
        for row, contour in enumerate(contours):
            object_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            cv2.drawContours(object_mask, [contour], -1, 255, -1)
            # 腐蚀后只取物体内部，避开阴影、白边和高光边缘。
            inner = cv2.erode(object_mask, np.ones((11, 11), np.uint8))
            if cv2.countNonZero(inner) < 50:
                inner = object_mask
            pixels_hsv = hsv[inner > 0]
            pixels_lab = lab[inner > 0]
            median_hsv = np.median(pixels_hsv, axis=0)
            median_lab = np.median(pixels_lab, axis=0)
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
                if kind == "方块":
                    for prototype in getattr(config, "TASK2_BLOCK_COLOR_PROTOTYPES_HSV", {}).get(color, ()):
                        prototype_hsv = np.asarray(prototype, dtype=np.float64)
                        hue_delta = abs(median_hsv[0] - prototype_hsv[0])
                        hue_distance = min(hue_delta, 180.0 - hue_delta) / 90.0
                        proto_bgr = cv2.cvtColor(
                            np.uint8([[[round(prototype_hsv[0]), round(prototype_hsv[1]),
                                        round(prototype_hsv[2])]]]), cv2.COLOR_HSV2BGR)
                        proto_lab = cv2.cvtColor(proto_bgr, cv2.COLOR_BGR2LAB)[0, 0].astype(float)
                        lab_distance = np.linalg.norm(median_lab - proto_lab) / 255.0
                        prototype_scores.append(
                            float(config.TASK2_COLOR_HUE_WEIGHT) * hue_distance +
                            float(config.TASK2_COLOR_LAB_WEIGHT) * lab_distance)
                scores[row, col] = (float(config.TASK2_COLOR_MASK_WEIGHT) * (1.0 - coverage) +
                                    min(prototype_scores))

        rows, columns = linear_sum_assignment(scores)
        assignment = sorted(zip(columns, rows))
        annotated = image.copy()
        targets = []
        for color_index, contour_index in assignment:
            if scores[contour_index, color_index] > config.TASK2_COLOR_MAX_ASSIGNMENT_COST:
                continue
            contour = contours[contour_index]
            color = colors[color_index]
            rect = cv2.minAreaRect(contour)
            (cx, cy), _, angle = rect
            transformer = self.get_transformer(kind)
            robot_pose = transformer.pixel_to_robot(cx, cy, kind) if include_robot_pose else None
            target = VisionTarget(kind, color, (float(cx), float(cy)), float(cv2.contourArea(contour)),
                                  float(angle), robot_pose,
                                  transformer.rectangle_angle_to_robot(rect))
            targets.append(target)
            box = cv2.boxPoints(rect).astype(np.int32)
            cv2.drawContours(annotated, [box], 0, (0, 255, 255), 2)
            cv2.circle(annotated, (round(cx), round(cy)), 5, (0, 0, 0), -1)
            cv2.putText(annotated, "C%d*" % (color_index + 1),
                        (max(0, round(cx) - 35), max(25, round(cy) - 15)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        return targets, annotated


def validate_six_colors(targets, kind: str) -> None:
    """兼容原调试工具函数名；国赛方块验证九色、托盘验证六色。"""
    validate_colors(targets, kind)


def validate_colors(targets, kind: str) -> None:
    expected = set(config.TASK2_BLOCK_COLORS if kind == "方块" else config.TASK2_TRAY_COLORS)
    if kind == "方块":
        expected -= set(config.TASK2_DISABLED_BLOCK_COLORS)
    actual = {item.color for item in targets}
    if actual != expected:
        raise RuntimeError("%s颜色识别不完整，缺少：%s，多出：%s" % (kind, sorted(expected-actual), sorted(actual-expected)))
