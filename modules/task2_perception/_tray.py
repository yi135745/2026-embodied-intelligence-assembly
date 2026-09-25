"""感知库内部、不依赖颜色的六托盘几何中心检测。

比赛托盘由多层同心方框构成。检测先聚合同一托盘的多条嵌套边缘，再从候选中选择
满足两行三列拓扑的六组；颜色只用于正式任务中的身份匹配，不参与标定。
"""

from dataclasses import dataclass
from itertools import combinations
import math

import cv2
import numpy as np
from scipy.optimize import linear_sum_assignment


SLOTS = (
    "top_left", "top_center", "top_right",
    "bottom_left", "bottom_center", "bottom_right",
)


@dataclass(frozen=True)
class TrayLandmark:
    slot: str
    center: tuple
    support_count: int
    median_side_px: float
    source: str = "geometry"


def _square_candidates(image):
    height, width = image.shape[:2]
    image_area = float(height * width)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 25, 90)
    contours, _hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if not image_area * 0.00065 <= area <= image_area * 0.035:
            continue
        (cx, cy), (rect_width, rect_height), angle = cv2.minAreaRect(contour)
        short, long = sorted((float(rect_width), float(rect_height)))
        if short <= 0 or long / short > 1.30:
            continue
        if area / max(1.0, rect_width * rect_height) < 0.65:
            continue
        if not 0.10 * width <= cx <= 0.90 * width:
            continue
        if not 0.10 * height <= cy <= 0.90 * height:
            continue
        candidates.append({
            "center": np.asarray([cx, cy], dtype=np.float64),
            "side": math.sqrt(rect_width * rect_height),
            "box": cv2.boxPoints(((cx, cy), (rect_width, rect_height), angle)),
            "strong": False,
        })
    # 托盘印刷线在强反光下常被 Canny 切成开口轮廓，此时轮廓面积
    # 接近零，但整组同心方框仍是清晰的。小范围闭运算将断点接回，再从
    # 外轮廓提取一个高填充率方框，作为不依赖颜色的强候选。
    closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    closed_contours, _ = cv2.findContours(
        closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in closed_contours:
        area = float(cv2.contourArea(contour))
        if not image_area * 0.004 <= area <= image_area * 0.035:
            continue
        (cx, cy), (rect_width, rect_height), angle = cv2.minAreaRect(contour)
        short, long = sorted((float(rect_width), float(rect_height)))
        if short <= 0 or long / short > 1.30:
            continue
        if area / max(1.0, rect_width * rect_height) < 0.55:
            continue
        if not 0.10 * width <= cx <= 0.90 * width:
            continue
        if not 0.10 * height <= cy <= 0.90 * height:
            continue
        candidates.append({
            "center": np.asarray([cx, cy], dtype=np.float64),
            "side": math.sqrt(rect_width * rect_height),
            "box": cv2.boxPoints(((cx, cy), (rect_width, rect_height), angle)),
            "strong": True,
        })
    return candidates, edges


def _cluster_candidates(candidates, image_shape):
    threshold = 0.025 * min(image_shape[:2])
    clusters = []
    for candidate in sorted(candidates, key=lambda item: item["side"], reverse=True):
        match = next((cluster for cluster in clusters
                      if np.linalg.norm(candidate["center"] - cluster["center"]) <= threshold), None)
        if match is None:
            clusters.append({"items": [candidate], "center": candidate["center"].copy()})
        else:
            match["items"].append(candidate)
            match["center"] = np.median(
                np.asarray([item["center"] for item in match["items"]]), axis=0)
    result = []
    for cluster in clusters:
        # 普通 Canny 轮廓需要至少两层嵌套边支持；闭运算生成的
        # 高填充率外轮廓本身已经聚合了多层方框，可单独作为强候选。
        if len(cluster["items"]) < 2 and not any(
                item.get("strong", False) for item in cluster["items"]):
            continue
        centers = np.asarray([item["center"] for item in cluster["items"]])
        sides = np.asarray([item["side"] for item in cluster["items"]])
        result.append({
            "center": np.median(centers, axis=0),
            "support": len(cluster["items"]),
            "side": float(np.median(sides)),
            "box": max(cluster["items"], key=lambda item: item["side"])["box"],
        })
    return result


def _grid_score(items, image_shape):
    height, width = image_shape[:2]
    ordered_y = sorted(items, key=lambda item: item["center"][1])
    top = sorted(ordered_y[:3], key=lambda item: item["center"][0])
    bottom = sorted(ordered_y[3:], key=lambda item: item["center"][0])
    top_xy = np.asarray([item["center"] for item in top])
    bottom_xy = np.asarray([item["center"] for item in bottom])
    row_gap = float(bottom_xy[:, 1].mean() - top_xy[:, 1].mean())
    top_gaps = np.diff(top_xy[:, 0])
    bottom_gaps = np.diff(bottom_xy[:, 0])
    if (row_gap < 0.07 * height or row_gap > 0.60 * height or
            np.min(top_gaps) < 0.08 * width or np.min(bottom_gaps) < 0.08 * width):
        return None
    if min(np.ptp(top_xy[:, 0]), np.ptp(bottom_xy[:, 0])) < 0.22 * width:
        return None
    row_spread = (np.std(top_xy[:, 1]) + np.std(bottom_xy[:, 1])) / height
    column_spread = np.mean(np.abs(top_xy[:, 0] - bottom_xy[:, 0])) / width
    gaps = np.concatenate((top_gaps, bottom_gaps))
    gap_unevenness = float(np.std(gaps) / max(1.0, np.mean(gaps)))
    row_gap_unevenness = abs(float(np.mean(top_xy[:, 1]) + row_gap - np.mean(bottom_xy[:, 1]))) / height
    support_reward = min(0.03, sum(item["support"] for item in items) * 0.0005)
    score = 5.0 * row_spread + 3.0 * column_spread + gap_unevenness + row_gap_unevenness - support_reward
    return score, top + bottom


def detect_tray_landmarks(image):
    """返回严格按两行三列排序的六个中心及标注图、边缘图。"""
    if image is None or image.ndim != 3:
        raise ValueError("托盘几何检测需要有效BGR图像。")
    candidates, edges = _square_candidates(image)
    clusters = _cluster_candidates(candidates, image.shape)
    # 组合数量做上限保护；优先保留嵌套边最多、方框较大的候选。
    clusters = sorted(clusters, key=lambda item: (item["support"], item["side"]), reverse=True)[:14]
    best = None
    for group in combinations(clusters, 6):
        scored = _grid_score(group, image.shape)
        if scored is not None and (best is None or scored[0] < best[0]):
            best = scored
    if best is None:
        raise RuntimeError(
            "未找到可靠的2×3托盘方框阵列（方形候选%d组）；请检查托盘是否完整入镜、边框是否清晰。" %
            len(clusters)
        )
    score, ordered = best
    # 保守拒绝明显不像规则阵列的结果，避免把台面固定件写进标定数据。
    if score > 0.55:
        raise RuntimeError("托盘2×3阵列几何一致性不足（score=%.3f），拒绝自动标定。" % score)
    landmarks = [
        TrayLandmark(slot, (float(item["center"][0]), float(item["center"][1])),
                     int(item["support"]), float(item["side"]))
        for slot, item in zip(SLOTS, ordered)
    ]
    annotated = image.copy()
    for index, (landmark, item) in enumerate(zip(landmarks, ordered), start=1):
        cv2.drawContours(annotated, [np.rint(item["box"]).astype(np.int32)], 0, (0, 255, 255), 3)
        center = tuple(round(value) for value in landmark.center)
        cv2.circle(annotated, center, 8, (0, 0, 255), -1)
        cv2.putText(annotated, "%d:%s n=%d" % (index, landmark.slot, landmark.support_count),
                    (max(0, center[0] - 80), max(30, center[1] - 20)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
    cv2.putText(annotated, "tray grid score=%.4f" % score, (30, 45),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
    return landmarks, annotated, edges, float(score)


def _match_reference_slots(reference_landmarks, clusters, image_shape):
    """以首帧六槽位为锚，用共同图像位移匹配后续帧；不虚构缺失中心。"""
    references = np.asarray([item.center for item in reference_landmarks], dtype=np.float64)
    candidates = np.asarray([item["center"] for item in clusters], dtype=np.float64)
    if len(candidates) == 0:
        return {}, np.zeros(2, dtype=np.float64), float("inf")
    max_distance = 0.075 * min(image_shape[:2])
    best = None
    # 任意一个真实对应都能提出共同平移假设；匈牙利匹配再由全体候选验证。
    for reference in references:
        for candidate in candidates:
            shift = candidate - reference
            distances = np.linalg.norm(
                (references + shift)[:, None, :] - candidates[None, :, :], axis=2)
            rows, columns = linear_sum_assignment(distances)
            valid = [(row, column, distances[row, column])
                     for row, column in zip(rows, columns)
                     if distances[row, column] <= max_distance]
            if not valid:
                continue
            score = (-len(valid), float(np.median([item[2] for item in valid])),
                     float(np.mean([item[2] for item in valid])))
            if best is None or score < best[0]:
                best = score, valid
    if best is None:
        return {}, np.zeros(2, dtype=np.float64), float("inf")
    valid = best[1]
    # 用所有已匹配中心的中位平移消除单个轮廓中心抖动，再匹配一次。
    shift = np.median(
        np.asarray([candidates[column] - references[row] for row, column, _distance in valid]),
        axis=0)
    distances = np.linalg.norm(
        (references + shift)[:, None, :] - candidates[None, :, :], axis=2)
    rows, columns = linear_sum_assignment(distances)
    matches = {int(row): clusters[int(column)] for row, column in zip(rows, columns)
               if distances[row, column] <= max_distance}
    rms = (float(np.sqrt(np.mean([distances[row, column] ** 2
                                  for row, column in zip(rows, columns)
                                  if distances[row, column] <= max_distance])))
           if matches else float("inf"))
    return matches, shift, rms


def detect_tray_landmarks_tracked(image, reference_landmarks, supplemental_centers=()):
    """跟踪首帧槽位；几何缺失时只接受靠近预测槽位的补充视觉中心。"""
    if len(reference_landmarks) != 6:
        raise ValueError("托盘跟踪参考必须包含首帧六个槽位。")
    candidates, edges = _square_candidates(image)
    clusters = _cluster_candidates(candidates, image.shape)
    clusters = sorted(clusters, key=lambda item: (item["support"], item["side"]), reverse=True)[:14]
    matches, shift, match_rms = _match_reference_slots(reference_landmarks, clusters, image.shape)
    max_supplement_distance = 0.060 * min(image.shape[:2])
    supplemental = [np.asarray(center, dtype=np.float64) for center in supplemental_centers]
    used_supplements = set()
    landmarks = []
    matched_boxes = {}
    for index, reference in enumerate(reference_landmarks):
        if index in matches:
            item = matches[index]
            matched_boxes[index] = item["box"]
            landmarks.append(TrayLandmark(
                reference.slot, (float(item["center"][0]), float(item["center"][1])),
                int(item["support"]), float(item["side"]), "geometry"))
            continue
        expected = np.asarray(reference.center, dtype=np.float64) + shift
        available = [(candidate_index, float(np.linalg.norm(center - expected)))
                     for candidate_index, center in enumerate(supplemental)
                     if candidate_index not in used_supplements]
        if available:
            candidate_index, distance = min(available, key=lambda item: item[1])
            if distance <= max_supplement_distance:
                used_supplements.add(candidate_index)
                center = supplemental[candidate_index]
                landmarks.append(TrayLandmark(
                    reference.slot, (float(center[0]), float(center[1])), 0, 0.0,
                    "hsv_near_predicted_slot"))

    slot_order = {slot: index for index, slot in enumerate(SLOTS)}
    landmarks.sort(key=lambda item: slot_order[item.slot])
    annotated = image.copy()
    for landmark in landmarks:
        index = slot_order[landmark.slot]
        if index in matched_boxes:
            cv2.drawContours(annotated, [np.rint(matched_boxes[index]).astype(np.int32)],
                             0, (0, 255, 255), 3)
        center = tuple(round(value) for value in landmark.center)
        color = (0, 0, 255) if landmark.source == "geometry" else (255, 0, 255)
        cv2.circle(annotated, center, 8, color, -1)
        cv2.putText(annotated, "%s:%s" % (landmark.slot, landmark.source),
                    (max(0, center[0] - 95), max(30, center[1] - 20)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
    cv2.putText(annotated, "observed=%d/6 geometry=%d match_rms=%.2fpx" %
                (len(landmarks), len(matches), match_rms), (30, 45),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
    return landmarks, annotated, edges, {
        "observed_count": len(landmarks),
        "geometry_count": len(matches),
        "supplement_count": len(landmarks) - len(matches),
        "common_image_shift_px": [float(value) for value in shift],
        "geometry_match_rms_px": match_rms,
    }


def detect_tray_landmarks_file(image_path):
    image = cv2.imread(str(image_path))
    if image is None:
        raise RuntimeError("无法读取托盘标定图片：" + str(image_path))
    return detect_tray_landmarks(image)
