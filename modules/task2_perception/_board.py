"""感知库内部的白板边界与轮廓门禁。"""

import cv2
import numpy as np


def _clusters(lines, horizontal):
    groups = {}
    for x1, y1, x2, y2 in lines.reshape(-1, 4):
        dx, dy = abs(x2 - x1), abs(y2 - y1)
        if horizontal:
            if dx < 4 * max(1, dy):
                continue
            position, length = (y1 + y2) / 2, dx
        else:
            if dy < 4 * max(1, dx):
                continue
            position, length = (x1 + x2) / 2, dy
        key = round(position / 8) * 8
        groups.setdefault(key, []).append((position, length))
    return [(sum(p * length for p, length in group) / sum(length for _, length in group),
             sum(length for _, length in group)) for group in groups.values()]


def detect_block_board(image):
    """检测近似水平的白板四条边，返回带安全内缩的(x0,y0,x1,y1)。"""
    height, width = image.shape[:2]
    scale = min(1.0, 1024.0 / width)
    small = cv2.resize(image, (round(width * scale), round(height * scale)))
    gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(cv2.GaussianBlur(gray, (5, 5), 0), 25, 80)
    segments = cv2.HoughLinesP(edges, 1, np.pi / 180, 40,
                               minLineLength=max(30, round(gray.shape[1] * 0.14)),
                               maxLineGap=max(10, round(gray.shape[1] * 0.04)))
    if segments is None:
        raise ValueError("未找到物块白板边界；禁止使用未限定区域的视觉结果。")
    horizontal = _clusters(segments, True)
    vertical = _clusters(segments, False)
    sh, sw = gray.shape
    strip = max(4, round(sw * 0.008))

    def x_contrast(x):
        x = round(x)
        if x - 3 * strip < 0 or x + 3 * strip >= sw:
            return 0.0
        return float(gray[round(sh * .2):round(sh * .85), x + strip:x + 3 * strip].mean() -
                     gray[round(sh * .2):round(sh * .85), x - 3 * strip:x - strip].mean())

    left = [(x, score) for x, score in vertical if .05 * sw < x < .55 * sw and x_contrast(x) > 22]
    right = [(x, score) for x, score in vertical if .45 * sw < x < .95 * sw and x_contrast(x) < -22]
    if not left or not right:
        raise ValueError("物块白板左右边界不可靠；禁止抓放。")
    x0 = max(left, key=lambda item: item[1])[0]
    x1 = max(right, key=lambda item: item[1])[0]
    if x1 - x0 < .4 * sw:
        raise ValueError("物块白板宽度异常；禁止抓放。")
    xa, xb = round(x0 + .08 * (x1 - x0)), round(x1 - .08 * (x1 - x0))

    def y_contrast(y):
        y = round(y)
        if y - 3 * strip < 0 or y + 3 * strip >= sh:
            return 0.0
        return float(gray[y + strip:y + 3 * strip, xa:xb].mean() -
                     gray[y - 3 * strip:y - strip, xa:xb].mean())

    def dark_line(y):
        y = round(y)
        if y - 5 * strip < 0 or y + strip >= sh:
            return 0.0
        return float(gray[y - 5 * strip:y - 3 * strip, xa:xb].mean() -
                     gray[y - strip:y + strip, xa:xb].mean())

    top = [(y, score) for y, score in horizontal if .05 * sh < y < .5 * sh and y_contrast(y) > 25]
    bottom = [(y, score) for y, score in horizontal if .55 * sh < y < .95 * sh and dark_line(y) > 18]
    if not top or not bottom:
        raise ValueError("物块白板上下边界不可靠；禁止抓放。")
    y0 = max(top, key=lambda item: item[1])[0]
    y1 = max(bottom, key=lambda item: item[1])[0]
    if y1 - y0 < .4 * sh:
        raise ValueError("物块白板高度异常；禁止抓放。")
    inset = max(6, round(.012 * sw))
    bounds = tuple(round(value / scale) for value in (x0 + inset, y0 + inset,
                                                       x1 - inset, y1 - inset))
    if bounds[2] <= bounds[0] or bounds[3] <= bounds[1]:
        raise ValueError("物块白板有效区域为空；禁止抓放。")
    return bounds


def valid_block_contour(contour, bounds):
    """要求整个近正方形位于白板内，尺寸相对板宽合理。"""
    x0, y0, x1, y1 = bounds
    area = cv2.contourArea(contour)
    rect = cv2.minAreaRect(contour)
    (cx, cy), (width, height), _angle = rect
    if min(width, height) <= 0:
        return False
    board_width = x1 - x0
    side_min, side_max = .06 * board_width, .17 * board_width
    if not (side_min <= width <= side_max and side_min <= height <= side_max):
        return False
    if max(width, height) / min(width, height) > 1.35 or area / (width * height) < .65:
        return False
    corners = cv2.boxPoints(rect)
    return bool(np.all(corners[:, 0] >= x0) and np.all(corners[:, 0] <= x1) and
                np.all(corners[:, 1] >= y0) and np.all(corners[:, 1] <= y1) and
                x0 <= cx <= x1 and y0 <= cy <= y1)
