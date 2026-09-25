"""标定库内部的 VisionMaster XML 写入与公共采样网格。"""

from datetime import datetime
from xml.etree import ElementTree

import cv2
import numpy as np


# 蛇形顺序减少机器人空行程；中心顺序由具体workflow自行调整。
GRID_OFFSETS_MM = (
    (-1, -1), (0, -1), (1, -1),
    (1, 0), (0, 0), (-1, 0),
    (-1, 1), (0, 1), (1, 1),
)


def _set_param(root, name, value):
    node = root.find(".//CalibParam[@ParamName='%s']/ParamValue" % name)
    if node is not None:
        node.text = str(value)


def _replace_points(root, name, points):
    node = root.find(".//CalibPointFListParam[@ParamName='%s']" % name)
    if node is None:
        raise RuntimeError("XML模板缺少%s。" % name)
    for child in list(node):
        node.remove(child)
    for x, y in points:
        point = ElementTree.SubElement(node, "PointF")
        ElementTree.SubElement(point, "X").text = "%.10g" % x
        ElementTree.SubElement(point, "Y").text = "%.10g" % y
        ElementTree.SubElement(point, "R").text = "0"


def write_vm_xml(template_path, output_path, image_points, world_points, matrix):
    """写入全部有效点，返回world/image逐点误差及RMS。"""
    tree = ElementTree.parse(template_path)
    root = tree.getroot()
    image_points = np.asarray(image_points, dtype=np.float64)
    world_points = np.asarray(world_points, dtype=np.float64)
    matrix = np.asarray(matrix, dtype=np.float64)
    projected_world = cv2.perspectiveTransform(
        image_points.reshape(-1, 1, 2), matrix).reshape(-1, 2)
    world_errors = projected_world - world_points
    inverse = np.linalg.inv(matrix)
    projected_image = cv2.perspectiveTransform(
        world_points.reshape(-1, 1, 2), inverse).reshape(-1, 2)
    image_errors = projected_image - image_points
    world_rms = float(np.sqrt(np.mean(np.sum(world_errors ** 2, axis=1))))
    pixel_rms = float(np.sqrt(np.mean(np.sum(image_errors ** 2, axis=1))))

    _set_param(root, "CreateCalibTime", datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
    _set_param(root, "CalibType", "NPointCalib")
    _set_param(root, "TransNum", len(image_points))
    _set_param(root, "RotNum", 0)
    _set_param(root, "CalibErrStatus", 0)
    _set_param(root, "TransError", "%.10g" % pixel_rms)
    _set_param(root, "TransWorldError", "%.10g" % world_rms)
    _set_param(root, "PixelPrecisionX", "%.10g" % float(np.sqrt(np.mean(image_errors[:, 0] ** 2))))
    _set_param(root, "PixelPrecisionY", "%.10g" % float(np.sqrt(np.mean(image_errors[:, 1] ** 2))))
    _set_param(root, "PixelPrecision", "%.10g" % pixel_rms)
    _replace_points(root, "ImagePointLst", image_points)
    _replace_points(root, "WorldPointLst", world_points)

    matrix_node = root.find(".//CalibFloatListParam[@ParamName='CalibMatrix']")
    if matrix_node is None:
        raise RuntimeError("XML模板缺少CalibMatrix。")
    for child in list(matrix_node):
        matrix_node.remove(child)
    normalized = matrix / matrix[2, 2]
    for value in normalized.reshape(-1):
        ElementTree.SubElement(matrix_node, "ParamValue").text = "%.12g" % float(value)
    ElementTree.indent(tree, space="    ")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output_path, encoding="UTF-8", xml_declaration=True)
    return world_errors, image_errors, world_rms, pixel_rms
