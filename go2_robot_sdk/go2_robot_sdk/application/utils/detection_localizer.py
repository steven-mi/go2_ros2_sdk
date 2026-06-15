# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

"""Project 2D COCO detections into 3D using LiDAR and camera calibration."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, PointCloud2
from sensor_msgs_py import point_cloud2
from vision_msgs.msg import Detection2D, Detection2DArray


@dataclass(frozen=True)
class BoundingBox:
    class_id: str
    score: float
    u_min: float
    v_min: float
    u_max: float
    v_max: float


@dataclass(frozen=True)
class LocalizedObject:
    class_id: str
    score: float
    position_odom: Tuple[float, float, float]


def parse_detection(detection: Detection2D, min_score: float) -> Optional[BoundingBox]:
    if not detection.results:
        return None

    hypothesis = detection.results[0].hypothesis
    score = float(hypothesis.score)
    if score < min_score:
        return None

    center_u = float(detection.bbox.center.position.x)
    center_v = float(detection.bbox.center.position.y)
    half_w = float(detection.bbox.size_x) * 0.5
    half_h = float(detection.bbox.size_y) * 0.5

    return BoundingBox(
        class_id=str(hypothesis.class_id),
        score=score,
        u_min=center_u - half_w,
        v_min=center_v - half_h,
        u_max=center_u + half_w,
        v_max=center_v + half_h,
    )


def expand_bbox(bbox: BoundingBox, padding_ratio: float) -> BoundingBox:
    width = max(bbox.u_max - bbox.u_min, 1.0)
    height = max(bbox.v_max - bbox.v_min, 1.0)
    pad_u = width * padding_ratio
    pad_v = height * padding_ratio
    return BoundingBox(
        class_id=bbox.class_id,
        score=bbox.score,
        u_min=bbox.u_min - pad_u,
        v_min=bbox.v_min - pad_v,
        u_max=bbox.u_max + pad_u,
        v_max=bbox.v_max + pad_v,
    )


def _transform_to_matrix(transform: TransformStamped) -> np.ndarray:
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    x = rotation.x
    y = rotation.y
    z = rotation.z
    w = rotation.w

    matrix = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w), translation.x],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w), translation.y],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y), translation.z],
        [0.0, 0.0, 0.0, 1.0],
    ], dtype=np.float64)
    return matrix


def _read_cloud_points(cloud: PointCloud2, max_points: int) -> np.ndarray:
    points = np.array(
        list(point_cloud2.read_points(cloud, field_names=('x', 'y', 'z'), skip_nans=True)),
        dtype=np.float64,
    )
    if points.size == 0:
        return points.reshape(0, 3)

    if len(points) > max_points:
        stride = max(1, len(points) // max_points)
        points = points[::stride]

    return points


def localize_bbox_with_pointcloud(
    bbox: BoundingBox,
    camera_info: CameraInfo,
    cloud: PointCloud2,
    transform_cloud_to_camera: TransformStamped,
    max_points: int,
) -> Optional[Tuple[float, float, float]]:
    """Return the object position in the point cloud frame (usually odom)."""

    points = _read_cloud_points(cloud, max_points)
    if points.size == 0:
        return None

    matrix = _transform_to_matrix(transform_cloud_to_camera)
    rotation = matrix[:3, :3]
    translation = matrix[:3, 3]
    points_camera = (rotation @ points.T).T + translation

    fx = float(camera_info.k[0])
    fy = float(camera_info.k[4])
    cx = float(camera_info.k[2])
    cy = float(camera_info.k[5])

    depths: List[float] = []
    for x_c, y_c, z_c in points_camera:
        if z_c <= 0.15:
            continue

        u = fx * x_c / z_c + cx
        v = fy * y_c / z_c + cy
        if bbox.u_min <= u <= bbox.u_max and bbox.v_min <= v <= bbox.v_max:
            depths.append(float(z_c))

    if not depths:
        return None

    depth = float(np.percentile(depths, 10))
    center_u = 0.5 * (bbox.u_min + bbox.u_max)
    center_v = 0.5 * (bbox.v_min + bbox.v_max)
    x_c = (center_u - cx) * depth / fx
    y_c = (center_v - cy) * depth / fy
    z_c = depth

    point_camera = np.array([x_c, y_c, z_c, 1.0], dtype=np.float64)
    inverse = np.linalg.inv(matrix)
    point_cloud_frame = inverse @ point_camera
    return float(point_cloud_frame[0]), float(point_cloud_frame[1]), float(point_cloud_frame[2])


def localize_detections(
    detections: Detection2DArray,
    camera_info: CameraInfo,
    cloud: PointCloud2,
    transform_cloud_to_camera: TransformStamped,
    *,
    min_score: float,
    bbox_padding_ratio: float,
    allowed_classes: Optional[Sequence[str]],
    max_points: int,
) -> List[LocalizedObject]:
    allowed = {cls.lower() for cls in allowed_classes} if allowed_classes else None
    localized: List[LocalizedObject] = []

    for detection in detections.detections:
        parsed = parse_detection(detection, min_score)
        if parsed is None:
            continue
        if allowed is not None and parsed.class_id.lower() not in allowed:
            continue

        expanded = expand_bbox(parsed, bbox_padding_ratio)
        position = localize_bbox_with_pointcloud(
            expanded,
            camera_info,
            cloud,
            transform_cloud_to_camera,
            max_points,
        )
        if position is None:
            continue

        localized.append(
            LocalizedObject(
                class_id=parsed.class_id,
                score=parsed.score,
                position_odom=position,
            )
        )

    return localized


def compute_standoff_goal(
    object_xy: Tuple[float, float],
    robot_xy: Tuple[float, float],
    standoff_distance: float,
) -> Tuple[float, float, float]:
    """Return map-frame goal x, y, yaw facing the object."""

    dx = robot_xy[0] - object_xy[0]
    dy = robot_xy[1] - object_xy[1]
    distance = max(float(np.hypot(dx, dy)), 0.05)
    unit_x = dx / distance
    unit_y = dy / distance

    goal_x = object_xy[0] + unit_x * standoff_distance
    goal_y = object_xy[1] + unit_y * standoff_distance
    yaw = float(np.arctan2(object_xy[1] - goal_y, object_xy[0] - goal_x))
    return goal_x, goal_y, yaw


def pick_best_detection(
    localized_objects: Iterable[LocalizedObject],
    class_id: str,
) -> Optional[LocalizedObject]:
    matches = [
        obj for obj in localized_objects
        if obj.class_id.lower() == class_id.lower()
    ]
    if not matches:
        return None
    return max(matches, key=lambda obj: obj.score)
