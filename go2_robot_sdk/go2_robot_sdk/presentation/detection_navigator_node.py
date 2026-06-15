# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

"""Navigate to detected objects and publish semantic obstacle scans for Nav2."""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from go2_interfaces.srv import ListDetectedObjects, NavigateToObject
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, LaserScan, PointCloud2
from std_msgs.msg import ColorRGBA
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker, MarkerArray

from ..application.utils.detection_localizer import (
    LocalizedObject,
    compute_standoff_goal,
    localize_detections,
    pick_best_detection,
)


def _yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


class DetectionNavigatorNode(Node):
    """Localize COCO detections with LiDAR and drive Nav2 goals to them."""

    def __init__(self) -> None:
        super().__init__('detection_navigator')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('camera_frame', 'front_camera')
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('pointcloud_frame', 'odom')
        self.declare_parameter('detection_topic', '/detected_objects')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('pointcloud_topic', '/pointcloud/filtered')
        self.declare_parameter('min_detection_score', 0.6)
        self.declare_parameter('standoff_distance', 1.0)
        self.declare_parameter('bbox_padding_ratio', 0.12)
        self.declare_parameter('max_pointcloud_points', 40000)
        self.declare_parameter('target_classes', ['person', 'chair', 'couch', 'bed', 'dining table'])
        self.declare_parameter('avoid_classes', ['person', 'chair', 'couch', 'bed', 'dining table', 'bench'])
        self.declare_parameter('semantic_obstacle_radius', 0.35)
        self.declare_parameter('publish_semantic_scan', True)
        self.declare_parameter('semantic_scan_rate', 5.0)

        self._map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self._camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self._robot_base_frame = self.get_parameter('robot_base_frame').get_parameter_value().string_value
        self._pointcloud_frame = self.get_parameter('pointcloud_frame').get_parameter_value().string_value
        self._min_detection_score = self.get_parameter('min_detection_score').get_parameter_value().double_value
        self._default_standoff = self.get_parameter('standoff_distance').get_parameter_value().double_value
        self._bbox_padding_ratio = self.get_parameter('bbox_padding_ratio').get_parameter_value().double_value
        self._max_pointcloud_points = self.get_parameter('max_pointcloud_points').get_parameter_value().integer_value
        self._target_classes = list(self.get_parameter('target_classes').get_parameter_value().string_array_value)
        self._avoid_classes = list(self.get_parameter('avoid_classes').get_parameter_value().string_array_value)
        self._semantic_obstacle_radius = self.get_parameter('semantic_obstacle_radius').get_parameter_value().double_value
        self._publish_semantic_scan = self.get_parameter('publish_semantic_scan').get_parameter_value().bool_value

        detection_topic = self.get_parameter('detection_topic').get_parameter_value().string_value
        camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        pointcloud_topic = self.get_parameter('pointcloud_topic').get_parameter_value().string_value

        self._latest_detections: Optional[Detection2DArray] = None
        self._latest_camera_info: Optional[CameraInfo] = None
        self._latest_cloud: Optional[PointCloud2] = None
        self._localized_objects: List[LocalizedObject] = []

        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)

        qos = rclpy.qos.QoSProfile(
            depth=5,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
        )

        self.create_subscription(Detection2DArray, detection_topic, self._detections_callback, 10)
        self.create_subscription(CameraInfo, camera_info_topic, self._camera_info_callback, qos)
        self.create_subscription(PointCloud2, pointcloud_topic, self._cloud_callback, qos)

        self._marker_pub = self.create_publisher(MarkerArray, '/semantic_detections', 10)
        if self._publish_semantic_scan:
            self._semantic_scan_pub = self.create_publisher(LaserScan, '/scan/semantic', qos)
        else:
            self._semantic_scan_pub = None

        self._nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._navigate_srv = self.create_service(NavigateToObject, 'navigate_to_object', self._navigate_to_object)
        self._list_srv = self.create_service(ListDetectedObjects, 'list_detected_objects', self._list_detected_objects)

        scan_rate = self.get_parameter('semantic_scan_rate').get_parameter_value().double_value
        self.create_timer(1.0 / max(scan_rate, 0.1), self._update_localization_and_outputs)

        self.get_logger().info(
            f'Semantic navigation ready. Targets: {self._target_classes}. '
            f'Avoid classes: {self._avoid_classes}.'
        )

    def _detections_callback(self, msg: Detection2DArray) -> None:
        self._latest_detections = msg

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        self._latest_camera_info = msg

    def _cloud_callback(self, msg: PointCloud2) -> None:
        self._latest_cloud = msg

    def _lookup_transform(self, target_frame: str, source_frame: str) -> Optional[TransformStamped]:
        try:
            return self._tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=0.3),
            )
        except TransformException as exc:
            self.get_logger().debug(f'TF {source_frame}->{target_frame} unavailable: {exc}')
            return None

    def _get_robot_xy_in_map(self) -> Optional[Tuple[float, float]]:
        transform = self._lookup_transform(self._map_frame, self._robot_base_frame)
        if transform is None:
            return None
        return (
            transform.transform.translation.x,
            transform.transform.translation.y,
        )

    def _refresh_localized_objects(self) -> List[LocalizedObject]:
        if (
            self._latest_detections is None
            or self._latest_camera_info is None
            or self._latest_cloud is None
        ):
            self._localized_objects = []
            return self._localized_objects

        cloud_frame = self._latest_cloud.header.frame_id or self._pointcloud_frame
        transform = self._lookup_transform(self._camera_frame, cloud_frame)
        if transform is None:
            self._localized_objects = []
            return self._localized_objects

        classes = sorted(set(self._target_classes + self._avoid_classes))
        self._localized_objects = localize_detections(
            self._latest_detections,
            self._latest_camera_info,
            self._latest_cloud,
            transform,
            min_score=self._min_detection_score,
            bbox_padding_ratio=self._bbox_padding_ratio,
            allowed_classes=classes,
            max_points=self._max_pointcloud_points,
        )
        return self._localized_objects

    def _transform_point(self, xyz: Tuple[float, float, float], target_frame: str, source_frame: str) -> Optional[Tuple[float, float, float]]:
        transform = self._lookup_transform(target_frame, source_frame)
        if transform is None:
            return None

        matrix = np.eye(4, dtype=np.float64)
        t = transform.transform.translation
        r = transform.transform.rotation
        rot = np.array([
            [1 - 2 * (r.y * r.y + r.z * r.z), 2 * (r.x * r.y - r.z * r.w), 2 * (r.x * r.z + r.y * r.w)],
            [2 * (r.x * r.y + r.z * r.w), 1 - 2 * (r.x * r.x + r.z * r.z), 2 * (r.y * r.z - r.x * r.w)],
            [2 * (r.x * r.z - r.y * r.w), 2 * (r.y * r.z + r.x * r.w), 1 - 2 * (r.x * r.x + r.y * r.y)],
        ], dtype=np.float64)
        matrix[:3, :3] = rot
        matrix[:3, 3] = np.array([t.x, t.y, t.z], dtype=np.float64)

        source = np.array([xyz[0], xyz[1], xyz[2], 1.0], dtype=np.float64)
        target = matrix @ source
        return float(target[0]), float(target[1]), float(target[2])

    def _object_to_map(self, obj: LocalizedObject) -> Optional[Tuple[float, float, float]]:
        cloud_frame = self._latest_cloud.header.frame_id if self._latest_cloud else self._pointcloud_frame
        if cloud_frame == self._map_frame:
            return obj.position_odom

        return self._transform_point(obj.position_odom, self._map_frame, cloud_frame)

    def _publish_markers(self) -> None:
        marker_array = MarkerArray()
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        for index, obj in enumerate(self._localized_objects):
            map_position = self._object_to_map(obj)
            if map_position is None:
                continue

            marker = Marker()
            marker.header.frame_id = self._map_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'semantic_detections'
            marker.id = index
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = map_position[0]
            marker.pose.position.y = map_position[1]
            marker.pose.position.z = map_position[2]
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.35
            marker.scale.y = 0.35
            marker.scale.z = 0.35
            marker.color = ColorRGBA(r=0.2, g=0.8, b=0.3, a=0.85)
            marker.lifetime = rclpy.duration.Duration(seconds=0.5).to_msg()
            marker.text = obj.class_id
            marker_array.markers.append(marker)

        self._marker_pub.publish(marker_array)

    def _publish_semantic_scan(self) -> None:
        if self._semantic_scan_pub is None:
            return

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self._robot_base_frame
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.radians(2.0)
        scan.time_increment = 0.0
        scan.scan_time = 0.2
        scan.range_min = 0.05
        scan.range_max = 12.0
        bin_count = int((scan.angle_max - scan.angle_min) / scan.angle_increment) + 1
        scan.ranges = [float('inf')] * bin_count
        scan.intensities = [0.0] * bin_count

        avoid = {cls.lower() for cls in self._avoid_classes}
        cloud_frame = self._latest_cloud.header.frame_id if self._latest_cloud else self._pointcloud_frame

        for obj in self._localized_objects:
            if obj.class_id.lower() not in avoid:
                continue

            base_position = self._transform_point(obj.position_odom, self._robot_base_frame, cloud_frame)
            if base_position is None:
                continue

            x, y = base_position[0], base_position[1]
            angle = math.atan2(y, x)
            center_range = max(math.hypot(x, y) - self._semantic_obstacle_radius, scan.range_min)
            half_angle = math.atan2(self._semantic_obstacle_radius, max(center_range, 0.1))

            start_bin = max(0, int(math.floor((angle - half_angle - scan.angle_min) / scan.angle_increment)))
            end_bin = min(bin_count - 1, int(math.ceil((angle + half_angle - scan.angle_min) / scan.angle_increment)))

            for bin_idx in range(start_bin, end_bin + 1):
                if center_range < scan.ranges[bin_idx]:
                    scan.ranges[bin_idx] = float(center_range)
                    scan.intensities[bin_idx] = 100.0

        self._semantic_scan_pub.publish(scan)

    def _update_localization_and_outputs(self) -> None:
        self._refresh_localized_objects()
        self._publish_markers()
        self._publish_semantic_scan()

    def _list_detected_objects(self, _request, response):
        self._refresh_localized_objects()
        for obj in self._localized_objects:
            map_position = self._object_to_map(obj)
            response.class_ids.append(obj.class_id)
            response.scores.append(obj.score)
            point = Point()
            if map_position is not None:
                point.x, point.y, point.z = map_position
                response.localized.append(True)
            else:
                response.localized.append(False)
            response.positions.append(point)
        return response

    def _build_goal_pose(self, object_map_xy: Tuple[float, float], standoff: float) -> Optional[PoseStamped]:
        robot_xy = self._get_robot_xy_in_map()
        if robot_xy is None:
            return None

        goal_x, goal_y, yaw = compute_standoff_goal(object_map_xy, robot_xy, standoff)
        qx, qy, qz, qw = _yaw_to_quaternion(yaw)

        pose = PoseStamped()
        pose.header.frame_id = self._map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = goal_x
        pose.pose.position.y = goal_y
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def _send_navigation_goal(self, goal_pose: PoseStamped) -> bool:
        if not self._nav_action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Nav2 navigate_to_pose action server not available')
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        self._nav_action_client.send_goal_async(goal_msg)
        return True

    def _navigate_to_object(self, request, response):
        class_id = request.class_id.strip()
        if not class_id:
            response.success = False
            response.message = 'class_id must not be empty'
            return response

        if class_id.lower() not in {cls.lower() for cls in self._target_classes}:
            response.success = False
            response.message = f'class_id "{class_id}" is not in target_classes'
            return response

        standoff = request.standoff_distance if request.standoff_distance > 0.0 else self._default_standoff
        localized = self._refresh_localized_objects()
        match = pick_best_detection(localized, class_id)
        if match is None:
            response.success = False
            response.message = f'No localized detection found for "{class_id}"'
            return response

        map_position = self._object_to_map(match)
        if map_position is None:
            response.success = False
            response.message = f'Could not transform "{class_id}" into {self._map_frame}'
            return response

        goal_pose = self._build_goal_pose((map_position[0], map_position[1]), standoff)
        if goal_pose is None:
            response.success = False
            response.message = 'Robot pose unavailable; set initial pose / wait for AMCL'
            return response

        if not self._send_navigation_goal(goal_pose):
            response.success = False
            response.message = 'Nav2 action server unavailable'
            return response

        response.success = True
        response.message = (
            f'Navigating to {match.class_id} (score={match.score:.2f}) '
            f'with {standoff:.1f} m standoff'
        )
        response.goal_pose = goal_pose
        self.get_logger().info(response.message)
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DetectionNavigatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
