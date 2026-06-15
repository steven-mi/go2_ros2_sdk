# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

"""Convert Go2 proximity sensors (range_obstacle) into a LaserScan for scan fusion."""

import math
from typing import List

import rclpy
from rclpy.node import Node
from go2_interfaces.msg import Go2State
from sensor_msgs.msg import LaserScan


class ProximityToLaserScanNode(Node):
    """Publish a sparse LaserScan from /go2_states range_obstacle readings."""

    def __init__(self) -> None:
        super().__init__('proximity_to_laserscan')

        self.declare_parameter('state_topic', '/go2_states')
        self.declare_parameter('scan_topic', '/scan/proximity')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('sensor_angles', [0.0, 1.5708, 3.14159, -1.5708])
        self.declare_parameter('beam_width', 0.52)
        self.declare_parameter('angle_min', -3.14159)
        self.declare_parameter('angle_max', 3.14159)
        self.declare_parameter('angle_increment', 0.00872665)
        self.declare_parameter('range_min', 0.05)
        self.declare_parameter('range_max', 3.0)
        self.declare_parameter('invalid_threshold', 0.01)

        state_topic = self.get_parameter('state_topic').get_parameter_value().string_value
        scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value

        self._frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self._sensor_angles: List[float] = list(
            self.get_parameter('sensor_angles').get_parameter_value().double_array_value
        )
        self._beam_width = self.get_parameter('beam_width').get_parameter_value().double_value
        self._angle_min = self.get_parameter('angle_min').get_parameter_value().double_value
        self._angle_max = self.get_parameter('angle_max').get_parameter_value().double_value
        self._angle_increment = self.get_parameter('angle_increment').get_parameter_value().double_value
        self._range_min = self.get_parameter('range_min').get_parameter_value().double_value
        self._range_max = self.get_parameter('range_max').get_parameter_value().double_value
        self._invalid_threshold = (
            self.get_parameter('invalid_threshold').get_parameter_value().double_value
        )

        self._scan_size = int(
            math.floor((self._angle_max - self._angle_min) / self._angle_increment) + 1
        )

        qos = rclpy.qos.QoSProfile(
            depth=5,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
        )

        self._scan_pub = self.create_publisher(LaserScan, scan_topic, qos)
        self.create_subscription(Go2State, state_topic, self._state_callback, qos)

        self.get_logger().info(
            f'Proximity scan: {state_topic} -> {scan_topic} '
            f'({len(self._sensor_angles)} sensors, {self._scan_size} bins)'
        )

    def _state_callback(self, msg: Go2State) -> None:
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self._frame_id
        scan.angle_min = float(self._angle_min)
        scan.angle_max = float(self._angle_max)
        scan.angle_increment = float(self._angle_increment)
        scan.time_increment = 0.0
        scan.scan_time = 0.05
        scan.range_min = float(self._range_min)
        scan.range_max = float(self._range_max)
        scan.ranges = [float('inf')] * self._scan_size
        scan.intensities = [0.0] * self._scan_size

        half_width = self._beam_width * 0.5
        for sensor_idx, angle in enumerate(self._sensor_angles):
            if sensor_idx >= len(msg.range_obstacle):
                break

            distance = float(msg.range_obstacle[sensor_idx])
            if distance <= self._invalid_threshold or distance > self._range_max:
                continue

            start_angle = angle - half_width
            end_angle = angle + half_width
            start_bin = max(
                0,
                int(math.floor((start_angle - self._angle_min) / self._angle_increment)),
            )
            end_bin = min(
                self._scan_size - 1,
                int(math.ceil((end_angle - self._angle_min) / self._angle_increment)),
            )

            for bin_idx in range(start_bin, end_bin + 1):
                if distance < scan.ranges[bin_idx]:
                    scan.ranges[bin_idx] = distance
                    scan.intensities[bin_idx] = 100.0

        self._scan_pub.publish(scan)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ProximityToLaserScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
