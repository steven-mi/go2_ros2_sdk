# Shared LiDAR + proximity sensor fusion pipeline for mapping and navigation.
# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

from launch_ros.actions import Node


def _laserscan_params(*, min_height: float, max_height: float, mapping_mode: bool) -> dict:
    if mapping_mode:
        return {
            'target_frame': 'base_link',
            'max_height': max_height,
            'min_height': min_height,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.00872665,  # 0.5 deg
            'scan_time': 0.1,
            'range_min': 0.2,
            'range_max': 20.0,
            'use_inf': True,
            'concurrency_level': 1,
        }
    return {
        'target_frame': 'base_link',
        'max_height': max_height,
        'min_height': min_height,
        'angle_min': -3.14159,
        'angle_max': 3.14159,
        'angle_increment': 0.0174533,  # 1 deg
        'scan_time': 0.033,
        'range_min': 0.1,
        'range_max': 20.0,
        'use_inf': True,
        'concurrency_level': 1,
    }


def get_sensor_fusion_nodes(
    *,
    mapping_mode: bool = True,
    include_semantic_scan: bool = False,
) -> list:
    """Return nodes that fuse UT LiDAR height slices + proximity into /scan."""

    scan_topics = [
        '/scan/lidar_low',
        '/scan/lidar_high',
        '/scan/proximity',
    ]
    if include_semantic_scan:
        scan_topics.append('/scan/semantic')

    if mapping_mode:
        aggregator_params = {
            'input_topic': '/pointcloud/current',
            'aggregation_mode': 'instant',
            'max_range': 20.0,
            'min_range': 0.2,
            'height_filter_min': -1.0,
            'height_filter_max': 3.0,
            'downsample_rate': 1,
            'publish_rate': 20.0,
        }
        low_height = (-0.15, 0.45)
        high_height = (0.15, 1.5)
    else:
        aggregator_params = {
            'input_topic': '/pointcloud/current',
            'aggregation_mode': 'instant',
            'max_range': 20.0,
            'min_range': 0.1,
            'height_filter_min': -2.0,
            'height_filter_max': 3.0,
            'downsample_rate': 1,
            'publish_rate': 30.0,
        }
        low_height = (-0.2, 0.35)
        high_height = (0.1, 2.0)

    return [
        Node(
            package='lidar_processor_cpp',
            executable='pointcloud_aggregator_node',
            name='pointcloud_aggregator',
            parameters=[aggregator_params],
        ),
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='go2_laserscan_low',
            remappings=[
                ('cloud_in', '/pointcloud/filtered'),
                ('scan', '/scan/lidar_low'),
            ],
            parameters=[_laserscan_params(
                min_height=low_height[0],
                max_height=low_height[1],
                mapping_mode=mapping_mode,
            )],
            output='screen',
        ),
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='go2_laserscan_high',
            remappings=[
                ('cloud_in', '/pointcloud/filtered'),
                ('scan', '/scan/lidar_high'),
            ],
            parameters=[_laserscan_params(
                min_height=high_height[0],
                max_height=high_height[1],
                mapping_mode=mapping_mode,
            )],
            output='screen',
        ),
        Node(
            package='go2_robot_sdk',
            executable='proximity_to_laserscan_node',
            name='proximity_to_laserscan',
            parameters=[{
                'state_topic': '/go2_states',
                'scan_topic': '/scan/proximity',
                'frame_id': 'base_link',
                'angle_increment': 0.00872665 if mapping_mode else 0.0174533,
                'range_min': 0.05,
                'range_max': 3.0,
            }],
            output='screen',
        ),
        Node(
            package='lidar_processor_cpp',
            executable='laser_scan_merger_node',
            name='laser_scan_merger',
            parameters=[{
                'scan_topics': scan_topics,
                'output_topic': '/scan',
                'max_age_sec': 0.5,
            }],
            output='screen',
        ),
    ]
