# Navigation launch file - optimized for AMCL localization and Nav2
# Usage: ros2 launch go2_robot_sdk navigation.launch.py map:=/path/to/map.yaml

import os
import sys
from typing import List
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sensor_fusion import get_sensor_fusion_nodes


def _default_map_path(env_map_file: str) -> str:
    return env_map_file or '/ros2_ws/data/my_map.yaml'


def _validate_nav_map(map_yaml: str) -> None:
    """Nav2 needs a slam_toolbox occupancy map (.yaml + .pgm), not a .ply LiDAR dump."""
    if not map_yaml:
        raise RuntimeError(
            'No map path set. Pass map:=/ros2_ws/data/YOUR_MAP.yaml or set MAP_FILE in the environment.'
        )
    if os.path.isfile(map_yaml):
        return
    ply_path = map_yaml.rsplit('.', 1)[0] + '.ply'
    hint = ''
    if os.path.isfile(ply_path):
        hint = (
            f'\nFound {ply_path} — that is a raw LiDAR dump, not a Nav2 map.\n'
            'Create a Nav2 map with mapping.launch.py, drive the dog, then use the web UI\n'
            'or slam_toolbox "Save Map" (writes .yaml + .pgm to /ros2_ws/data/).'
        )
    raise RuntimeError(
        f'Nav2 map not found: {map_yaml}\n'
        f'Expected {map_yaml} and matching .pgm in the same folder.{hint}\n'
        'Mapping mode:\n'
        '  cd /ros2_ws/data && ros2 launch go2_robot_sdk mapping.launch.py\n'
        '  # drive around, then Save Map as "my_map" on http://localhost:8080\n'
        'Navigation mode:\n'
        '  ros2 launch go2_robot_sdk navigation.launch.py map:=/ros2_ws/data/my_map.yaml'
    )


def generate_launch_description():
    """Generate launch description for Go2 navigation mode"""
    
    # Environment variables
    robot_token = os.getenv('ROBOT_TOKEN', '')
    robot_ip = os.getenv('ROBOT_IP', '')
    robot_ip_list = robot_ip.replace(" ", "").split(",") if robot_ip else []
    map_file = _default_map_path(os.getenv('MAP_FILE', ''))
    conn_type = os.getenv('CONN_TYPE', 'webrtc')
    _validate_nav_map(map_file)
    
    # Determine connection mode
    conn_mode = "single" if len(robot_ip_list) == 1 and conn_type != "cyclonedds" else "multi"
    
    # Package paths
    package_dir = get_package_share_directory('go2_robot_sdk')
    urdf_file = 'go2.urdf' if conn_mode == 'single' else 'multi_go2.urdf'
    rviz_config = 'single_robot_conf.rviz' if conn_mode == 'single' else 'multi_robot_conf.rviz'
    
    config_paths = {
        'joystick': os.path.join(package_dir, 'config', 'joystick.yaml'),
        'twist_mux': os.path.join(package_dir, 'config', 'twist_mux.yaml'),
        'nav2': os.path.join(package_dir, 'config', 'nav2_params.yaml'),
        'semantic_nav': os.path.join(package_dir, 'config', 'semantic_nav_params.yaml'),
        'rviz': os.path.join(package_dir, 'config', rviz_config),
        'urdf': os.path.join(package_dir, 'urdf', urdf_file),
    }
    
    print(f"🧭 Go2 Navigation Mode:")
    print(f"   Robot IPs: {robot_ip_list}")
    print(f"   Connection: {conn_type} ({conn_mode})")
    print(f"   Map: {map_file}")
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_arg = LaunchConfiguration('map')
    with_rviz = LaunchConfiguration('rviz', default='true')
    with_foxglove = LaunchConfiguration('foxglove', default='true')
    with_joystick = LaunchConfiguration('joystick', default='true')
    with_web_ui = LaunchConfiguration('web_ui', default='true')
    web_dev = LaunchConfiguration('web_dev', default='false')
    with_semantic_nav = LaunchConfiguration('semantic_nav', default='true')
    coco_device = LaunchConfiguration('coco_device', default='cpu')
    coco_threshold = LaunchConfiguration('coco_threshold', default='0.6')
    
    launch_args = [
        DeclareLaunchArgument(
            'map',
            default_value=map_file,
            description='Full path to map yaml file for navigation'
        ),
        DeclareLaunchArgument('rviz', default_value='true', description='Launch RViz2'),
        DeclareLaunchArgument('foxglove', default_value='true', description='Launch Foxglove Bridge'),
        DeclareLaunchArgument('joystick', default_value='true', description='Launch joystick control'),
        DeclareLaunchArgument('web_ui', default_value='true', description='Launch web dashboard on port 8080'),
        DeclareLaunchArgument(
            'web_dev',
            default_value='true',
            description='Vite dev server with hot reload on port 5173 (web_dev:=false for static :8080)',
        ),
        DeclareLaunchArgument(
            'semantic_nav',
            default_value='true',
            description='Enable COCO detection + semantic navigation to detected objects',
        ),
        DeclareLaunchArgument(
            'coco_device',
            default_value='cpu',
            description='Torch device for coco_detector (cpu or cuda)',
        ),
        DeclareLaunchArgument(
            'coco_threshold',
            default_value='0.6',
            description='Minimum COCO detection score',
        ),
    ]
    
    # Load URDF
    with open(config_paths['urdf'], 'r') as file:
        robot_desc = file.read()
    
    # Core nodes
    core_nodes = [
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='go2_robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc
            }],
        ),
        # Main robot driver
        Node(
            package='go2_robot_sdk',
            executable='go2_driver_node',
            name='go2_driver_node',
            output='screen',
            parameters=[{
                'robot_ip': robot_ip,
                'token': robot_token,
                'conn_type': conn_type
            }],
        ),
        # LiDAR accumulation + per-frame cloud for fused /scan
        Node(
            package='lidar_processor_cpp',
            executable='lidar_to_pointcloud_node',
            name='lidar_to_pointcloud',
            remappings=[
                ('robot0/point_cloud2', 'point_cloud2'),
            ] if conn_mode == 'single' else [],
            parameters=[{
                'robot_ip_lst': robot_ip_list,
                'map_name': '3d_map',
                'map_save': 'false'  # Don't save during navigation
            }],
        ),
        *get_sensor_fusion_nodes(mapping_mode=False, include_semantic_scan=True),
        Node(
            package='coco_detector',
            executable='coco_detector_node',
            name='coco_detector_node',
            condition=IfCondition(with_semantic_nav),
            output='screen',
            parameters=[{
                'device': coco_device,
                'detection_threshold': coco_threshold,
                'publish_annotated_image': True,
            }],
        ),
        Node(
            package='go2_robot_sdk',
            executable='detection_navigator_node',
            name='detection_navigator',
            condition=IfCondition(with_semantic_nav),
            output='screen',
            parameters=[config_paths['semantic_nav']],
        ),
        # TTS Node
        Node(
            package='speech_processor',
            executable='tts_node',
            name='tts_node',
            parameters=[{
                'api_key': os.getenv('ELEVENLABS_API_KEY', ''),
                'provider': 'elevenlabs',
                'voice_name': 'XrExE9yKIg1WjnnlVkGX',
                'local_playback': False,
                'use_cache': True,
                'audio_quality': 'standard'
            }],
        ),
    ]
    
    # Teleop nodes
    teleop_nodes = [
        Node(
            package='joy',
            executable='joy_node',
            condition=IfCondition(with_joystick),
            parameters=[config_paths['joystick']]
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='go2_teleop_node',
            condition=IfCondition(with_joystick),
            parameters=[config_paths['twist_mux']],
        ),
        Node(
            package='twist_mux',
            executable='twist_mux',
            output='screen',
            condition=IfCondition(with_joystick),
            parameters=[
                {'use_sim_time': use_sim_time},
                config_paths['twist_mux']
            ],
        ),
    ]
    
    # Visualization nodes
    viz_nodes = [
        Node(
            package='rviz2',
            executable='rviz2',
            condition=IfCondition(with_rviz),
            name='go2_rviz2',
            output='screen',
            arguments=['-d', config_paths['rviz']],
            parameters=[{'use_sim_time': False}]
        ),
    ]
    
    # Include launches
    foxglove_launch = os.path.join(
        get_package_share_directory('foxglove_bridge'),
        'launch', 'foxglove_bridge_launch.xml'
    )
    
    include_launches = [
        # Foxglove Bridge
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(foxglove_launch),
            condition=IfCondition(with_foxglove),
        ),
        # AMCL Localization
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('nav2_bringup'),
                            'launch', 'localization_launch.py')
            ]),
            launch_arguments={
                'map': map_arg,
                'params_file': config_paths['nav2'],
                'use_sim_time': use_sim_time,
            }.items(),
        ),
        # Nav2 Navigation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('nav2_bringup'),
                            'launch', 'navigation_launch.py')
            ]),
            launch_arguments={
                'params_file': config_paths['nav2'],
                'use_sim_time': use_sim_time,
            }.items(),
        ),
        # Web dashboard (rosbridge + browser UI)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('go2_dashboard'),
                            'launch', 'web_ui.launch.py')
            ]),
            condition=IfCondition(with_web_ui),
            launch_arguments={'web_dev': web_dev}.items(),
        ),
    ]
    
    return LaunchDescription(
        launch_args +
        core_nodes +
        teleop_nodes +
        viz_nodes +
        include_launches
    )
