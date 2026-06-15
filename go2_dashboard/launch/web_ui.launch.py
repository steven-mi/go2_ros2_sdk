# Web dashboard: rosbridge + static file server (prod) or Vite dev server (HMR)
# Usage: included by mapping/navigation launches, or standalone:
#   ros2 launch go2_dashboard web_ui.launch.py
# Dev with hot reload (requires Node.js + npm install in web/):
#   ros2 launch go2_dashboard web_ui.launch.py web_dev:=true
# Docker: source is mounted at /ros2_ws/src — web_dev uses that tree automatically.

import os
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _resolve_web_dirs(package_share: str) -> tuple[str, str]:
    """Return (dev_cwd, prod_serve_dir) for Vite / static HTTP server."""
    install_web = os.path.join(package_share, 'web')
    src_web = os.environ.get('GO2_DASHBOARD_WEB_SRC', '/ros2_ws/src/go2_dashboard/web')

    dev_web = (
        src_web
        if os.path.isfile(os.path.join(src_web, 'package.json'))
        else install_web
    )

    serve_candidates = [
        install_web,
        os.path.join(install_web, 'dist'),
        os.path.join(src_web, 'dist'),
        src_web,
    ]
    prod_serve = install_web
    for candidate in serve_candidates:
        if candidate and os.path.isfile(os.path.join(candidate, 'index.html')):
            prod_serve = candidate
            break

    return dev_web, prod_serve


def generate_launch_description():
    web_ui = LaunchConfiguration('web_ui', default='true')
    web_dev = LaunchConfiguration('web_dev', default='false')
    web_port = LaunchConfiguration('web_port', default='8080')
    dev_port = LaunchConfiguration('dev_port', default='5173')
    rosbridge_port = LaunchConfiguration('rosbridge_port', default='9090')
    with_camera = LaunchConfiguration('camera', default='true')

    dashboard_dir = get_package_share_directory('go2_dashboard')
    web_dev_dir, serve_dir = _resolve_web_dirs(dashboard_dir)
    npm = shutil.which('npm')
    use_vite = bool(npm)

    rosbridge_launch = os.path.join(
        get_package_share_directory('rosbridge_server'),
        'launch',
        'rosbridge_websocket_launch.xml',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'web_ui',
            default_value='true',
            description='Launch web dashboard (rosbridge + HTTP server)',
        ),
        DeclareLaunchArgument(
            'web_dev',
            default_value='true',
            description='Run Vite dev server with hot reload (set false for static build on web_port)',
        ),
        DeclareLaunchArgument(
            'web_port',
            default_value='8080',
            description='Port for production static HTTP server (web/dist)',
        ),
        DeclareLaunchArgument(
            'dev_port',
            default_value='5173',
            description='Port for Vite dev server when web_dev:=true',
        ),
        DeclareLaunchArgument(
            'rosbridge_port',
            default_value='9090',
            description='Port for rosbridge WebSocket',
        ),
        DeclareLaunchArgument(
            'camera',
            default_value='true',
            description='Republish camera as compressed for the web UI',
        ),

        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(rosbridge_launch),
            condition=IfCondition(web_ui),
            launch_arguments={
                'port': rosbridge_port,
                'address': '0.0.0.0',
            }.items(),
        ),

        Node(
            package='image_transport',
            executable='republish',
            name='dashboard_image_republisher',
            condition=IfCondition(with_camera),
            arguments=['raw', 'compressed'],
            remappings=[
                ('in', 'camera/image_raw'),
                ('out/compressed', 'camera/compressed'),
            ],
            parameters=[{
                'qos_overrides': {
                    '/camera/image_raw': {
                        'subscription': {
                            'reliability': 'best_effort',
                            'history': 'keep_last',
                            'depth': 1,
                        },
                    },
                },
            }],
        ),

        ExecuteProcess(
            condition=IfCondition(
                PythonExpression([
                    "'", web_ui, "' == 'true' and ('", web_dev, "' != 'true' or not ",
                    str(use_vite), ")",
                ]),
            ),
            cmd=[
                'python3', '-m', 'http.server',
                web_port,
                '--bind', '0.0.0.0',
            ],
            cwd=serve_dir,
            output='screen',
            additional_env={'PYTHONUNBUFFERED': '1'},
        ),

        ExecuteProcess(
            condition=IfCondition(
                PythonExpression([
                    "'", web_ui, "' == 'true' and '", web_dev, "' == 'true' and ",
                    str(use_vite),
                ]),
            ),
            cmd=[npm, 'run', 'dev', '--', '--host', '0.0.0.0', '--port', dev_port],
            cwd=web_dev_dir,
            output='screen',
            additional_env={'PYTHONUNBUFFERED': '1'},
        ),
    ])
