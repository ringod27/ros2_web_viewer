from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('websocket_bridge'),
        'config',
        'default_params.yaml'
    )

    node = Node(
        package='websocket_bridge',
        executable='websocket_bridge',
        name='websocket_bridge',
        parameters=[config_path],
        output='screen'
    )

    return LaunchDescription([node])