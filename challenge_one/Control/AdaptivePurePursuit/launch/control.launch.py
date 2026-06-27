import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Find the path to your package configuration
    # This replaces "$(find adaptive_pure_pursuit)"
    pkg_share = get_package_share_directory('adaptive_pure_pursuit')
    
    # 2. Build the path to the params.yaml file
    config_file = os.path.join(pkg_share, 'config', 'params.yaml')

    # 3. Define the Node
    controller_node = Node(
        package='adaptive_pure_pursuit',
        executable='adaptiveLd_fixed',     # Must match the key in setup.py console_scripts
        name='controller',
        output='screen',
        parameters=[config_file]      # Load parameters from the YAML file
    )

    return LaunchDescription([
        controller_node
    ])