import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Get package paths
    pkg_share = get_package_share_directory('adaptive_pure_pursuit')
    #roar_simulation_share = get_package_share_directory('roar_simulation')

    # 2. Path to params.yaml
    config_file = os.path.join(pkg_share, 'config', 'params.yaml')

    # 3. Include the Mars Yard launch file
    # Assumption: You are migrating roar_simulation to 'mars_yard_24.launch.py'
    #If it is still a .launch file: <--- Change PythonLaunch... to XMLLaunch... at line 21
    #and import this: from launch_xml.launch_description_sources import XMLLaunchDescriptionSource 
    #mars_yard_launch = IncludeLaunchDescription(
     #   PythonLaunchDescriptionSource(
      #      os.path.join(roar_simulation_share, 'launch', 'mars_yard_24.launch.py')
       # )
    #)

    # 4. Define the Controller Node
    controller_node = Node(
        package='adaptive_pure_pursuit',
        executable='adaptiveLd_fixed',    # Must match setup.py entry point
        name='controller',
        output='screen',
        parameters=[config_file]     # Load the YAML file
    )

    # 5. Define the Path Test Node
    path_node = Node(
        package='adaptive_pure_pursuit',
        executable='path_test',      # Must match setup.py entry point!
        name='path',
        output='screen'
    )

    return LaunchDescription([
        #mars_yard_launch,
        controller_node,
        path_node
    ])