# cartographer_slam_launch.py

from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
# from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get paths to config files
    cartographer_config_file = os.path.join(get_package_share_directory('rs1_project'), 'config', 'turtlebot3_config_rs1.lua')
    # rviz_config_dir = os.path.join(get_package_share_directory('rs1_project'), 'rviz', 'mapping_rviz.rviz')

    # Launch Cartographer node for 2D SLAM
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{
            'use_sim_time': True  # Enable simulation time for Cartographer
        }],
        arguments=['-configuration_directory', os.path.dirname(cartographer_config_file),
                   '-configuration_basename', os.path.basename(cartographer_config_file)]
    )

    # Launch teleop node for controlling the robot manually
    # teleop_node = Node(
    #     package='teleop_twist_keyboard',
    #     executable='teleop_twist_keyboard',
    #     name='teleop_twist_keyboard',
    #     output='screen'
    # )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        # arguments=['-d', rviz_config_dir],
        output='screen'
    )

    return LaunchDescription([
        cartographer_node,
        # teleop_node,
        rviz_node
    ])
