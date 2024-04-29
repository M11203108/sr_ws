import os
from pathlib import Path
import launch
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import PushRosNamespace
import launch_ros.actions
from launch.conditions import UnlessCondition
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import Command

def generate_launch_description():

    bringup_dir = get_package_share_directory('control_robot') #分享目錄
    launch_dir = os.path.join(bringup_dir, 'launch') #launch目錄

    robot_description = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'robot_mode_description.launch.py')),
    )
    base_to_link = launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_link',
            arguments=['0', '0', '0','0', '0','0','base_footprint','base_link'],
    )
    base_to_gyro = launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_gyro',
            arguments=['0', '0', '0','0', '0','0','base_footprint','gyro_link'],
    )
    
    keyboard__robot_node = Node(
        package='control_robot',
        executable='keyboard_robot_node.py',
        name='keyboard_robot_node',
        output='screen'
    )


    joint_state_publisher_node = launch_ros.actions.Node(
            package='joint_state_publisher', 
            executable='joint_state_publisher', 
            name='joint_state_publisher',
    )

    



    return LaunchDescription([
        robot_description,
        base_to_link,
        base_to_gyro,
        joint_state_publisher_node,
        keyboard__robot_node
    ])