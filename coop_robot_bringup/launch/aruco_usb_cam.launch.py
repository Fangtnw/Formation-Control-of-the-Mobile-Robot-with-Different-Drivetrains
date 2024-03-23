from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command,PathJoinSubstitution,LaunchConfiguration,PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription,DeclareLaunchArgument
from ament_index_python.packages import get_package_share_path
from launch.conditions import IfCondition

def generate_launch_description():


    camera= ExecuteProcess(
        cmd=['ros2', 'run', 'usb_cam', 'usb_cam_node_exe','--ros-args', '--params-file' ,'coop_ws/src/coop_robot_bringup/config/usb_cam_param.yaml'],
        output='screen',
    )

    aruco= ExecuteProcess(
        cmd=['ros2', 'run', 'aruco_usb_cam', 'aruco_usb_cam'],
        output='screen',
    )


    return LaunchDescription([
        camera,
        # spawn_ackermann_back,
        aruco,
        
    ])