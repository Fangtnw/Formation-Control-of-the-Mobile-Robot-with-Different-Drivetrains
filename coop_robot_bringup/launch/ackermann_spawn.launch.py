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
import xacro

def generate_launch_description():

    xacro_ack=os.path.join(get_package_share_path('my_robot_description'),
                           'urdf','ackermann_basic.xacro')
    
    rviz_config_path=os.path.join(get_package_share_path('my_robot_description'),
                           'rviz','urdf_config.rviz')
    
    ack_description=xacro.process_file(xacro_ack).toxml()

    
    ack_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description':ack_description} , {'use_sim_time': True}],
        # remappings=[('/robot_description', '/ackermann_description')]
        namespace="ackermann",
        output='screen'
    )

    spawn_ackermann= Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "/ackermann/robot_description",
            "-entity", "ackermann",
            "-x", "1.25",   # Example: Set x-coordinate to 1.0
            "-y", "2",   # Example: Set y-coordinate to 2.0
            "-z", "0.0",   # Example: Set z-coordinate to 0.0
            "-Y","-1.57",
        ],
        namespace="ackermann",
        output='screen'
    )

    return LaunchDescription([
        spawn_ackermann,
        ack_state_publisher,
        
    ])