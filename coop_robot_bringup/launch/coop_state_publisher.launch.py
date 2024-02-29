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
    xacro_diff=os.path.join(get_package_share_path('my_robot_description'),
                           'urdf','diffdrive.xacro')
    
    xacro_mec=os.path.join(get_package_share_path('my_robot_description'),
                           'urdf','mecanum.xacro')

    xacro_ack=os.path.join(get_package_share_path('my_robot_description'),
                           'urdf','ackermann_basic.xacro')
    
    mec_description=xacro.process_file(xacro_mec).toxml()

    diff_description=xacro.process_file(xacro_diff).toxml()

    ack_description=xacro.process_file(xacro_ack).toxml()
    
    diff_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description':diff_description} , {'use_sim_time': True}],
        namespace="diffdrive",
        output='screen',
    )
    
    mecanum_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description':mec_description} , {'use_sim_time': True}],
        # remappings=[('/robot_description', '/mecanum_description')]
        namespace="mecanum",
        output='screen'
    )
    
    ack_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description':ack_description} , {'use_sim_time': True}],
        # remappings=[('/robot_description', '/ackermann_description')]
        namespace="ackermann",
        output='screen'
    )

    return LaunchDescription([
        ack_state_publisher,
        diff_state_publisher,
        # joint_state_publisher_gui,
        mecanum_state_publisher,
   
        # spawn_diffdrive,
        # spawn_mecanum,
        # tf_ack_to_mec,
    ])
