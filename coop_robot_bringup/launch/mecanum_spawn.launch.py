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


    xacro_mec=os.path.join(get_package_share_path('my_robot_description'),
                           'urdf','mecanum.xacro')
    
    rviz_config_path=os.path.join(get_package_share_path('my_robot_description'),
                           'rviz','urdf_config.rviz')

    mec_description = ParameterValue(Command(['xacro ',xacro_mec]),value_type=str)
    
    
    gazebo_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])
        ])
    )

    mecanum_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description':mec_description} , {'use_sim_time': True}]
    )


    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path]
    )
       

    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_footprint_base",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "base_footprint"]
    )

    spawn_mecanum= Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "mecanum"]
    )

    return LaunchDescription([
        # gazebo_launch_description,
        # static_transform_publisher,

        # ackermann_state_publisher,
        # spawn_ackermann,
        # joint_state_publisher_gui,
        # rviz2,

        mecanum_state_publisher,
        spawn_mecanum,
        
        # fake_joint_calibration
    ])