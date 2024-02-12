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

    xacro_file=os.path.join(get_package_share_path('my_robot_description'),
                           'urdf','diffdrive.xacro')
    
    rviz_config_path=os.path.join(get_package_share_path('my_robot_description'),
                           'rviz','urdf_config.rviz')

    robot_description = ParameterValue(Command(['xacro ',xacro_file]),value_type=str)
    
    
    gazebo_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])
        ])
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description':robot_description} , {'use_sim_time': True}]
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path]
    )
       
    spawn_ackermann= Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "ackermann"]
    )
    static_transform_publisher_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_footprint_base",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "base_footprint"]
    )

    spawn_diffdrive= Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "diffdrive"]
    )

    fake_joint_calibration_node = Node(
        package='ros2topic',
        executable='ros2',
        name='fake_joint_calibration',
        arguments=['topic', 'pub', '/calibrated', 'std_msgs/Bool', 'true']
    )

    spawn_mecanum= Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "mecanum"]
    )

    return LaunchDescription([
        gazebo_launch_description,
        static_transform_publisher_node,
        robot_state_publisher_node,
        # joint_state_publisher_gui_node,
        rviz2_node,
        spawn_diffdrive,
        fake_joint_calibration_node
    ])