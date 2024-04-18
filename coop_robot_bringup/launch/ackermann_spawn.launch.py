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
    pose_arg = DeclareLaunchArgument(
        'pose',
        default_value='front',  # Default value is 'front'
        description='Specify the pose of the ackermann vehicle (front/back)'
    )

    xacro_ack=os.path.join(get_package_share_path('my_robot_description'),
                           'urdf','ackermann_fork.xacro')
    
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

    spawn_ackermann_front = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "/ackermann/robot_description",
            "-entity", "ackermann",
            "-x", "1.8",
            "-y", "-1.5",
            "-z", "0.0",
            "-Y", "-3.14159265359",
        ],
        namespace="ackermann",
        output='screen',
        condition=IfCondition(PythonExpression(['"', LaunchConfiguration('pose'), '" == "ack_front"']))
    )

    spawn_ackermann_back = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "/ackermann/robot_description",
            "-entity", "ackermann",
            "-x", "-0.1",
            "-y", "-1.5",
            "-z", "0.0",
            "-Y", "0.0",
        ],
        namespace="ackermann",
        output='screen',
        condition=IfCondition(PythonExpression(['"', LaunchConfiguration('pose'), '" == "ack_back"']))
    )

    load_ackermann_controller = Node(
        package="controller_manager",
        executable="spawner",
        namespace="ackermann",
        arguments=["ackermann_steering_controller", "--controller-manager", "controller_manager"]
    )
    
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        namespace="ackermann",
        arguments=["joint_state_broadcaster", "--controller-manager", "controller_manager"]
    )
    load_position_controller = Node(
        package="controller_manager",
        executable="spawner",
        namespace="ackermann",
        arguments=["position_controller", "--controller-manager", "controller_manager"]
    )
    load_velocity_controller = Node(
        package="controller_manager",
        executable="spawner",
        namespace="ackermann",
        arguments=["velocity_controller", "--controller-manager", "controller_manager"]
    )


    return LaunchDescription([
        pose_arg,
        ack_state_publisher,
        spawn_ackermann_front,
        spawn_ackermann_back
    ])