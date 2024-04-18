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
        default_value='back',  # Default value is 'front'
        description='Specify the pose of the ackermann vehicle (front/back)'
    )
    xacro_mec=os.path.join(get_package_share_path('my_robot_description'),
                           'urdf','mecanum_fork.xacro')
    
    rviz_config_path=os.path.join(get_package_share_path('my_robot_description'),
                           'rviz','urdf_config.rviz')
    
    mec_description=xacro.process_file(xacro_mec).toxml()

    
    mecanum_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description':mec_description} , {'use_sim_time': True}],
        # remappings=[('/robot_description', '/mecanum_description')]
        namespace="mecanum",
        output='screen'
    )

    spawn_mecanum= Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "/mecanum/robot_description",
            "-entity", "mecanum",
            "-x", "-0.1",   # Example: Set x-coordinate to 1.0
            "-y", "-1.5",   # Example: Set y-coordinate to 2.0
            "-z", "0.0",   # Example: Set z-coordinate to 0.0
            "-Y"," 0.0",
        ],
        namespace="mecanum",
        output='screen',
        condition=IfCondition(PythonExpression(['"', LaunchConfiguration('pose'), '" == "ack_front" or "', LaunchConfiguration('pose'), '" == "diff_front"']))
    )

    spawn_mecanum_front= Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "/mecanum/robot_description",
            "-entity", "mecanum",
            "-x", "1.8",   # Example: Set x-coordinate to 1.0
            "-y", "-1.5",   # Example: Set y-coordinate to 2.0
            "-z", "0.0",   # Example: Set z-coordinate to 0.0
            "-Y","-3.14159265359",
        ],
        namespace="mecanum",
        output='screen',
        condition=IfCondition(PythonExpression(['"', LaunchConfiguration('pose'), '" == "ack_back" or "', LaunchConfiguration('pose'), '" == "diff_back"']))
    )

    return LaunchDescription([
        pose_arg,
        mecanum_state_publisher,
        spawn_mecanum,
        spawn_mecanum_front
    ])