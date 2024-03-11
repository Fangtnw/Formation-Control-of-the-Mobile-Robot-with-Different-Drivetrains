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

    rviz_config_path=os.path.join(get_package_share_path('my_robot_description'),
                           'rviz','urdf_config.rviz')

    diff_description = ParameterValue(Command(['xacro ',xacro_diff]),value_type=str)
    
    gazebo_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])
        ])
    )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        launch_arguments={'world': get_package_share_directory('my_robot_description') + '/urdf/workshop_test_2.world','verbose': 'false'}.items(),
    )   

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ]),
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

    spawn_diffdrive= IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('coop_robot_bringup'),
                'launch',
                'diffdrive_spawn.launch.py'
            ])
        ]),
    )   

    spawn_mecanum= IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('coop_robot_bringup'),
                'launch',
                'mecanum_spawn.launch.py'
            ])
        ]),
    )   

    spawn_ackermann= IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('coop_robot_bringup'),
                'launch',
                'ackermann_spawn.launch.py'
            ])
        ]),
    )   

        
    tf_ack_to_mec= Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="diff_to_mec_tf_publisher",
            output="screen",
            arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "odom", "mec_odom"]
        )
    
    tf_ack_to_diff= Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="ack_to_mec_tf_publisher",
            output="screen",
            arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "odom", "diff_odom"]
        )

    return LaunchDescription([
        gazebo_server,
        gazebo_client,
        # joint_state_publisher_gui,
        spawn_ackermann,
   
        spawn_diffdrive,
        # spawn_mecanum,
        # tf_ack_to_mec,

        tf_ack_to_diff,
        rviz2,
    ])