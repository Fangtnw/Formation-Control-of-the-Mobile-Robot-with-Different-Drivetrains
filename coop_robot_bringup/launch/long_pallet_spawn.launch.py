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

    xacro_pallet=os.path.join(get_package_share_path('my_robot_description'),
                           'urdf','long_pallet.xacro')
    
    rviz_config_path=os.path.join(get_package_share_path('my_robot_description'),
                           'rviz','urdf_config.rviz')
    
    pallet_description=xacro.process_file(xacro_pallet).toxml()

    
    pallet_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description':pallet_description} , {'use_sim_time': True}],
        # remappings=[('/robot_description', '/ackermann_description')]
        namespace="long_pallet",
        output='screen'
    )

    spawn_pallet= Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "/long_pallet/robot_description",
            "-entity", "long_pallet",
            "-x", "1.25",   # Example: Set x-coordinate to 1.0
            "-y", "1.0",   # Example: Set y-coordinate to 2.0
            "-z", "0.085",   # Example: Set z-coordinate to 0.0
            "-Y","1.57",
        ],
        namespace="long_pallet",
        output='screen'
    )


    return LaunchDescription([
        spawn_pallet,
        pallet_state_publisher,
        
    ])