import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import math

angle = -math.pi

# Calculate half angle for quaternion construction
half_angle = angle / 2.0

# Generate sine and cosine values
sin_half_angle = math.sin(half_angle)
cos_half_angle = math.cos(half_angle)

def generate_launch_description():
    diffbot_bringup_dir = get_package_share_directory("coop_robot_bringup")

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration("use_sim_time")

    # declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument("use_sim_time")

    ydliar = ExecuteProcess(
        cmd=['ros2','launch','ydlidar_ros2_driver','ydlidar_launch.py']

    )

    # xicro = Node(
    #     package='xicro_pkg',
    #     executable='xicro_node_mec_xicro_ID_3_arduino.py',
    #     output='screen',
    #     # ros_arguments=['--remap','__ns:=/mecanum',],
    
    # )

    xicro = ExecuteProcess(
        cmd=['ros2', 'run', 'xicro_pkg', 'xicro_node_mec_xicro_ID_3_arduino.py'],
        output='screen',
    )

    odom_compute = Node(
        package='coop_controller',
        executable='mec_odom',
        output='screen',
        # ros_arguments=['--remap','__ns:=/mecanum',],
    )

    robot_localization_odom = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            remappings=[('/odometry/filtered', '/mecanum/odom')],
            ros_arguments=['--params-file','coop_ws/src/coop_robot_bringup/config/ekf_mec.yaml'],  # Replace with the actual path
        )
    
    imu_to_base_link_tf = ExecuteProcess(
        cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '0', '0', '0', '0', '0', '0', 'Imu_mec', 'base_link_mec'],
        output='screen',
    )

    # robot_localization_odom = Node(
    #     package="robot_localization",
    #     executable="ekf_node",
    #     name="ekf_localization_odom",
    #     output="screen",
    #     parameters=[
    #         os.path.join(diffbot_bringup_dir, "config", "ekf.yaml"),
    #     ],
    #     remappings=[('odometry/filtered', 'odom')],
    #     arguments=['--ros-args', '--params-file', os.path.join(diffbot_bringup_dir, "config", "ekf.yaml")],
    # )

    rviz = ExecuteProcess(
        cmd=['ros2', 'run', 'rviz2', 'rviz2', '-d', '/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz'],
        output='screen',
    )

    slam_toolbox = ExecuteProcess(
        cmd=['ros2', 'launch', 'slam_toolbox', 'online_async_launch.py'],
        output='screen',
    )

    nav2 = ExecuteProcess(
        cmd=['ros2', 'launch', 'nav2_bringup', 'navigation_launch.py'],
        output='screen',
    )

    # nav2 = ExecuteProcess(
    #     cmd=['ros2', 'launch', 'nav2_bringup', 'navigation_launch.py', 'params_file:=coop_ws/src/coop_robot_bringup/config/nav2_params.yaml'],
    #     output='screen',
    # )

    ack_to_mec_tf = Node(package='tf2_ros',
                        executable='static_transform_publisher',
                        name='static_tf_pub_imu',
                        output='screen',
                        arguments=[
                            '1.65', '0.0', '0.0',  # Set translation to zero (no translation)
                            '0.0', '0.0', str(sin_half_angle), str(cos_half_angle),  # Set rotation using quaternion
                            'odom', 'odom_mec'  # Parent and child frame IDs
                        ])
    
    ld = LaunchDescription()

    # Add actions to the LaunchDescription
    # ld.add_action(ydliar) 
    # ld.add_action(xicro)
    ld.add_action(odom_compute)
    # ld.add_action(imu_to_base_link_tf)
    ld.add_action(ack_to_mec_tf)
    # ld.add_action(robot_localization_odom)
    # ld.add_action(rviz)
    # ld.add_action(slam_toolbox)
    # ld.add_action(nav2)

    return ld

if __name__ == "__main__":
    generate_launch_description()
