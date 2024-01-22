import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    diffbot_bringup_dir = get_package_share_directory("coop_robot_bringup")

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration("use_sim_time")

    # declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument("use_sim_time")

    ydliar = ExecuteProcess(
        cmd=['ros2','launch','ydlidar_ros2_driver','ydlidar_launch.py']

    )

    xicro = ExecuteProcess(
        cmd=['ros2', 'run', 'xicro_pkg', 'xicro_node_diffdrive_ID_1_arduino.py'],
        output='screen',
    )

    odom_compute = Node(
        package='coop_controller',
        executable='diffdrive_odom',
        output='screen',
    )

    robot_localization_odom = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_localization_odom",
        output="screen",
        parameters=[
            os.path.join(diffbot_bringup_dir, "config", "ekf.yaml"),
        ],
        remappings=[('odometry/filtered', 'odom')],
        arguments=['--ros-args', '--params-file', os.path.join(diffbot_bringup_dir, "config", "ekf.yaml")],
    )

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

    ld = LaunchDescription()

    # Add actions to the LaunchDescription
    # ld.add_action(ydliar)
    # ld.add_action(xicro)
    ld.add_action(odom_compute)
    # ld.add_action(robot_localization_odom)
    ld.add_action(rviz)
    ld.add_action(slam_toolbox)
    ld.add_action(nav2)

    return ld

if __name__ == "__main__":
    generate_launch_description()
