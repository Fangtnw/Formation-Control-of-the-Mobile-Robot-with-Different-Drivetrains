import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration,PythonExpression
from launch_ros.actions import PushRosNamespace
from launch.conditions import IfCondition

def generate_launch_description():
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='sim',  # Default value is 'front'
        description='Specify the mode of the SLAM and Nav2'
    )
    diffbot_bringup_dir = get_package_share_directory("coop_robot_bringup")

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration("use_sim_time")

    # declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument("use_sim_time")

    ydliar = ExecuteProcess(
        cmd=['ros2','launch','ydlidar_ros2_driver','ydlidar_launch.py']

    )

    xicro = ExecuteProcess(
        cmd=['ros2', 'run', 'xicro_pkg', 'xicro_node_ack_xicro_ID_2_arduino.py '],
        output='screen',
    )

    diff_odom_compute = Node(
        package='coop_controller',
        executable='diffdrive_odom',
        condition=IfCondition(PythonExpression(['"', LaunchConfiguration('mode'), '" == "real"'])
                    ),
        output='screen',
    )

    ack_odom_compute = Node(
        package='coop_controller',
        executable='ack_odom',
        output='screen',
        condition=IfCondition(PythonExpression(['"', LaunchConfiguration('mode'), '" == "real"'])
                    )
    )
    mec_bringup = ExecuteProcess(
        cmd=['ros2','launch','coop_robot_bringup','mec.launch.py']
    )

    diff_bringup = ExecuteProcess(
        cmd=['ros2','launch','coop_robot_bringup','pi_diff.launch.py']
    )
    ekf_odom = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            # remappings=[('/odometry/filtered', '/diffdrive/odom')],
            ros_arguments=['--params-file','coop_ws/src/coop_robot_bringup/config/ekf.yaml'],  # Replace with the actual path
        )
    
    # imu_to_base_link_tf = ExecuteProcess(
    #     cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '0', '0', '0', '0', '0', '0', 'Imu', 'base_link'],
    #     output='screen',
    # )

    imu_to_base_link_tf = Node(package='tf2_ros',
                    executable='static_transform_publisher',
                    name='static_tf_pub_imu',
                    output='screen',
                    arguments=['0', '0', '0','0', '0', '0', '0','Imu_ack','base_footprint'],
                    )
    
    ack_to_mec_tf = Node(package='tf2_ros',
                    executable='static_transform_publisher',
                    name='static_tf_pub_imu',
                    output='screen',
                    arguments=['-1.5', '0', '0','0', '0', '0', '0','base_footprint','base_footprint_mec'],
                    )
    
    diff_footprint_to_base_tf = Node(package='tf2_ros',
                    executable='static_transform_publisher',
                    name='static_tf_pub_laser',
                    output='screen',
                    arguments=['0', '0', '0','0', '0', '3.14','base_footprint_diff','base_link_diff'],
                    )
    
    laser_to_base_footprint_tf = Node(package='tf2_ros',
                    executable='static_transform_publisher',
                    name='static_tf_pub_laser',
                    output='screen',
                    arguments=['-0.025', '0', '0','0', '0', '0', '1','base_footprint','laser_frame'],
                    condition=IfCondition(PythonExpression(['"', LaunchConfiguration('mode'), '" == "real"'])
                    )
                    )
    
    camera_to_mec_footprint = Node(package='tf2_ros',
                    executable='static_transform_publisher',
                    name='static_tf_pub_laser',
                    output='screen',
                    arguments=['0', '0', '0','0', '0', '0', '1','camera_frame','base_footprint_mec'],
                    condition=IfCondition(PythonExpression(['"', LaunchConfiguration('mode'), '" == "real"'])
                    )
                    )

    aruco_from_base_footprint = Node(package='tf2_ros',
                    executable='static_transform_publisher',
                    name='static_tf_pub_laser',
                    output='screen',
                    arguments=['0', '0', '0','0', '0', '0', '1','base_footprint','aruco_frame'],
                    condition=IfCondition(PythonExpression(['"', LaunchConfiguration('mode'), '" == "real"'])
                    )
                    )

    rviz = ExecuteProcess(
        cmd=['ros2', 'run', 'rviz2', 'rviz2', '-d', 'coop_view.rviz'],   #/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
        output='screen',
    )



    slam_toolbox_localize1 = ExecuteProcess(
        cmd=['ros2', 'launch', 'slam_toolbox', 'online_async_launch.py',
             'slam_params_file:=coop_ws/src/coop_robot_bringup/config/sim_mapper_params_online_async.yaml','use_sim_time:=true'],
        output='screen',
        condition=IfCondition(PythonExpression(['"', LaunchConfiguration('mode'), '" == "sim1"'])
                    )
    )

    slam_toolbox_localize2 = ExecuteProcess(
        cmd=['ros2', 'launch', 'slam_toolbox', 'online_async_launch.py',
             'slam_params_file:=coop_ws/src/coop_robot_bringup/config/sim2_mapper_params_online_async.yaml','use_sim_time:=True'],
        output='screen',
        condition=IfCondition(PythonExpression(['"', LaunchConfiguration('mode'), '" == "sim2"'])
                    )
    )

    slam_toolbox_localize_real = ExecuteProcess(
        cmd=['ros2', 'launch', 'slam_toolbox', 'online_async_launch.py',
             'slam_params_file:=coop_ws/src/coop_robot_bringup/config/localize_mapper_params_online_async.yaml','use_sim_time:=true'],
        output='screen',
        condition=IfCondition(PythonExpression(['"', LaunchConfiguration('mode'), '" == "real"'])
                    )
    )

    slam_toolbox= ExecuteProcess(
        cmd=['ros2', 'launch', 'slam_toolbox', 'online_async_launch.py',
             'slam_params_file:=coop_ws/src/coop_robot_bringup/config/mapper_params_online_async.yaml','use_sim_time:=true'],
        output='screen',
        condition=IfCondition(PythonExpression(['"', LaunchConfiguration('mode'), '" == "real"'])
                    )
    )


    nav2= ExecuteProcess(
        cmd=['ros2', 'launch', 'nav2_bringup', 'navigation_launch.py', 'params_file:=coop_ws/src/coop_robot_bringup/config/nav2_params.yaml' ,'use_sim_time:=false'],
        output='screen',
    )

    nav2_sim= ExecuteProcess(
        cmd=['ros2', 'launch', 'nav2_bringup', 'navigation_launch.py', 'params_file:=coop_ws/src/coop_robot_bringup/config/nav2_params.yaml', 'use_sim_time:=true'],
        output='screen',

    )


    map_server = ExecuteProcess(
        cmd=['ros2', 'run', 'nav2_map_server', 'map_server', '--ros-args', '-p', 'yaml_filename:=workshop_real.yaml'],   #real wide2
        output='screen',
        condition=IfCondition(PythonExpression(['"', LaunchConfiguration('mode'), '" == "real"']))
    )

    lifecycle_map_server = ExecuteProcess(
        cmd=['ros2', 'run', 'nav2_util', 'lifecycle_bringup', 'map_server'],
        output='screen',
        condition=IfCondition(PythonExpression(['"', LaunchConfiguration('mode'), '" == "real"']))
    )

    # amcl = ExecuteProcess(
    #     cmd=['ros2', 'run', 'nav2_amcl', 'amcl', '--params_file','coop_ws/src/coop_robot_bringup/config/nav2_params.yaml'],
    #     output='screen',
    #     condition=IfCondition(PythonExpression(['"', LaunchConfiguration('mode'), '" == "real"']))
    # )

    amcl =Node(
        package='nav2_amcl',
        executable='amcl',
        # cmd=['ros2', 'run', 'nav2_amcl', 'amcl', '--params_file','coop_ws/src/coop_robot_bringup/config/nav2_params.yaml'],
        output='screen',
        parameters=['coop_ws/src/coop_robot_bringup/config/nav2_params.yaml'],
        condition=IfCondition(PythonExpression(['"', LaunchConfiguration('mode'), '" == "real"']))
    )
    # nav2= ExecuteProcess(
    #     cmd=['ros2', 'launch', 'nav2_bringup', 'navigation_launch.py', 'params_file:=coop_ws/src/coop_robot_bringup/config/nav2_params.yaml' ,'use_sim_time:=false'],
    #     output='screen',
    # )

    
    lifecycle_amcl = ExecuteProcess(
        cmd=['ros2', 'run', 'nav2_util', 'lifecycle_bringup', 'amcl'],
        output='screen',
        condition=IfCondition(PythonExpression(['"', LaunchConfiguration('mode'), '" == "real"']))
    )

    ld = LaunchDescription()

    # Add actions to the LaunchDescription
    ld.add_action(mode_arg)
    # ld.add_action(ydliar)
    # ld.add_action(xicro)

    ld.add_action(laser_to_base_footprint_tf)
    # ld.add_action(diff_odom_compute)

    # ld.add_action(ack_to_mec_tf)
    ld.add_action(ack_odom_compute)
    # ld.add_action(ekf_odom)
    ld.add_action(camera_to_mec_footprint)
    ld.add_action(aruco_from_base_footprint)

    # ld.add_action(mec_bringup)
    # ld.add_action(diff_bringup)

    # ld.add_action(imu_to_base_link_tf)
    # ld.add_action(diff_robot_localization_odom)

    ld.add_action(rviz)

    # ld.add_action(slam_toolbox)
    # ld.add_action(nav2)
    
    # ld.add_action(slam_toolbox)
    # ld.add_action(nav2_sim)

    ld.add_action(map_server)
    ld.add_action(lifecycle_map_server)
    ld.add_action(slam_toolbox_localize1)
    ld.add_action(slam_toolbox_localize2)
    # ld.add_action(slam_toolbox_localize_real)
    ld.add_action(amcl)
    ld.add_action(lifecycle_amcl)

    return ld

if __name__ == "__main__":
    generate_launch_description()
