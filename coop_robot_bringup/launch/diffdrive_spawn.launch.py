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
    
    robot_description_raw = xacro.process_file(xacro_file).toxml()
    
    rviz_config_path=os.path.join(get_package_share_path('my_robot_description'),
                           'rviz','urdf_config.rviz')
    world_path = os.path.join(get_package_share_path('gazebo_ros'), 'worlds', 'empty.world')
    robot_description = ParameterValue(Command(['xacro ',xacro_file]),value_type=str)

    environment = 'worlds/car.world'
    environment_path = os.path.join(get_package_share_directory('coop_robot_bringup'),environment)  
    os.environ["GAZEBO_MODEL_PATH"] = environment_path
    
    headless = LaunchConfiguration('headless')
    use_simulator = LaunchConfiguration('use_simulator')
    world = LaunchConfiguration('world')

    declare_simulator_cmd = DeclareLaunchArgument(
        name='headless',
        default_value='False',
        description='Whether to execute gzclient')
        
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    declare_use_simulator_cmd = DeclareLaunchArgument(
        name='use_simulator',
        default_value='True',
        description='Whether to start the simulator')

    declare_world_cmd = DeclareLaunchArgument(
        name='world',
        default_value=environment_path,
        description='Full path to the world model file to load'
    )
    
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        condition=IfCondition(use_simulator),
        launch_arguments={'world': world}.items()
    )    
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ]),
        condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless]))
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

    gazebo_node = ExecuteProcess(
        cmd=['ros2', 'launch', 'gazebo_ros', 'gazebo.launch.py'],
        output='screen',
    )
       
    spawn_diffdrive= Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "diffdrive"]
    )

    return LaunchDescription([
        declare_simulator_cmd,
        declare_use_sim_time_cmd,
        declare_use_simulator_cmd,
        declare_world_cmd,
        gazebo_server,
        gazebo_client,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz2_node,
        # gazebo_node,
        spawn_diffdrive
    ])