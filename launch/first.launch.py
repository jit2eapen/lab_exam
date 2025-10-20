#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import (
    TextSubstitution,
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # package share
    pkg_robot = get_package_share_directory('fourth')

    # --- Declare launch configurations / arguments (first) ---
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    robot_name = LaunchConfiguration('robot_name', default='fi')
    world_file = LaunchConfiguration('world_file', default='empty.sdf')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation (Ignition) clock if true'
    )
    declare_world_file = DeclareLaunchArgument(
        'world_file', default_value='empty.sdf', description='World file name inside fourth/worlds/'
    )
    declare_robot_name = DeclareLaunchArgument(
        'robot_name', default_value='fi', description='Name to give the spawned robot'
    )

    # --- Configure environment so Ignition can find worlds/models in the package ---
    pkg_worlds = os.path.join(pkg_robot, 'worlds')
    pkg_models = os.path.join(pkg_robot, 'models')

    set_ign_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            TextSubstitution(text=pkg_worlds + os.pathsep + pkg_models + os.pathsep),
            EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value='')
        ]
    )

    # --- URDF robot_description ---
    urdf_file = os.path.join(pkg_robot, 'urdf', 'box.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # --- World path ---
    world_path = PathJoinSubstitution([TextSubstitution(text=pkg_robot), 'worlds', world_file])

    # --- Include Ignition / ros_gz_sim launch ---
    ignition_launch_pkg = get_package_share_directory('ros_gz_sim')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ignition_launch_pkg, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': [TextSubstitution(text='-r -v4 '), world_path],
        }.items()
    )

    # --- Nodes ---
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc,   # ✅ fixed here
        }]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Spawn robot into Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        output='screen',
        arguments=[
            '-name', robot_name,
            '-param', 'robot_description',
            '-allow_renaming', 'true'
        ]
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/tf_static@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # --- Compose launch description ---
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_world_file)
    ld.add_action(declare_robot_name)
    ld.add_action(set_ign_resource_path)
    ld.add_action(gazebo_launch)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    ld.add_action(spawn_robot)
    ld.add_action(bridge)
    ld.add_action(rviz_node)

    return ld

