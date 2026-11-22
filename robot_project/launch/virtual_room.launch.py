#!/usr/bin/env python3
"""
Launch file for the Robotics Laboratory simulation.

Features:
- Spawns the enhanced realistic robot model onto the green start pad.
- High timeout (180s) and graceful delay to ensure Gazebo is fully ready.
- Automatically sets GAZEBO_MODEL_PATH so all meshes load seamlessly.
- Configurable initial pose and sim time.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    # ── Paths ──
    current_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.abspath(os.path.join(current_dir, '..'))
    world_path = os.path.join(project_root, 'worlds', 'robotics_lab.world')
    models_path = os.path.join(project_root, 'models')

    # Enhanced realistic robot model SDF
    enhanced_sdf = os.path.join(
        models_path, 'turtlebot3_burger_enhanced', 'model.sdf'
    )

    # ── Launch configurations ──
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-1.0')
    y_pose = LaunchConfiguration('y_pose', default='-1.0')

    # Ensure GAZEBO_MODEL_PATH includes our custom models and turtlebot3 models
    tb3_models = os.path.join(pkg_turtlebot3_gazebo, 'models')
    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    new_model_path = f"{models_path}:{tb3_models}:{existing_model_path}".strip(':')

    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=new_model_path,
    )

    # 1. Gazebo Server with the robotics_lab world
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_path}.items(),
    )

    # 2. Gazebo Client GUI
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
    )

    # 3. Robot State Publisher (publishes URDF -> TF)
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # 4. Spawner: Spawns the enhanced realistic robot with 180s timeout so it NEVER dies
    spawn_robot_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'turtlebot3_burger',
            '-file', enhanced_sdf,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01',
            '-timeout', '180.0',
        ],
        output='screen',
    )

    # Delay spawn by 2 seconds to let gzserver initialize its ROS services
    delayed_spawn_cmd = TimerAction(
        period=2.0,
        actions=[spawn_robot_node],
    )

    ld = LaunchDescription()
    ld.add_action(set_model_path)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(delayed_spawn_cmd)

    return ld
