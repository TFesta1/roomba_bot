import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    SetEnvironmentVariable,
    IncludeLaunchDescription,
    TimerAction,
    RegisterEventHandler,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node

def generate_launch_description():
    pkg = 'roomba_bot'

    # 1) Mute SDL audio (avoids warning if Gazebo audio is enabled)
    mute_audio = SetEnvironmentVariable('SDL_AUDIODRIVER', 'dummy')

    # 2) Include the robot_state_publisher (our URDF → TF broadcaster)
    #    - pass use_sim_time=false (we’re on real hardware)
    #    - pass use_ros2_control=true so the xacro adds the ROS2 Control tags
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(pkg),
                'launch',
                'rsp.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'use_ros2_control': 'true'
        }.items()
    )

    # 3) Pull the robot_description parameter from rsp’s publisher
    robot_description = Command([
        'ros2 param get --hide-type /robot_state_publisher robot_description'
    ])

    # 4) Point to your controllers YAML
    controller_params = os.path.join(
        get_package_share_directory(pkg),
        'config',
        'my_controllers.yaml'
    )

    # 5) Launch the core ros2_control_node (controller_manager)
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controller_params
        ],
        output='screen'
    )

    # 6) Delay its start by a few seconds so that rsp has published first
    delayed_controller_manager = TimerAction(
        period=3.0,
        actions=[controller_manager]
    )

    # 7) Prepare the two spawner nodes (they will be started later)
    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont'],
        output='screen'
    )
    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad'],
        output='screen'
    )

    # 8) Use event handlers to start them only once the controller_manager is up
    spawn_diff_on_manager = RegisterEventHandler(
        OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner]
        )
    )
    spawn_joint_on_manager = RegisterEventHandler(
        OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_state_spawner]
        )
    )

    # 9) Assemble everything into the LaunchDescription
    return LaunchDescription([
        mute_audio,
        rsp,
        delayed_controller_manager,
        spawn_diff_on_manager,
        spawn_joint_on_manager,
    ])