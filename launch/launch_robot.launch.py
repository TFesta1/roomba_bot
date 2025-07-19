import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, TimerAction


from launch_ros.actions import Node

from launch.actions import SetEnvironmentVariable

from launch.substitutions import Command

from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart







def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!
    mute_audio = SetEnvironmentVariable('SDL_AUDIODRIVER', 'dummy')


    package_name='roomba_bot' 

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control': "true"}.items()
    )

    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    controller_params_file = os.path.join(get_package_share_directory(package_name),'config','my_controllers.yaml')


    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description},
                    controller_params_file]
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['diff_cont']
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager, # Start when the controller_manager starts
            on_start=[diff_drive_spawner], #When this exists, we run diff_drive_spawner
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['joint_broad']
    )

    joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager, # Start when the controller_manager starts
            on_start=[joint_broad_spawner], #When this exists, we run diff_drive_spawner
        )
    )

    
    # display = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         os.path.join(
    #             get_package_share_directory('urdf_tutorial'),
    #             'launch',
    #             'display.launch.py'
    #         )
    #     ]),
    #     launch_arguments={
    #         'model': '/home/tanner/dev_ws/src/roomba_bot/description/robot.urdf.xacro'
    #     }.items()
    # )




    # Launch them all!
    return LaunchDescription([
        mute_audio,
        rsp,
        delayed_controller_manager,
        delayed_diff_drive_spawner, #spawn_entity
        # display
        diff_drive_spawner,
        joint_broad_spawner
    ])