import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, TimerAction


from launch_ros.actions import Node

from launch.actions import SetEnvironmentVariable







def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!
    mute_audio = SetEnvironmentVariable('SDL_AUDIODRIVER', 'dummy')


    package_name='roomba_bot' #<--- CHANGE ME

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': "true"}.items()
    )
    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')
    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
        launch_arguments={'gui': 'true',
                          'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()  # or false for headless
    )


    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'roomba_bot'
                                   ],
                        output='screen')
    spawn_entity_delayed = TimerAction(
        period=0.0,# seconds
        actions=[spawn_entity]
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['diff_cont']
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['joint_broad']
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
        gazebo,
        spawn_entity_delayed, #spawn_entity
        # display
        diff_drive_spawner,
        joint_broad_spawner
    ])