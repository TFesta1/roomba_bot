#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro

def generate_launch_description():
    pkg_share = get_package_share_directory('roomba_bot')

    # Launch arguments
    use_sim_time        = LaunchConfiguration('use_sim_time')
    url                 = LaunchConfiguration('url')
    frame_id            = LaunchConfiguration('frame_id')
    fps                 = LaunchConfiguration('fps')
    camera_info_url     = LaunchConfiguration('camera_info_url')
    model_path          = LaunchConfiguration('model_path')
    midas_input_topic   = LaunchConfiguration('midas_input_topic')
    midas_output_topic  = LaunchConfiguration('midas_output_topic')
    image_topic         = LaunchConfiguration('image_topic')
    compressed_topic    = LaunchConfiguration('compressed_topic')
    info_topic          = LaunchConfiguration('info_topic')
    scan_topic          = LaunchConfiguration('scan_topic')
    scan_frame_id       = LaunchConfiguration('scan_frame_id')
    min_range           = LaunchConfiguration('min_range')
    max_range           = LaunchConfiguration('max_range')

    # Process the robot URDF
    xacro_file = os.path.join(pkg_share, 'description', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)

    # Nodes
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_config.toxml(),
            'use_sim_time': use_sim_time
        }]
    )

    """
    ros2 run roomba_bot esp32_cam_http.py \
    --ros-args \
        -p url:=http://192.168.0.84/cam-hi.jpg \
        -p frame_id:=camera_frame \
        -p fps:=3.0 \
        -p camera_info_url:=file:///home/tanner/dev_ws/src/roomba_bot/calib/esp32_cam.yaml

    """
    node_esp32_cam = Node(
        package='roomba_bot',
        executable='esp32_cam_http.py',
        name='esp32_cam_http',
        output='screen',
        parameters=[{
            'url': url,
            'frame_id': frame_id,
            'fps': fps,
            'camera_info_url': camera_info_url,
            'use_sim_time': use_sim_time
        }]
    )

    """
    ros2 run roomba_bot midas_depth.py \
    --ros-args \
        -p model_path:=/home/tanner/dev_ws/src/roomba_bot/models/midas_small.onnx \
        -p input_topic:=/camera/image_raw \
        -p output_topic:=/camera/depth/image_raw

    """

    node_midas = Node(
        package='roomba_bot',
        executable='midas_depth.py',
        name='midas_depth',
        output='screen',
        parameters=[{
            'model_path': model_path,
            'input_topic': midas_input_topic,
            'output_topic': midas_output_topic,
            'use_sim_time': use_sim_time
        }]
    )

    """
    ros2 run roomba_bot cam_to_laserscan.py \
    --ros-args \
        -p image_topic:=/camera/image_raw \
        -p info_topic:=/camera/camera_info \
        -p scan_topic:=/scan \
        -p frame_id:=base_link \
        -p min_range:=0.001 \
        -p max_range:=5.0

    
    """

    node_cam2scan = Node(
        package='roomba_bot',
        executable='cam_to_laserscan.py',
        name='cam_to_laserscan',
        output='screen',
        parameters=[{
            'image_topic': image_topic,
            'info_topic': info_topic,
            'scan_topic': scan_topic,
            'frame_id': scan_frame_id,
            'min_range': min_range,
            'max_range': max_range,
            'use_sim_time': use_sim_time
        }]
    )
    """
    rviz2
    """
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    """
    ros2 run image_transport republish raw compressed \
        --ros-args \
            -r in:=/camera/image_raw \
            -r out:=/camera/compressed

    """

    republish_node = Node(
            package='image_transport',
            executable='republish',
            name='republish_raw_to_compressed',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['raw', 'compressed'],
            remappings=[
                ('in', image_topic),
                ('out', compressed_topic),
            ],
        )

    return LaunchDescription([
        # Arguments for simulation time
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock'),

        # ESP32-CAM node args
        DeclareLaunchArgument(
            'url',
            default_value='http://192.168.0.84/cam-hi.jpg',
            description='ESP32-CAM JPEG URL'),
        DeclareLaunchArgument(
            'frame_id',
            default_value='camera_frame',
            description='Frame ID for the camera images'),
        DeclareLaunchArgument(
            'fps',
            default_value='3.0',
            description='Frames per second to poll the camera'),
        DeclareLaunchArgument(
            'camera_info_url',
            default_value=(
                'file://'
                + os.path.join(
                    pkg_share, 'calib', 'esp32_cam.yaml'
                  )
            ),
            description='file:// URL to your camera_info YAML'),

        # MiDaS depth node args
        DeclareLaunchArgument(
            'model_path',
            default_value="/home/tanner/dev_ws/src/roomba_bot/models/midas_small.onnx",
            description='ONNX model path for MiDaS'),
        DeclareLaunchArgument(
            'midas_input_topic',
            default_value='/camera/image_raw',
            description='Input image topic for MiDaS'),
        DeclareLaunchArgument(
            'midas_output_topic',
            default_value='/camera/depth/image_raw',
            description='Output depth image topic from MiDaS'),

        # LaserScan converter node args
        DeclareLaunchArgument(
            'image_topic',
            default_value='/camera/image_raw',
            description='Input image topic for scanners'),
         DeclareLaunchArgument(
            'compressed_topic',
            default_value='/camera/compressed',
            description='Output compressed image topic for scanners'),
        DeclareLaunchArgument(
            'info_topic',
            default_value='/camera/camera_info',
            description='CameraInfo topic for scanners'),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan',
            description='Output LaserScan topic'),
        DeclareLaunchArgument(
            'scan_frame_id',
            default_value='base_link',
            description='Frame ID for LaserScan'),
        DeclareLaunchArgument(
            'min_range',
            default_value='0.001',
            description='Minimum range for LaserScan'),
        DeclareLaunchArgument(
            'max_range',
            default_value='5.0',
            description='Maximum range for LaserScan'),
        
        


        # Launch all nodes
        node_robot_state_publisher,
        node_esp32_cam,
        republish_node,
        node_midas,
        node_cam2scan,
        rviz_node,
    ])