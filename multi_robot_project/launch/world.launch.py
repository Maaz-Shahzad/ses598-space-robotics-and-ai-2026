#!/usr/bin/env python3
#!~/multi_robot_project/venv python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, TimerAction, RegisterEventHandler, EmitEvent, LogInfo
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    package_name = 'multi_robot_project'
#    world_name = 'empty'
    world_name = 'shapes_world'
    
    world_path = os.path.join(
    get_package_share_directory('multi_robot_project'),
    'worlds',
    'shapes_world.sdf'
    )

    # 1. Paths
    pkg_share = get_package_share_directory(package_name)
    # Ensure this matches your ACTUAL folder structure from the ls command earlier
    xacro_file1 = os.path.join(pkg_share, 'models', 'diffDriveBot1', 'model_urdf.xacro')
    xacro_file2 = os.path.join(pkg_share, 'models', 'diffDriveBot2', 'model_urdf.xacro')
    
    # 2. Process Xacro
    robot_description_raw1 = xacro.process_file(xacro_file1).toxml()
    robot_description_raw2 = xacro.process_file(xacro_file2).toxml()

    # 3. Robot State Publisher
    node_robot_state_publisher1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        namespace='robot1',
        parameters=[{
            'robot_description': robot_description_raw1,
            'use_sim_time': True
        }],
    )

    node_robot_state_publisher2 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        namespace='robot2',    
        parameters=[{
            'robot_description': robot_description_raw2,
            'use_sim_time': True
        }],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
#        launch_arguments={'gz_args': f'{world_path}'}.items(),
    )

    # 5. Spawn Robot (Using 'create' instead of 'spawn_entity')
    spawn_robot1 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-string', robot_description_raw1,
            '-name', 'robot1',
            '-x', '-1.5',
            '-y', '-3.0',
            '-z', '0.1',
            '-Y', '1.57'
        ]
    )

    spawn_robot2 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-string', robot_description_raw2,
            '-name', 'robot2',
            '-x', '1.5',
            '-y', '-3.0',
            '-z', '0.1',
            '-Y', '1.57'
        ]
    )

    robot1_controller = Node(
        package='multi_robot_project',
        executable='robot1_controller',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }]
    )
    robot2_controller = Node(
        package='multi_robot_project',
        executable='robot2_controller',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }]
    )

    robot1_camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # RGB Image bridge
            '/robot1/camera/image_raw/image@sensor_msgs/msg/Image[gz.msgs.Image',
            # Depth Image bridge
            '/robot1/camera/image_raw/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            # Camera Info bridge 
            '/robot1/camera/image_raw/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
#            '/robot1/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
            # Point cloud
#            '/robot1/camera/image_raw/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        remappings=[
            ('/robot1/camera/image_raw/image', '/robot1/camera/rgb'),
            ('/robot1/camera/image_raw/depth_image', '/robot1/camera/depth'),
            ('/robot1/camera/image_raw/camera_info', '/robot1/camera/camera_info'), 
#            ('/robot1/camera/camera_info', '/robot1/camera/camera_info'), 
#            ('/robot1/camera/image_raw/points', '/robot1/camera/points'), 
        ],
        output='screen'
    )

    robot2_camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # RGB Image bridge
            '/robot2/camera/image_raw/image@sensor_msgs/msg/Image[gz.msgs.Image',
            # Depth Image bridge
            '/robot2/camera/image_raw/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            # Camera Info bridge 
            '/robot2/camera/image_raw/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
#            '/robot2/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
            # Point cloud
#            '/robot2/camera/image_raw/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        remappings=[
            ('/robot2/camera/image_raw/image', '/robot2/camera/rgb'),
            ('/robot2/camera/image_raw/depth_image', '/robot2/camera/depth'),
            ('/robot2/camera/image_raw/camera_info', '/robot2/camera/camera_info'), 
#            ('/robot2/camera/camera_info', '/robot2/camera/camera_info'), 
#            ('/robot2/camera/image_raw/points', '/robot2/camera/points'), 
        ],
        output='screen'
    )

#    mapper_node = Node(
#            package='multi_robot_project',
#            executable='mapper_node',
#            name='multi_robot_mapper',
#            output='screen',
#            parameters=[{'use_sim_time': True}] 
#    )
#
    robot1_mapper_node = Node(
        package='multi_robot_project',
        executable='robot1_mapper_node',
        name='robot1_mapper',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }]
    )

    robot2_mapper_node = Node(
        package='multi_robot_project',
        executable='robot2_mapper_node',
        name='robot2_mapper',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }]
    )

    terminate_on_finish = RegisterEventHandler(
            OnProcessExit(
                target_action=robot1_mapper_node,
                on_exit=[
                    LogInfo(msg='Robot 1 reached goal. Shutting down simulation...'),
                    EmitEvent(event=Shutdown(reason='Mission Accomplished')) # This now refers to the Event
                ]
            )
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
#            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
            f'/world/{world_name}/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',

#            Robot 1
            '/model/robot1/joint_state@sensor_msgs/msg/JointState@gz.msgs.Model',
            '/model/robot1/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/model/robot1/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/model/robot1/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
#            Robot 2
            '/model/robot2/joint_state@sensor_msgs/msg/JointState@gz.msgs.Model',
            '/model/robot2/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/model/robot2/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/model/robot2/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
        ],
        remappings=[
            ('/model/robot1/odometry', '/robot1/odom'),
            ('/model/robot1/cmd_vel', '/robot1/cmd_vel'),
            ('/model/robot1/joint_state', '/robot1/joint_states'),

            ('/model/robot2/odometry', '/robot2/odom'),
            ('/model/robot2/cmd_vel', '/robot2/cmd_vel'),
            ('/model/robot2/joint_state', '/robot2/joint_states'),

            (f'/world/{world_name}/clock', '/clock')
        ],
        output='screen'
    )

    return LaunchDescription([
        node_robot_state_publisher1,
        node_robot_state_publisher2,
        gazebo,
#        TimerAction(
#            period=5.0,
#            actions=[spawn_robot1, spawn_robot2]
#        ),
        spawn_robot1,
        spawn_robot2,
        bridge,
        robot1_camera_bridge,
        robot2_camera_bridge,
#        TimerAction(
#            period=10.0,
#            actions=[robot1_controller, robot2_controller]
#        )
        robot1_controller,
        robot2_controller,
#        mapper_node,
        robot1_mapper_node,
        robot2_mapper_node,
        terminate_on_finish
    ])
