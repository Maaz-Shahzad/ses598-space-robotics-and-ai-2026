#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    package_name = 'multi_robot_project'
    world_name = 'empty'
    
    
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

    # 4. Gazebo Sim (Harmonic)
    # This launches the simulator itself
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # 5. Spawn Robot (Using 'create' instead of 'spawn_entity')
    spawn_robot1 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-string', robot_description_raw1,
            '-name', 'robot1',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ]
    )

    spawn_robot2 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-string', robot_description_raw2,
            '-name', 'robot2',
            '-x', '2.0',
            '-y', '0.0',
            '-z', '0.1'
        ]
    )

#    spawn_robot1 = Node(
#        package='ros_gz_sim',
#        executable='create',
#        arguments=[
#            '-topic', '/robot1/robot_description',
#            '-name', 'robot1',
#            '-x', '0.0',
#            '-y', '0.0',
#            '-z', '0.1'
#        ],
#        output='screen'
#    )
#    spawn_robot2 = Node(
#        package='ros_gz_sim',
#        executable='create',
#        arguments=[
#            '-topic', '/robot1/robot_description',
#            '-name', 'robot2',
#            '-x', '2.0',   # shift in X
#            '-y', '0.0',   # or offset in Y if you prefer
#            '-z', '0.1'
#        ],
#        output='screen'
#    )
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
#            period=2.0,
#            actions=[spawn_robot1]
#        )
        spawn_robot1,
        spawn_robot2,
        bridge,
        TimerAction(
            period=2.5,
            actions=[robot1_controller, robot2_controller]
        )
#        robot1_controller
    ])
