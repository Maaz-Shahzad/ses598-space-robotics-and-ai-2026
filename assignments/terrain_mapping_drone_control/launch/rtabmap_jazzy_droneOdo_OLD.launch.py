from launch.actions import ExecuteProcess
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import LogInfo

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        # Static TF: base_link -> camera_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_to_base_link',
            arguments=[
                '--x', '0.1', '--y', '0', '--z', '0.05',
                '--yaw', '0', '--pitch', '0', '--roll', '0',
                '--frame-id', 'base_link', '--child-frame-id', 'camera_link'
            ],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen'
        ),

        # Static TF: base_link -> OakD-Lite-Modify/base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_oakd',
            arguments=[
                '--x', '0.12', '--y', '0.03', '--z', '0.242',
                '--yaw', '0', '--pitch', '0', '--roll', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'OakD-Lite-Modify/base_link'
            ],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen'
        ),

        # RTAB-Map SLAM node
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'subscribe_depth': True,
                'subscribe_rgb': True,
                'subscribe_scan': False,
                'subscribe_odom_info': False,
                'subscribe_odom': True,
                'approx_sync': True,
                'approx_sync_max_interval': 0.5,
                'sync_queue_size': 10,
                'wait_for_transform': 0.5,
                'visual_odometry': False,
                'grid_cell_size': 0.05,
                'grid_size': 20.0,
                'optimize_from_graph_end': True,
                'optimizer_iterations': 100,
                'loop_closure_activated': True,
                'loop_closure_min_inliers': 20,
                'memory_management': True,
                'max_cloud_size': 50000,
                'min_cluster_size': 100,
                'qos_image': 1,
                'qos_camera_info': 1,
                'qos_odom': 1,
#
                'Mem/SaveDepth16Format': 'false',
                'Rtabmap/SaveWMState': 'true',
                'Odom/Strategy': '0',
                'RGBD/LinearUpdate': '0.0',
                'RGBD/AngularUpdate': '0.0',
            }],
            remappings=[
                ('rgb/image', '/drone/front_rgb'),
                ('depth/image', '/drone/front_depth'),
                ('depth/camera_info', '/drone/front_depth/camera_info'),
                ('rgb/camera_info', '/drone/front_rgb/camera_info'),
                ('odom', '/drone/odom'),
                ('grid_map', 'map'),
                ('mapData', 'mapData'),
                ('mapPath', 'mapPath'),
                ('cloud_map', 'cloud_map')
            ]
        ),

        # RTAB-Map point cloud generation
        Node(
            package='rtabmap_util',
            executable='point_cloud_xyz',
            name='point_cloud_xyz',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'decimation': 4,
                'voxel_size': 0.02,
                'max_depth': 5.0,
                'min_depth': 0.5
            }],
            remappings=[
                ('depth/image', '/drone/front_depth'),
                ('depth/camera_info', '/drone/front_depth/camera_info'),
                ('odom', '/drone/odom'),
                ('cloud', 'cloud_xyz')
            ]
        ),

        # RTAB-Map Viz
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'subscribe_depth': True,
                'subscribe_rgb': True,
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'map_frame_id': 'map',
                'qos_image': 1,
                'qos_camera_info': 1,
                'qos_odom': 1,
            }],
            remappings=[
                ('rgb/image', '/drone/front_rgb'),
                ('depth/image', '/drone/front_depth'),
                ('rgb/camera_info', '/drone/front_rgb/camera_info'),
                ('odom', '/drone/odom'),
            ]
        ),

#        Node(
#            package='terrain_mapping_drone_control',
#            executable='odom_tf_publisher',
#            name='odom_tf_publisher',
#            output='screen',
#            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
#        ),

#        ExecuteProcess(
#            cmd=['python3', os.path.join(
#                os.environ['HOME'],
#                'ros2_ws/src/terrain_mapping_drone_control/terrain_mapping_drone_control/odom_tf_publisher.py'
#            )],
#            output='screen'
#        ),

        ExecuteProcess(
            cmd=['python3', os.path.join(
                os.environ['HOME'],
                'ros2_ws/src/terrain_mapping_drone_control/terrain_mapping_drone_control/px4_odom_converter.py'
            )],
            output='screen'
        ),

        LogInfo(msg="RTAB-Map launched with vehicle odometry")
    ])
