from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import LogInfo

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        # Static TF publisher for camera to base link transform
        # Updated to new ROS 2 Jazzy argument style
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_to_base_link',
            arguments=[
                '--x', '0.1', '--y', '0', '--z', '0.05', 
                '--yaw', '0', '--pitch', '0', '--roll', '0', 
                '--frame-id', 'base_link', '--child-frame-id', 'camera_link'
            ],
            output='screen'
        ),  

        # RTAB-Map node
        Node(
            package='rtabmap_slam', 
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                
                # RTAB-Map parameters
                'frame_id': 'base_link',
                'subscribe_depth': True,
                'subscribe_rgb': True,
                'subscribe_scan': False,
                'approx_sync': True,
#                'approx_sync_max_interval': 0.1,  # Increase allowed delay to 100ms
                'approx_sync_max_interval': 0.5,
                'sync_queue_size': 10,
                'wait_for_transform': 0.5,

#                # Change Quality of service to Best effort
#                'qos_image': 1,  # Force Best Effort
#                'qos_depth': 1,
#                'qos_odom': 2,

                # Odometry parameters
                'odom_frame_id': 'odom',
                'subscribe_odom_info': False,
                'subscribe_odom': True, # Check
                'odom_tf_angular_variance': 0.01,
                'odom_tf_linear_variance': 0.001,
                
                # Visual odometry parameters
                'visual_odometry': False,  # Using PX4/Gazebo odometry instead
                
                # Mapping parameters
                'grid_cell_size': 0.05,
                'grid_size': 20.0,
                'optimize_from_graph_end': True,
                'optimizer_iterations': 100,
                
                # Loop closure parameters
                'loop_closure_activated': True,
                'loop_closure_restriction_type': 0,
                'loop_closure_min_inliers': 20,
                
                # Memory management
                'memory_management': True,
                'max_cloud_size': 50000,
                'min_cluster_size': 100
            }],
            remappings=[
                # Camera topics matched to rover_world.launch.py
                ('rgb/image', '/drone/front_rgb'),
                ('depth/image', '/drone/front_depth'),
                ('depth/camera_info', '/drone/front_depth/camera_info'),
                ('rgb/camera_info', '/drone/front_rgb/camera_info'),
                
                # Odometry from PX4/Gazebo bridge
#                ('odom', '/fmu/out/vehicle_odometry'),
                ('odom', '/drone/odom'),
                
                # Output topics
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
                # Odometry from PX4/Gazebo bridge
#                ('odom', '/fmu/out/vehicle_odometry'),
                ('odom', '/drone/odom'),

                ('cloud', 'cloud_xyz')
            ]
        ),
        
        # RTAB-Map Viz Node
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
                'map_frame_id': 'map'
            }],
            remappings=[
                # Camera topics matched to rover_world.launch.py
                ('rgb/image', '/drone/front_rgb'),
                ('depth/image', '/drone/front_depth'),
                ('rgb/camera_info', '/drone/front_rgb/camera_info'),
#                ('odom', '/fmu/out/vehicle_odometry'),
                ('odom', '/drone/odom')
            ]
        ),

        # Log info
        LogInfo(
            msg="RTAB-Map launched with synchronized drone topics"
        )
    ])
