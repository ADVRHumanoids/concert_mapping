from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument

from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    # -----------------------------------------------------
    # 1) Pull the environment variable 'USE_SIM_TIME', 
    #    defaulting to 'true' if not set
    # -----------------------------------------------------
    use_sim_time_env = EnvironmentVariable(
        name='USE_SIM_TIME',
        default_value='true'
    )

    # -----------------------------------------------------
    # 2) Declare Launch Arguments, using the environment
    #    variable as the default for 'use_sim_time'
    # -----------------------------------------------------
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value=use_sim_time_env,
        description='Use simulation time (useful for Gazebo simulations)'
    )

    deskewing_arg = DeclareLaunchArgument(
        'deskewing',
        default_value='false',
        description='Enable LiDAR deskewing'
    )

    # -----------------------------------------------------
    # 3) Retrieve LaunchConfigurations
    # -----------------------------------------------------
    use_sim_time = LaunchConfiguration('use_sim_time')
    deskewing = LaunchConfiguration('deskewing')

    # -----------------------------------------------------
    # 4) Define Nodes
    # -----------------------------------------------------
    icp_odometry_node = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='icp_odometry',
        output='screen',
        parameters=[{
            'frame_id': 'base_link_projected',
            'odom_frame_id': 'odom',
            'wait_for_transform': 0.2,
            'expected_update_rate': 15.0,
            'deskewing': deskewing,
            'use_sim_time': use_sim_time,
            # RTAB-Map's internal parameters are strings:
            'Icp/PointToPlane': 'true',
            'Icp/Iterations': '10',
            'Icp/VoxelSize': '0.1',
            'Icp/Epsilon': '0.001',
            'Icp/PointToPlaneK': '20',
            'Icp/PointToPlaneRadius': '0',
            'Icp/MaxTranslation': '2',
            'Icp/MaxCorrespondenceDistance': '1',
            'Icp/Strategy': '1',
            'Icp/OutlierRatio': '0.7',
            'Icp/CorrespondenceRatio': '0.01',
            'Odom/ScanKeyFrameThr': '0.4',
            'OdomF2M/ScanSubtractRadius': '0.1',
            'OdomF2M/ScanMaxSize': '15000',
            'OdomF2M/BundleAdjustment': 'false'
        }],
        remappings=[
            ('scan_cloud', '/merged_cloud')
        ]
    )

    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'frame_id': 'base_link_projected',
            'subscribe_depth': False,
            'subscribe_rgb': False,
            'subscribe_scan_cloud': True,
            'approx_sync': False,
            'wait_for_transform': 0.2,
            'use_sim_time': use_sim_time,
            # RTAB-Map's internal parameters
            'RGBD/ProximityMaxGraphDepth': '0',
            'RGBD/ProximityPathMaxNeighbors': '1',
            'RGBD/AngularUpdate': '0.05',
            'RGBD/LinearUpdate': '0.05',
            'RGBD/CreateOccupancyGrid': 'false',
            'Mem/NotLinkedNodesKept': 'false',
            'Mem/STMSize': '30',
            'Mem/LaserScanNormalK': '20',
            'Reg/Strategy': '1',
            'Icp/VoxelSize': '0.1',
            'Icp/PointToPlaneK': '20',
            'Icp/PointToPlaneRadius': '0',
            'Icp/PointToPlane': 'true',
            'Icp/Iterations': '10',
            'Icp/Epsilon': '0.001',
            'Icp/MaxTranslation': '3',
            'Icp/MaxCorrespondenceDistance': '1',
            'Icp/Strategy': '1',
            'Icp/OutlierRatio': '0.7',
            'Icp/CorrespondenceRatio': '0.2'
        }],
        remappings=[
            ('scan_cloud', 'odom_filtered_input_scan')
        ],
        arguments=['-d']  # Delete the previous database (~/.ros/rtabmap.db)
    )

    rtabmap_viz_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[{
            'frame_id': 'base_link_projected',
            'odom_frame_id': 'odom',
            'subscribe_odom_info': True,
            'subscribe_scan_cloud': True,
            'approx_sync': False,
            'use_sim_time': use_sim_time
        }],
        remappings=[
            ('scan_cloud', 'odom_filtered_input_scan')
        ]
    )

    # -----------------------------------------------------
    # 5) Assemble LaunchDescription
    # -----------------------------------------------------
    return LaunchDescription([
        use_sim_time_arg,
        deskewing_arg,
        icp_odometry_node,
        rtabmap_node,
        rtabmap_viz_node
    ])
