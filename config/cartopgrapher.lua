-- Include additional configuration files for Cartographer’s map and trajectory building
include "map_builder.lua"
include "trajectory_builder.lua"

-- Define main Cartographer options
options = {
  -- General SLAM parameters
  map_builder = MAP_BUILDER,  -- Specifies the map builder configuration
  trajectory_builder = TRAJECTORY_BUILDER,  -- Specifies the trajectory builder configuration
  
  -- ROS frame and transform configurations
  map_frame = "map",  -- The reference frame for the map; usually set to "map"
  tracking_frame = "base_link_projected",  -- The frame tracked by SLAM; typically the robot's base
  published_frame = "odom",  -- Frame for publishing SLAM poses; commonly "odom"
  odom_frame = "odom",  -- The frame for odometry if provide_odom_frame is enabled
  provide_odom_frame = false,  -- If true, publishes a continuous odom_frame without loop closure corrections
  publish_frame_projected_to_2d = true,  -- Projects the pose into 2D for 2D SLAM applications

  -- Sensor configurations
  use_odometry = true,  -- Enable if odometry is available; subscribes to /odom for additional pose data
  use_nav_sat = false,  -- Enable if GPS (NavSat) data is available
  use_landmarks = false,  -- Enable if landmarks are used in mapping (e.g., known beacons)

  -- Sensor topic configurations
  num_laser_scans = 0,  -- Number of single echo laser scanners; subscribes to /scan for laser data
  num_multi_echo_laser_scans = 0,  -- Number of multi-echo laser scanners
  num_subdivisions_per_laser_scan = 1,  -- Number of subdivisions per laser scan to improve map accuracy
  num_point_clouds = 2,  -- Number of 3D point clouds to subscribe to

  -- Timeouts and publish intervals
  lookup_transform_timeout_sec = 0.2,  -- Timeout for transform lookups using tf2
  submap_publish_period_sec = 0.3,  -- Interval in seconds for publishing submap poses
  pose_publish_period_sec = 5e-3,  -- Interval for publishing poses (200 Hz)
  trajectory_publish_period_sec = 30e-3,  -- Interval for publishing trajectory markers (30 ms)

  -- Sampling ratios for data sources
  rangefinder_sampling_ratio = 1.0,  -- Ratio of rangefinder data used in SLAM
  odometry_sampling_ratio = 1.0,  -- Ratio of odometry data used in SLAM
  fixed_frame_pose_sampling_ratio = 1.0,  -- Ratio of fixed frame pose data
  imu_sampling_ratio = 1.0,  -- Ratio of IMU data used in SLAM
  landmarks_sampling_ratio = 1.0,  -- Ratio of landmark data used in SLAM
}

-- Map builder configurations
MAP_BUILDER.use_trajectory_builder_2d = true  -- Enables 2D SLAM trajectory builder

-- 2D trajectory builder specific parameters
TRAJECTORY_BUILDER_2D.min_range = 0.12  -- Minimum range to consider for laser scans (meters)
TRAJECTORY_BUILDER_2D.max_range = 3.5  -- Maximum range for laser scans (meters)
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.0  -- Length to assign to missing laser rays (meters)
TRAJECTORY_BUILDER_2D.use_imu_data = false  -- Whether to use IMU data; set false if IMU is unavailable
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true  -- Enables correlative scan matching for better accuracy
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)  -- Angle threshold for motion filtering (in radians)

-- Pose graph optimization parameters
POSE_GRAPH.constraint_builder.min_score = 0.65  -- Minimum score for adding constraints to the pose graph
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7  -- Min score for global localization constraints

-- Uncomment to set optimization frequency for the pose graph
-- POSE_GRAPH.optimize_every_n_nodes = 0

-- Return the options table to be read by Cartographer
return options