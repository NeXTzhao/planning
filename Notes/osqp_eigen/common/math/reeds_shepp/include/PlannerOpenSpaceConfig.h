// Hybrid a star for warm start
// Hybrid a star for warm start
constexpr double xy_grid_resolution = 0.2;
constexpr double phi_grid_resolution = 0.05;
constexpr int next_node_num = 10;
constexpr double step_size = 0.5;
constexpr double traj_forward_penalty = 0.0;
constexpr double traj_back_penalty = 0.0;
constexpr double traj_gear_switch_penalty = 10.0;
constexpr double traj_steer_penalty = 100.0;
constexpr double traj_steer_change_penalty = 10.0;
// Grid a star for heuristic
constexpr double grid_a_star_xy_resolution = 0.1;
constexpr double node_radius = 0.5;
//   constexpr PiecewiseJerkSpeedOptimizerConfig s_curve_config = 17;

// constexpr PiecewiseJerkSpeedOptimizerConfig s_curve_config = 17;

constexpr double length = 4.933;
constexpr double width = 2.11;
constexpr double height = 1.48;
constexpr double max_steer_angle = 8.20304748437;
constexpr double max_steer_angle_rate = 8.55211;
constexpr double steer_ratio = 16;
constexpr double wheel_base = 2.8448;
constexpr double max_abs_speed_when_stopped = 0.2;
constexpr double back_edge_to_center = 0.995;

// Open Space ROIConfig
// constexpr ROIConfig roi_config = 1;
// // Hybrid A Star Warm Start
// constexpr WarmStartConfig warm_start_config = 2;
// // Dual Variable Warm Start
// constexpr DualVariableWarmStartConfig dual_variable_warm_start_config = 3;
// // Distance Approach Configs
// constexpr DistanceApproachConfig distance_approach_config = 4;
// // Iterative Anchoring Configs
// constexpr IterativeAnchoringConfig iterative_anchoring_smoother_config = 5;
// Trajectory PartitionConfig Configs
//   constexpr TrajectoryPartitionConfig trajectory_partition_config = 6;
constexpr float delta_t = 1.0;
constexpr double is_near_destination_threshold = 0.001;
constexpr bool enable_check_parallel_trajectory = false;
constexpr bool enable_linear_interpolation = false;
constexpr double is_near_destination_theta_threshold = 0.05;

// Dual variable configs for OSQP
constexpr double alpha = 1.0;
constexpr double eps_abs = 1.0e-3;
constexpr double eps_rel = 1.0e-3;
constexpr int max_iter = 10000;
constexpr bool polish = true;
constexpr bool osqp_debug_log = false;