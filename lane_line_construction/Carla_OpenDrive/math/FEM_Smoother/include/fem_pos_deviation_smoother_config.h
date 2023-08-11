constexpr double weight_fem_pos_deviation = 1.0e10;
constexpr double weight_ref_deviation = 1.0;
constexpr double weight_path_length = 1.0;
//constexpr bool apply_curvature_constraint = true;
constexpr double weight_curvature_constraint_slack_var = 1.0e2;
constexpr double curvature_constraint = 0.2;
//constexpr bool use_sqp = true;
constexpr double sqp_ftol = 1e-4;
constexpr double sqp_ctol = 1e-3;
constexpr int sqp_pen_max_iter = 10;
constexpr int sqp_sub_max_iter = 100;

// osqp settings
constexpr int max_iter = 500;
// time_limit set to be 0.0 meaning no time limit
constexpr double time_limit = 0.0;
constexpr bool verbose = false;
constexpr bool scaled_termination = true;
constexpr bool warm_start = true;
