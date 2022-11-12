/*
 * @Author: wangdezhao
 * @Date: 2022-04-12 17:52:27
 * @LastEditTime: 2022-04-17 16:23:00
 * @FilePath: /osqp_eigen/include/vehicleConfig.hpp
 * @Copyright:
 */
constexpr double length = 4.933;
constexpr double width = 2.11;
constexpr double height = 1.48;
constexpr double min_turn_radius = 5.05386147161;
constexpr double max_acceleration = 2.0;
constexpr double max_deceleration = -6.0;
constexpr double max_steer_angle = 8.20304748437;
constexpr double max_steer_angle_rate = 8.55211;
constexpr double steer_ratio = 16;
constexpr double wheel_base = 2.8448;
constexpr double wheel_rolling_radius = 0.335;
constexpr double max_abs_speed_when_stopped = 0.2;
constexpr double FLAGS_lateral_derivative_bound_default = 2;
constexpr double FLAGS_lateral_jerk_bound = 4.0;

/*weight*/
constexpr double l_weight = 1.0;
constexpr double dl_weight = 20.0;
constexpr double ddl_weight = 1000.0;
constexpr double dddl_weight = 50000.0;

// constexpr double l_weight = 1.0;
// constexpr double dl_weight = 5.0;
// constexpr double ddl_weight = 800.0;
// constexpr double dddl_weight = 30000.0;

// constexpr double kMaxVariableRange = 1.0e10;
constexpr double kMaxVariableRange = 8;

/**********************************************************************/
// config_.mutable_spiral()->set_max_deviation(0.1);
// config_.mutable_spiral()->set_max_iteration(300);
// config_.mutable_spiral()->set_opt_tol(1.0e-6);
// config_.mutable_spiral()->set_opt_acceptable_tol(1.0e-4);

// config_.mutable_spiral()->set_weight_curve_length(1.0);
// config_.mutable_spiral()->set_weight_kappa(1.0);
// config_.mutable_spiral()->set_weight_dkappa(100.0);

constexpr double spiral_max_deviation = 0.1;
constexpr double spiral_max_iteration = 300;
constexpr double spiral_opt_tol = 1.0e-6;
constexpr double spiral_opt_acceptable_tol = 1.0e-4;
constexpr double spiral_weight_curve_length = 1.0;
constexpr double spiral_weight_kappa = 1.0;
constexpr double spiral_weight_dkappa = 100.0;
constexpr double spiral_opt_acceptable_iteration = 15;
constexpr double spiral_piecewise_length = 10.0;
constexpr double resolution = 0.02;

// message SpiralSmootherConfig {
//   // The max deviation of spiral reference line smoother.
//   optional double max_deviation = 1 [default = 0.1];

//   // The piecewise length of spiral smoother.
//   optional double piecewise_length = 2 [default = 10.0];

//   // The iteration num of spiral reference line smoother.");
//   optional uint32 max_iteration = 3 [default = 1000];

//   // The desired convergence tol for spiral opt;
//   optional double opt_tol = 4 [default = 1.0e-8];

//   // The acceptable convergence tol for spiral opt
//   optional double opt_acceptable_tol = 5 [default = 1e-6];

//   // The number of acceptable iters before termination for spiral opt;
//   optional uint32 opt_acceptable_iteration = 6 [default = 15];

//   // The weight of curve length term in objective function
//   optional double weight_curve_length = 7 [default = 1.0];

//   // The weight of kappa term in objective function
//   optional double weight_kappa = 8 [default = 1.0];

//   // The weight of dkappa term in objective function
//   optional double weight_dkappa = 9 [default = 100.0];
// }

// message ReferenceLineSmootherConfig {
//   // The output resolution for discrete point smoother reference line is
//   // directly decided by max_constraint_interval
//   optional double max_constraint_interval = 1 [default = 5];
//   optional double longitudinal_boundary_bound = 2 [default = 1.0];
//   optional double max_lateral_boundary_bound = 3 [default = 0.5];
//   optional double min_lateral_boundary_bound = 4 [default = 0.2];
//   // The output resolution for qp smoother reference line.
//   optional uint32 num_of_total_points = 5 [default = 500];
//   optional double curb_shift = 6 [default = 0.2];
//   optional double lateral_buffer = 7 [default = 0.2];
//   // The output resolution for spiral smoother reference line.
//   optional double resolution = 8 [default = 0.02];
//   oneof SmootherConfig {
//     QpSplineSmootherConfig qp_spline = 20;
//     SpiralSmootherConfig spiral = 21;
//     DiscretePointsSmootherConfig discrete_points = 22;
//   }
// }

/*********************************************************/