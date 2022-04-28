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
