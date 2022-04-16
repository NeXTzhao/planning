/*
 * @Author: wangdezhao
 * @Date: 2022-04-13 17:46:06
 * @LastEditTime: 2022-04-13 21:45:45
 * @FilePath: /osqp_eigen/src/piecewise_jerk_path_optimizer.cc
 * @Copyright:
 */
#include "piecewise_jerk_path_optimizer.h"

#include <string>

namespace planning {

PiecewiseJerkPathOptimizer::PiecewiseJerkPathOptimizer() {}

void PiecewiseJerkPathOptimizer::Process(const points::PathPoint& init_point,
                                         const points::PathPoint& end_points) {
  std::array<double, 5> w = {l_weight, dl_weight, ddl_weight, dddl_weight, 0.0};

  int max_iter = 4000;
  max_iter = 4000;

  std::vector<double> opt_l;
  std::vector<double> opt_dl;
  std::vector<double> opt_ddl;

  std::array<double, 3> end_state = {0.0, 0.0, 0.0};

  const double veh_param = 10.0;
  // const auto& veh_param =10.
  //     common::VehicleConfigHelper::GetConfig().vehicle_param();
  const double lat_acc_bound =
      std::tan(max_steer_angle / steer_ratio) / wheel_base;
  std::vector<std::pair<double, double>> ddl_bounds;
}

bool PiecewiseJerkPathOptimizer::OptimizePath(
/*     const std::array<double, 3>& init_state,
    const std::array<double, 3>& end_state,  */
    const double delta_s,
    const std::vector<std::pair<double, double>>& lat_boundaries,
    const std::vector<std::pair<double, double>>& ddl_bounds,
    const std::array<double, 5>& w, std::vector<double>* x,
    std::vector<double>* dx, std::vector<double>* ddx, const int max_iter) {
  // PiecewiseJerkPathProblem piecewise_jerk_problem(lat_boundaries.size(),
  //                                                 delta_s, init_state);
  // std::array<double, 3> x_init;
  PiecewiseJerkPathProblem piecewise_jerk_problem(100, 0.1, init_state);
  // lat_boundaries d的个数
  std::vector<double> x_ref(lat_boundaries.size(), end_state[0]);

  piecewise_jerk_problem.set_weight_x(w[0]);
  piecewise_jerk_problem.set_weight_dx(w[1]);
  piecewise_jerk_problem.set_weight_ddx(w[2]);
  piecewise_jerk_problem.set_weight_dddx(w[3]);

  piecewise_jerk_problem.set_scale_factor({1.0, 10.0, 100.0});

  piecewise_jerk_problem.set_x_bounds(lat_boundaries);
  piecewise_jerk_problem.set_dx_bounds(-FLAGS_lateral_derivative_bound_default,
                                       FLAGS_lateral_derivative_bound_default);
  piecewise_jerk_problem.set_ddx_bounds(ddl_bounds);
  piecewise_jerk_problem.set_dddx_bound(FLAGS_lateral_jerk_bound);
  const double axis_distance = wheel_base;
  const double max_yaw_rate = max_steer_angle_rate / steer_ratio / 2.0;
  const double jerk_bound = EstimateJerkBoundary(std::fmax(init_state[1], 1.0),
                                                 axis_distance, max_yaw_rate);
  piecewise_jerk_problem.set_dddx_bound(jerk_bound);

  bool success = piecewise_jerk_problem.Optimize(max_iter);

  *x = piecewise_jerk_problem.opt_x();
  *dx = piecewise_jerk_problem.opt_dx();
  *ddx = piecewise_jerk_problem.opt_ddx();

  return true;
}

double PiecewiseJerkPathOptimizer::EstimateJerkBoundary(
    const double vehicle_speed, const double axis_distance,
    const double max_yaw_rate) const {
  return max_yaw_rate / axis_distance / vehicle_speed;
}

}  // namespace planning

int main() {
  points::PathPoint init_point(1.0);
  points::PathPoint end_point(500.0);

  // PiecewiseJerkPathOptimizer pathJerkOptm;
  // pathJerkOptm.OptimizePath();
  return 0;
}