/*
 * @Author: wangdezhao
 * @Date: 2022-04-09 13:52:36
 * @LastEditTime: 2022-04-13 18:01:02
 * @FilePath: /osqp_eigen/include/piecewise_jerk_path_optimizer.h
 * @Copyright:
 */

#pragma once

#include <array>
#include <utility>
#include <vector>
#include <cmath>

#include "vehicleConfig.hpp"
#include "piecewise_jerk_path_problem.h"

namespace points {
struct PathPoint {
  PathPoint(double x_): x(x_){};
  double x;
  double y = 0;
  double z = 0;
  double s = 0;
  double theta = 0;
  double kappa = 0;
  double dkappa = 0;
  double ddkappa = 0;
};
}  // namespace points

namespace planning {

class PiecewiseJerkPathOptimizer {
 public:

  PiecewiseJerkPathOptimizer();
  // points::PathPoint init_points(0.0);
  // points::PathPoint end_points(100.0);

  virtual ~PiecewiseJerkPathOptimizer() = default;

 public:
  void Process(
      /* const SpeedData& speed_data, */ /* const ReferenceLine& reference_line,
                                          */
      /* const common::TrajectoryPoint& init_point, */
      const points::PathPoint& init_point,
      /* const bool path_reusable, */ /* PathData* const path_data */const points::PathPoint& end_points);

  //   common::TrajectoryPoint InferFrontAxeCenterFromRearAxeCenter(
  //       const common::TrajectoryPoint& traj_point);

  //   std::vector<common::PathPoint> ConvertPathPointRefFromFrontAxeToRearAxe(
  //       const PathData& path_data);

  bool OptimizePath(
      const std::array<double, 3>& init_state,
      const std::array<double, 3>& end_state, const double delta_s,
      const std::vector<std::pair<double, double>>& lat_boundaries,
      const std::vector<std::pair<double, double>>& ddl_bounds,
      const std::array<double, 5>& w, std::vector<double>* ptr_x,
      std::vector<double>* ptr_dx, std::vector<double>* ptr_ddx,
      const int max_iter);

  //   FrenetFramePath ToPiecewiseJerkPath(const std::vector<double>& l,
  //                                       const std::vector<double>& dl,
  //                                       const std::vector<double>& ddl,
  //                                       const double delta_s,
  //                                       const double start_s) const;

  double EstimateJerkBoundary(const double vehicle_speed,
                              const double axis_distance,
                              const double max_steering_rate) const;
  //    void Process(
  //       const SpeedData &speed_data, const ReferenceLine &reference_line,
  //       const common::TrajectoryPoint &init_point, const bool path_reusable,
  //       PathData *const path_data){};

};

}  // namespace planning
