/*
 * @Author: wangdezhao
 * @Date: 2022-04-09 13:52:36
 * @LastEditTime: 2022-04-13 11:18:33
 * @FilePath: /osqp_eigen/include/piecewise_jerk_path_optimizer.h
 * @Copyright:
 */

#pragma once

#include <utility>
#include <vector>

#include "vehicleConfig.hpp"

namespace planning {

class PiecewiseJerkPathOptimizer {
 public:
  PiecewiseJerkPathOptimizer();

  virtual ~PiecewiseJerkPathOptimizer() = default;

 private:
  void Process(/* const SpeedData& speed_data, */ const ReferenceLine& reference_line,
               const common::TrajectoryPoint& init_point,
               /* const bool path_reusable, */ PathData* const path_data) override;

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
