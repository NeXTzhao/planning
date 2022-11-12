/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "discrete_points_reference_line_smoother.h"

#include <algorithm>
#include <iostream>

//#include "cyber/common/file.h"
//#include "log.h"
//#include "modules/common/util/util.h"
//#include "modules/planning/common/planning_gflags.h"
#include "discrete_points_math.h"
#include "fem_pos_deviation_smoother.h"

namespace apollo::planning {
  bool DiscretePointsReferenceLineSmoother::Smooth(
    const std::vector<Eigen::Vector2d> &raw_points_,
    std::vector<double> &solve_x, std::vector<double> &solve_y) {
    std::vector<std::pair<double, double>> raw_point2d;
    std::vector<double> anchorpoints_lateralbound;

    for (const auto &point: raw_points_) {
      raw_point2d.emplace_back(point.x(), point.y());
      anchorpoints_lateralbound.emplace_back(0.25);
    }
    // fix front and back points to avoid end states deviate from the center of
    // road
    anchorpoints_lateralbound.front() = 0.0;
    anchorpoints_lateralbound.back() = 0.0;

    NormalizePoints(&raw_point2d);

    bool status = false;
    std::vector<std::pair<double, double>> smoothed_point2d;
    status =
      FemPosSmooth(raw_point2d, anchorpoints_lateralbound, &smoothed_point2d);

    if (!status) {
      std::cout << "discrete_points reference line smoother fails" << '\n';
      return false;
    }

    DeNormalizePoints(&smoothed_point2d);
    if (smoothed_point2d.size() < 2) {
      std::cout << "Fail to generate smoothed reference line." << '\n';
      return false;
    }
//    int i = 0;
    for (const auto &item: smoothed_point2d) {
      solve_x.emplace_back(item.first);
      solve_y.emplace_back(item.second);
//      printf("solveX_%u=%f,solveY_%u =%f\n", i, item.first, i, item.second);
//      ++i;
    }
    return true;
  }

  bool DiscretePointsReferenceLineSmoother::FemPosSmooth(
    const std::vector<std::pair<double, double>> &raw_point2d,
    const std::vector<double> &bounds,
    std::vector<std::pair<double, double>> *ptr_smoothed_point2d) {
    // const auto& fem_pos_config =
    //     config_.discrete_points().fem_pos_deviation_smoothing();

    // FemPosDeviationSmoother smoother(fem_pos_config);
    FemPosDeviationSmoother smoother;

    // box contraints on pos are used in fem pos smoother, thus shrink the
    // bounds by 1.0 / sqrt(2.0)
    std::vector<double> box_bounds = bounds;
    const double box_ratio = 1.0 / std::sqrt(2.0);
    for (auto &bound: box_bounds) {
      bound *= box_ratio;
    }

    std::vector<double> opt_x;
    std::vector<double> opt_y;
    bool status = smoother.Solve(raw_point2d, box_bounds, &opt_x, &opt_y);

    if (!status) {
      //    AERROR << "Fem Pos reference line smoothing failed";
      return false;
    }

    if (opt_x.size() < 2 || opt_y.size() < 2) {
      //    AERROR << "Return by fem pos smoother is wrong. Size smaller than 2 ";
      return false;
    }

    //  CHECK_EQ(opt_x.size(), opt_y.size()) << "x and y result size not equal";

    size_t point_size = opt_x.size();
    for (size_t i = 0; i < point_size; ++i) {
      ptr_smoothed_point2d->emplace_back(opt_x[i], opt_y[i]);
    }

    return true;
  }

  //void DiscretePointsReferenceLineSmoother::SetAnchorPoints(
  //    const std::vector<AnchorPoint> &anchor_points) {
  //  CHECK_GT(anchor_points.size(), 1);
  //  anchor_points_ = anchor_points;
  //}

  void DiscretePointsReferenceLineSmoother::NormalizePoints(
    std::vector<std::pair<double, double>> *xy_points) {
    zero_x_ = xy_points->front().first;
    zero_y_ = xy_points->front().second;
    std::for_each(xy_points->begin(), xy_points->end(),
                  [this](std::pair<double, double> &point) {
                    auto curr_x = point.first;
                    auto curr_y = point.second;
                    std::pair<double, double> xy(curr_x - zero_x_,
                                                 curr_y - zero_y_);
                    point = std::move(xy);
                  });
  }

  void DiscretePointsReferenceLineSmoother::DeNormalizePoints(
    std::vector<std::pair<double, double>> *xy_points) {
    std::for_each(xy_points->begin(), xy_points->end(),
                  [this](std::pair<double, double> &point) {
                    auto curr_x = point.first;
                    auto curr_y = point.second;
                    std::pair<double, double> xy(curr_x + zero_x_,
                                                 curr_y + zero_y_);
                    point = std::move(xy);
                  });
  }

}// namespace apollo::planning
