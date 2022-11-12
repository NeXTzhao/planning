/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

/**
 * @file
 **/

#pragma once

#include <string>
#include <vector>

#include "reference_line.h"
// #include "reference_line_smoother_config.pb.h"

namespace apollo {
namespace common {
struct PathPoint {
  void set_x(const double value) { x_ = value; }
  void set_y(const double value) { y_ = value; }
  void set_z(const double value) { z_ = value; }
  void set_theta(const double value) { theta_ = value; }
  void set_kappa(const double value) { kappa_ = value; }
  void set_s(const double value) { s_ = value; }
  void set_dkappa(const double value) { dkappa_ = value; }
  void set_x_derivative(const double value) { x_derivative_ = value; }
  void set_y_derivative(const double value) { y_derivative_ = value; }

  double x() const { return x_; }
  double y() const { return y_; }
  double z() const { return z_; }
  double theta() const { return theta_; }
  double kappa() const { return kappa_; }
  double s() const { return s_; }
  double dkappa() const { return dkappa_; }
  double x_derivative() const { return x_derivative_; }
  double y_derivative() const { return y_derivative_; }

  // coordinates
  double x_;
  double y_;
  double z_;

  // direction on the x-y plane
  double theta_;
  // curvature on the x-y planning
  double kappa_;
  // accumulated distance from beginning of the path
  double s_;

  // derivative of kappa w.r.t s.
  double dkappa_;
  // derivative of derivative of kappa w.r.t s.
  double ddkappa_;
  // The lane ID where the path point is on
  // std::string lane_id;

  // derivative of x and y w.r.t parametric parameter t in CosThetareferenceline
  double x_derivative_;
  double y_derivative_;
};
}  // namespace common

namespace planning {
struct AnchorPoint {
  // common::PathPoint path_point;
  common::PathPoint path_point;
  double lateral_bound = 0.0;
  double longitudinal_bound = 0.0;
  // enforce smoother to strictly follow this reference point
  bool enforced = false;
};

class ReferenceLineSmoother {
 public:
  // explicit ReferenceLineSmoother(const ReferenceLineSmootherConfig& config)
  //     : config_(config) {}
  explicit ReferenceLineSmoother() {}

  /**
   * Smoothing constraints
   */
  virtual void SetAnchorPoints(
      const std::vector<AnchorPoint>& achor_points) = 0;

  /**
   * Smooth a given reference line
   */
  virtual bool Smooth(const ReferenceLine&, ReferenceLine* const) = 0;

  virtual ~ReferenceLineSmoother() = default;

//  protected:
  // ReferenceLineSmootherConfig config_;
};
}  // namespace planning
}  // namespace apollo
