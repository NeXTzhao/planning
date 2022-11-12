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

/*
 * @file
 */

#pragma once

#include "../../math_method/box2d.h"
#include "PlannerOpenSpaceConfig.h"
#include <cassert>
#include <memory>
#include <string>
#include <vector>

namespace apollo::planning {

class Node3d {
 public:
  Node3d(double const x, double const y, double const phi);
  Node3d(double const x, double const y, double const phi,
         std::vector<double> const& XYbounds);
  Node3d(std::vector<double> const& traversed_x,
         std::vector<double> const& traversed_y,
         std::vector<double> const& traversed_phi,
         std::vector<double> const& XYbounds);
  virtual ~Node3d() = default;
  static apollo::common::math::Box2d GetBoundingBox(double const x,
                                                    double const y,
                                                    double const phi);
  double GetCost() const { return traj_cost_ + heuristic_cost_; }
  double GetTrajCost() const { return traj_cost_; }
  double GetHeuCost() const { return heuristic_cost_; }
  int GetGridX() const { return x_grid_; }
  int GetGridY() const { return y_grid_; }
  int GetGridPhi() const { return phi_grid_; }
  double GetX() const { return x_; }
  double GetY() const { return y_; }
  double GetPhi() const { return phi_; }
  bool operator==(Node3d const& right) const;
  std::string const& GetIndex() const { return index_; }
  size_t GetStepSize() const { return step_size_; }
  bool GetDirec() const { return direction_; }
  double GetSteer() const { return steering_; }
  std::shared_ptr<Node3d> GetPreNode() const { return pre_node_; }
  std::vector<double> const& GetXs() const { return traversed_x_; }
  std::vector<double> const& GetYs() const { return traversed_y_; }
  std::vector<double> const& GetPhis() const { return traversed_phi_; }
  void SetPre(std::shared_ptr<Node3d> pre_node) { pre_node_ = pre_node; }
  void SetDirec(bool direction) { direction_ = direction; }
  void SetTrajCost(double cost) { traj_cost_ = cost; }
  void SetHeuCost(double cost) { heuristic_cost_ = cost; }
  void SetSteer(double steering) { steering_ = steering; }

// private:
  //   static std::string ComputeStringIndex(int x_grid, int y_grid, int
  //   phi_grid);

 private:
  double x_ = 0.0;
  double y_ = 0.0;
  double phi_ = 0.0;
  size_t step_size_ = 1;
  std::vector<double> traversed_x_;
  std::vector<double> traversed_y_;
  std::vector<double> traversed_phi_;
  int x_grid_ = 0;
  int y_grid_ = 0;
  int phi_grid_ = 0;
  std::string index_;
  double traj_cost_ = 0.0;
  double heuristic_cost_ = 0.0;
  double cost_ = 0.0;
  std::shared_ptr<Node3d> pre_node_ = nullptr;
  double steering_ = 0.0;
  // true for moving forward and false for moving backward
  bool direction_ = true;
};

}  // namespace apollo
