

#pragma once

#include <Eigen/Eigen>
#include <utility>
#include <vector>

// #include "modules/planning/proto/reference_line_smoother_config.pb.h"
// #include "modules/planning/reference_line/reference_line.h"
// #include "reference_line_smoother.h"
// #include "modules/planning/reference_line/reference_point.h"

namespace apollo::planning {

class DiscretePointsReferenceLineSmoother {
 public:
  explicit DiscretePointsReferenceLineSmoother() = default;

  ~DiscretePointsReferenceLineSmoother() = default;

  bool Smooth(std::vector<double> const& raw_x,
              std::vector<double> const& raw_y, std::vector<double>& solve_x,
              std::vector<double>& solve_y);

 private:
  static bool FemPosSmooth(
      std::vector<std::pair<double, double>> const& raw_point2d,
      std::vector<double> const& bounds,
      std::vector<std::pair<double, double>>* ptr_smoothed_point2d);

  void NormalizePoints(std::vector<std::pair<double, double>>* xy_points);

  void DeNormalizePoints(std::vector<std::pair<double, double>>* xy_points);

  // bool GenerateRefPointProfile(
  //     const ReferenceLine& raw_reference_line,
  //     const std::vector<std::pair<double, double>>& xy_points,
  //     std::vector<ReferencePoint>* reference_points);

  //    std::vector<AnchorPoint> anchor_points_{};

  double zero_x_ = 0.0;

  double zero_y_ = 0.0;
};

}  // namespace apollo::planning
