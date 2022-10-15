/**
 * @file discrete_points_math.h
 **/

#pragma once

#include <utility>
#include <vector>

namespace apollo {
namespace planning {

class DiscretePointsMath {
 public:
  DiscretePointsMath() = delete;

  static bool ComputePathProfile(
      const std::vector<std::pair<double, double>>& xy_points,
      std::vector<double>* headings, std::vector<double>* accumulated_s,
      std::vector<double>* kappas, std::vector<double>* dkappas);
};

}  // namespace planning
}  // namespace apollo
