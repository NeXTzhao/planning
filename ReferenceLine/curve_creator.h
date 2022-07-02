#pragma once

#include <array>
#include <cmath>
#include <memory>
#include <tuple>
#include <utility>

#include "curve.h"

namespace reference_line {

class CurveCreator {
  using Node = std::array<double, 4>;
  std::vector<Node> nodes_;

  std::vector<double> deltas_;

  std::shared_ptr<Curve> curve_ = nullptr;

 public:
  CurveCreator(std::vector<double> a_xs, std::vector<double> a_ys);

  void solve();

  const std::shared_ptr<Curve> &curve() const { return curve_; }
};

}  // namespace reference_line