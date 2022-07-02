/*
 * @Author: wangdezhao
 * @Date: 2022-04-29 21:54:47
 * @LastEditTime: 2022-04-29 21:54:48
 * @FilePath: /osqp_eigen/common/math/qp_spline_2d/src/curve_math.cc
 * @Copyright:
 */

/**
 * @file curve_math.cc
 **/

#include "curve_math.h"

#include <cmath>

namespace apollo {
namespace planning {

// kappa = (dx * d2y - dy * d2x) / [(dx * dx + dy * dy)^(3/2)]
double CurveMath::ComputeCurvature(const double dx, const double d2x,
                                   const double dy, const double d2y) {
  const double a = dx * d2y - dy * d2x;
  auto norm_square = dx * dx + dy * dy;
  auto norm = std::sqrt(norm_square);
  const double b = norm * norm_square;
  return a / b;
}

double CurveMath::ComputeCurvatureDerivative(const double dx, const double d2x,
                                             const double d3x, const double dy,
                                             const double d2y,
                                             const double d3y) {
  const double a = dx * d2y - dy * d2x;
  const double b = dx * d3y - dy * d3x;
  const double c = dx * d2x + dy * d2y;
  const double d = dx * dx + dy * dy;

  return (b * d - 3.0 * a * c) / (d * d * d);
}

}  // namespace planning
}  // namespace apollo
