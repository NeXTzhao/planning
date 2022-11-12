/*
 * @Author: wangdezhao
 * @Date: 2022-04-28 15:48:22
 * @LastEditTime: 2022-04-28 19:03:20
 * @FilePath: /osqp_eigen/common/math/qp_spline_2d/include/spline_2d.h
 * @Copyright:
 */

/**
 * @file : spline_2d.h
 * @brief: piecewise smoothing spline 2d class
 **/

#pragma once

#include <utility>
#include <vector>

#include "Eigen/Core"
#include "polynomial_xd.h"
#include "spline_2d_seg.h"

namespace apollo {
namespace planning {

class Spline2d {
 public:
  Spline2d(const std::vector<double>& t_knots, const uint32_t order);
  std::pair<double, double> operator()(const double t) const;
  double x(const double t) const;
  double y(const double t) const;
  /* derivativex 一阶导 */
  double DerivativeX(const double t) const;
  double DerivativeY(const double t) const;
  /* derivativex 二阶导 */
  double SecondDerivativeX(const double t) const;
  double SecondDerivativeY(const double t) const;
  /* derivativex 三阶导 */
  double ThirdDerivativeX(const double t) const;
  double ThirdDerivativeY(const double t) const;
  //设置样条参数、阶数
  bool set_splines(const Eigen::MatrixXd& params, const uint32_t order);
  const Spline2dSeg& smoothing_spline(const uint32_t index) const;
  const std::vector<double>& t_knots() const;
  uint32_t spline_order() const;

 private:
  uint32_t find_index(const double x) const;

 private:
  std::vector<Spline2dSeg> splines_;
  std::vector<double> t_knots_;
  uint32_t spline_order_;
};

}  // namespace planning
}  // namespace apollo
