/*
 * @Author: wangdezhao
 * @Date: 2022-04-28 15:49:53
 * @LastEditTime: 2022-04-28 17:01:02
 * @FilePath: /osqp_eigen/common/math/qp_spline_2d/include/polynomial_xd.h
 * @Copyright:
 */

/**
 * @file: polynomial_xd.h
 **/

#pragma once

#include <cinttypes>
#include <vector>

namespace apollo {
namespace planning {

class PolynomialXd {
 public:
  PolynomialXd() = default;
  explicit PolynomialXd(const std::uint32_t order);
  explicit PolynomialXd(const std::vector<double>& params);
  double operator()(const double value) const;
  double operator[](const std::uint32_t index) const;
  void SetParams(const std::vector<double>& params);

  static PolynomialXd DerivedFrom(const PolynomialXd& base);
  static PolynomialXd IntegratedFrom(const PolynomialXd& base,
                                     const double intercept = 0.0);

  std::uint32_t order() const;
  const std::vector<double>& params() const;

 private:
  std::vector<double> params_;
};

}  // namespace planning
}  // namespace apollo
