/*
 * @Author: wangdezhao
 * @Date: 2022-04-28 15:48:22
 * @LastEditTime: 2022-04-29 21:12:36
 * @FilePath: /osqp_eigen/common/math/qp_spline_2d/include/spline_seg_kernel.h
 * @Copyright:
 */

/**
 * @file : spline_seg_kernel.h
 * @brief: generating integrated kernels for smoothing spline
 *
 *           x' P x  = int_0 ^x  (f(x)^(k))^2 dx, k = 0, 1, 2, 3
 *           P is the kernel of k-th smooth kernel
 **/

#pragma once

#include <string>

#include "Eigen/Core"
// #include "cyber/common/macros.h"

namespace apollo {
namespace planning {

class SplineSegKernel {
 public:
  SplineSegKernel();
  static SplineSegKernel& Instance();

  // generating kernel matrix
  Eigen::MatrixXd Kernel(const uint32_t num_params, const double accumulated_x);

  // only support N <= 3 cases
  Eigen::MatrixXd NthDerivativeKernel(const uint32_t n,
                                      const uint32_t num_params,
                                      const double accumulated_x);

 private:
  Eigen::MatrixXd DerivativeKernel(const uint32_t num_of_params,
                                   const double accumulated_x);
  Eigen::MatrixXd SecondOrderDerivativeKernel(const uint32_t num_of_params,
                                              const double accumulated_x);
  Eigen::MatrixXd ThirdOrderDerivativeKernel(const uint32_t num_of_params,
                                             const double accumulated_x);

  void IntegratedTermMatrix(const uint32_t num_of_params, const double x,
                            const std::string& type,
                            Eigen::MatrixXd* term_matrix) const;
  void CalculateFx(const uint32_t num_of_params);
  void CalculateDerivative(const uint32_t num_of_params);
  void CalculateSecondOrderDerivative(const uint32_t num_of_params);
  void CalculateThirdOrderDerivative(const uint32_t num_of_params);

  const uint32_t reserved_order_ = 5;
  Eigen::MatrixXd kernel_fx_;
  Eigen::MatrixXd kernel_derivative_;
  Eigen::MatrixXd kernel_second_order_derivative_;
  Eigen::MatrixXd kernel_third_order_derivative_;

  // DECLARE_SINGLETON(SplineSegKernel)
};

}  // namespace planning
}  // namespace apollo
