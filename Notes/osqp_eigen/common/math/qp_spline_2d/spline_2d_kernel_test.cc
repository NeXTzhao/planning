/*
 * @Author: wangdezhao
 * @Date: 2022-04-28 15:48:22
 * @LastEditTime: 2022-04-28 19:12:36
 * @FilePath: /osqp_eigen/common/math/qp_spline_2d/spline_2d_kernel_test.cc
 * @Copyright:
 */


/**
 * @file
 **/
#include "spline_2d_kernel.h"

// #include "cyber/common/log.h"
#include "gtest/gtest.h"

namespace apollo {
namespace planning {

TEST(Spline2dKernel, add_regularization) {
  std::vector<double> x_knots = {0.0, 1.0, 2.0, 3.0};
  int32_t spline_order = 4;
  Spline2dKernel kernel(x_knots, spline_order);

  kernel.AddRegularization(0.2);

  for (int i = 0; i < kernel.kernel_matrix().rows(); ++i) {
    for (int j = 0; j < kernel.kernel_matrix().cols(); ++j) {
      if (i == j) {
        EXPECT_DOUBLE_EQ(kernel.kernel_matrix()(i, j), 0.4);
      } else {
        EXPECT_DOUBLE_EQ(kernel.kernel_matrix()(i, j), 0.0);
      }
    }
  }

  for (int i = 0; i < kernel.offset().rows(); ++i) {
    for (int j = 0; j < kernel.offset().cols(); ++j) {
      EXPECT_DOUBLE_EQ(kernel.offset()(i, j), 0.0);
    }
  }
}

}  // namespace planning
}  // namespace apollo
