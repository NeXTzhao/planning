/*
 * @Author: wangdezhao
 * @Date: 2022-04-28 15:40:52
 * @LastEditTime: 2022-04-30 11:31:03
 * @FilePath:
 * /osqp_eigen/common/math/qp_spline_2d/include/osqp_spline_2d_solver.h
 * @Copyright:
 */

/**
 * @file
 **/

#pragma once

#include <vector>

#include "gtest/gtest_prod.h"
#include "osqp/osqp.h"
#include "spline_2d.h"
#include "spline_2d_solver.h"

namespace apollo {
namespace planning {

class OsqpSpline2dSolver final : public Spline2dSolver {
 public:
  // knots 表示约束变量个数
  // order 表示曲线阶数
  OsqpSpline2dSolver(const std::vector<double>& t_knots, const uint32_t order);

  void Reset(const std::vector<double>& t_knots, const uint32_t order) override;

  // customize setup
  Spline2dConstraint* mutable_constraint() override;
  Spline2dKernel* mutable_kernel() override;
  Spline2d* mutable_spline() override;

  // solve
  bool Solve() override;

  // extract
  const Spline2d& spline() const override;

 private:
  // FRIEND_TEST(OSQPSolverTest, basic_test);

 private:
  OSQPSettings* osqp_settings_ = nullptr;
  OSQPWorkspace* work_ = nullptr;  // Workspace
  OSQPData* data_ = nullptr;       // OSQPData

  int last_num_constraint_ = 0;
  int last_num_param_ = 0;
  bool last_problem_success_ = false;
};

}  // namespace planning
}  // namespace apollo
