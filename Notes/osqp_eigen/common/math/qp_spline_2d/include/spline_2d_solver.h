/*
 * @Author: wangdezhao
 * @Date: 2022-04-28 15:48:22
 * @LastEditTime: 2022-04-28 16:21:38
 * @FilePath: /osqp_eigen/common/math/qp_spline_2d/include/spline_2d_solver.h
 * @Copyright:  
 */

/**
 * @file spline_2d_solver.h
 **/

#pragma once

#include <vector>

#include "qp_solver.h"
#include "spline_2d.h"
#include "spline_2d_constraint.h"
#include "spline_2d_kernel.h"

namespace apollo {
namespace planning {

class Spline2dSolver {
 public:
  Spline2dSolver(const std::vector<double>& t_knots, const uint32_t order)
      : spline_(t_knots, order),
        kernel_(t_knots, order),
        constraint_(t_knots, order) {}

  virtual ~Spline2dSolver() = default;

  virtual void Reset(const std::vector<double>& t_knots,
                     const uint32_t order) = 0;

  // customize setup
  virtual Spline2dConstraint* mutable_constraint() = 0;
  virtual Spline2dKernel* mutable_kernel() = 0;
  virtual Spline2d* mutable_spline() = 0;

  // solve
  virtual bool Solve() = 0;

  // extract
  virtual const Spline2d& spline() const = 0;

 protected:
  Spline2d spline_;
  Spline2dKernel kernel_;
  Spline2dConstraint constraint_;
};

}  // namespace planning
}  // namespace apollo
