/*
 * @Author: wangdezhao
 * @Date: 2022-04-28 18:15:46
 * @LastEditTime: 2022-04-30 11:29:24
 * @FilePath: /osqp_eigen/common/math/qp_spline_2d/include/affine_constraint.h
 * @Copyright:
 */

/**
 * @file : affine_constraint.h
 **/

#pragma once
#include <iostream>

#include "Eigen/Core"
#include "polynomial_xd.h"

namespace apollo {
namespace planning {

class AffineConstraint {
 public:
  AffineConstraint() = default;
  explicit AffineConstraint(const bool is_equality);
  AffineConstraint(const Eigen::MatrixXd& constraint_matrix,
                   const Eigen::MatrixXd& constraint_boundary,
                   const bool is_equality);

  void SetIsEquality(const double is_equality);

  const Eigen::MatrixXd& constraint_matrix() const;
  const Eigen::MatrixXd& constraint_boundary() const;
  bool AddConstraint(const Eigen::MatrixXd& constraint_matrix,
                     const Eigen::MatrixXd& constraint_boundary);

 private:
  Eigen::MatrixXd constraint_matrix_;
  Eigen::MatrixXd constraint_boundary_;
  bool is_equality_ = true;
};

}  // namespace planning
}  // namespace apollo
