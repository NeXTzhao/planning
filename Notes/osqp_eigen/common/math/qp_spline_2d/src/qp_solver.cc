/*
 * @Author: wangdezhao
 * @Date: 2022-04-28 18:14:12
 * @LastEditTime: 2022-04-30 11:24:54
 * @FilePath: /osqp_eigen/common/math/qp_spline_2d/src/qp_solver.cc
 * @Copyright:
 */

/**
 * @file: qp_solver.cc
 **/
#include "qp_solver.h"

namespace apollo {
namespace common {
namespace math {

QpSolver::QpSolver(const Eigen::MatrixXd& kernel_matrix,
                   const Eigen::MatrixXd& offset,
                   const Eigen::MatrixXd& affine_inequality_matrix,
                   const Eigen::MatrixXd& affine_inequality_boundary,
                   const Eigen::MatrixXd& affine_equality_matrix,
                   const Eigen::MatrixXd& affine_equality_boundary)
    : kernel_matrix_(kernel_matrix),
      offset_(offset),
      affine_inequality_matrix_(affine_inequality_matrix),
      affine_inequality_boundary_(affine_inequality_boundary),
      affine_equality_matrix_(affine_equality_matrix),
      affine_equality_boundary_(affine_equality_boundary) {}

const Eigen::MatrixXd& QpSolver::params() const { return params_; }

const Eigen::MatrixXd& QpSolver::kernel_matrix() const {
  return kernel_matrix_;
}

const Eigen::MatrixXd& QpSolver::offset() const { return offset_; }

const Eigen::MatrixXd& QpSolver::affine_equality_matrix() const {
  return affine_equality_matrix_;
}

const Eigen::MatrixXd& QpSolver::affine_equality_boundary() const {
  return affine_equality_boundary_;
}

const Eigen::MatrixXd& QpSolver::affine_inequality_matrix() const {
  return affine_inequality_matrix_;
}

const Eigen::MatrixXd& QpSolver::affine_inequality_boundary() const {
  return affine_inequality_boundary_;
}

}  // namespace math
}  // namespace common
}  // namespace apollo
