/*
 * @Author: wangdezhao
 * @Date: 2022-04-07 21:11:19
 * @LastEditTime: 2022-04-09 10:18:39
 * @FilePath: /osqp_eigen/common/math/osqp_interface/include/csc_matrix_conv.hpp
 * /osqp_eigen/common/math/osqp_interface/include/osqp_interface/csc_matrix_conv.hpp
 * @Copyright:
 */

#pragma once

#include <vector>

#include "eigen3/Eigen/Core"
#include "osqp/glob_opts.h"  // for 'c_int' type ('long' or 'long long')
// #include "osqp_interface/visibility_control.hpp"
#include "interpolation_utils.hpp"
#include "visibility_control.hpp"

// namespace autoware {
namespace common {
namespace osqp {
/// \brief Compressed-Column-Sparse Matrix
struct OSQP_INTERFACE_PUBLIC CSC_Matrix {
  /// Vector of non-zero values. Ex: [4,1,1,2]
  std::vector<c_float> m_vals;
  /// Vector of row index corresponding to values. Ex: [0, 1, 0, 1] (Eigen:
  /// 'inner')
  std::vector<c_int> m_row_idxs;
  /// Vector of 'val' indices where each column starts. Ex: [0, 2, 4] (Eigen:
  /// 'outer')
  std::vector<c_int> m_col_idxs;
};

/// \brief Calculate CSC matrix from Eigen matrix
OSQP_INTERFACE_PUBLIC CSC_Matrix calCSCMatrix(const Eigen::MatrixXd& mat);
// 从方形 Eigen 矩阵计算上梯形 CSC 矩阵
/// \brief Calculate upper trapezoidal CSC matrix from square Eigen matrix
OSQP_INTERFACE_PUBLIC CSC_Matrix
calCSCMatrixTrapezoidal(const Eigen::MatrixXd& mat);
/// \brief Print the given CSC matrix to the standard output
OSQP_INTERFACE_PUBLIC void printCSCMatrix(const CSC_Matrix& csc_mat);

}  // namespace osqp
}  // namespace common
