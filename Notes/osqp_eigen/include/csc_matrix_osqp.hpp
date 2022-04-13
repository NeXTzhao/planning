/*
 * @Author: wangdezhao
 * @Date: 2022-04-09 00:48:07
 * @LastEditTime: 2022-04-09 14:00:38
 * @FilePath: /osqp_eigen/include/csc_matrix_osqp.hpp
 * @Copyright:
 */
/*
 * @Author: wangdezhao
 * @Date: 2022-04-09 00:16:20
 * @LastEditTime: 2022-04-09 00:42:37
 * @FilePath: /osqp_eigen/src/csc_matrix.cpp
 * @Copyright:
 */
#pragma once
#include <iostream>

#include "csc_matrix_conv.hpp"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/SparseCore"
#include "osqp_interface.hpp"

using common::osqp::calCSCMatrix;
using common::osqp::CSC_Matrix;
using common::osqp::float64_t;
using common::osqp::printCSCMatrix;
