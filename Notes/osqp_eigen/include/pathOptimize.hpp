/*
 * @Author: wangdezhao
 * @Date: 2022-04-10 21:27:05
 * @LastEditTime: 2022-04-10 21:58:13
 * @FilePath: /osqp_eigen/include/pathOptimize.hpp
 * @Copyright:
 */
#pragma once
#include <iostream>

#include "csc_matrix_conv.hpp"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/SparseCore"
#include "osqp_interface.hpp"

class PathOptimize {
//   size_t num_of_knots_ = 0;

//   // output
//   std::vector<double> x_;
//   std::vector<double> dx_;
//   std::vector<double> ddx_;

//   std::array<double, 3> x_init_;

//   std::vector<std::pair<double, double>> x_bounds_;
//   std::vector<std::pair<double, double>> dx_bounds_;
//   std::vector<std::pair<double, double>> ddx_bounds_;
//   std::pair<double, double> dddx_bound_;
public:
  double weight_x_ = 0.0;
  double weight_dx_ = 0.0;
  double weight_ddx_ = 0.0;
  double weight_dddx_ = 0.0;

  double delta_s_ = 1.0;
  
  Eigen::Matrix<double, 24, 1> x;
  Eigen::Matrix<double, 12, 12> p;
  Eigen::Matrix<double, 1, 12> q;
  Eigen::Matrix<double, 24, 12> A ;

// Eigen::MatrixXd A = Eigen::MatrixXd::Identity(24,12);
  Eigen::Matrix<double, 24, 1>  lb;
  Eigen::Matrix<double, 24, 1>  ub;


//   bool has_x_ref_ = false;
//   double weight_x_ref_ = 0.0;
//   std::vector<double> x_ref_;

//   bool has_end_state_ref_ = false;
//   std::array<double, 3> weight_end_state_ = {{0.0, 0.0, 0.0}};
//   std::array<double, 3> end_state_ref_;
};