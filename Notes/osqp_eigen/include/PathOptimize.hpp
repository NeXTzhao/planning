/*
 * @Author: wangdezhao
 * @Date: 2022-04-10 21:27:05
 * @LastEditTime: 2022-04-16 15:39:17
 * @FilePath: /osqp_eigen/include/PathOptimize.hpp
 * @Copyright:
 */
#pragma once
#include <array>
#include <cmath>
#include <iostream>
#include <memory>
#include <tuple>
#include <utility>
#include <vector>

#include "circle.hpp"
#include "matplotlibcpp.h"
#include "osqp/osqp.h"
#include "spline_interpolation.hpp"
#include "stdio.h"
#include "vehicleConfig.hpp"

template <typename T>
T* CopyData(const std::vector<T>& vec) {
  T* data = new T[vec.size()];
  memcpy(data, vec.data(), sizeof(T) * vec.size());
  return data;
}

template <typename T>
void printVector(T& vec) {
  for (const auto& item : vec) {
    std::cout << item << std::endl;
  }
}

class PathOptimize {
 public:
  PathOptimize(size_t num_of_knots) : num_of_knots_(num_of_knots) {}
  void CalculateKernel(std::vector<c_float>* P_data,
                       std::vector<c_int>* P_indices,
                       std::vector<c_int>* P_indptr);
  void CalculateOffset(std::vector<c_float>* q);
  void CalculateAffineConstraint(std::vector<c_float>* A_data,
                                 std::vector<c_int>* A_indices,
                                 std::vector<c_int>* A_indptr,
                                 std::vector<c_float>* lower_bounds,
                                 std::vector<c_float>* upper_bounds);
  OSQPData* FormulateProblem();
  OSQPSettings* SolverDefaultSettings();
  void Optimize();
  void set_x_bounds(std::vector<std::pair<double, double>> x_bounds);

  void set_x_bounds(const double x_lower_bound, const double x_upper_bound);

  void set_dx_bounds(std::vector<std::pair<double, double>> dx_bounds);

  void set_dx_bounds(const double dx_lower_bound, const double dx_upper_bound);

  void set_ddx_bounds(std::vector<std::pair<double, double>> ddx_bounds);

  void set_ddx_bounds(const double ddx_lower_bound,
                      const double ddx_upper_bound);

  void set_dddx_bound(const double dddx_bound) {
    set_dddx_bound(-dddx_bound, dddx_bound);
  }

  void set_dddx_bound(const double dddx_lower_bound,
                      const double dddx_upper_bound) {
    dddx_bound_.first = dddx_lower_bound;
    dddx_bound_.second = dddx_upper_bound;
  }

  void set_weight_x(const double weight_x) { weight_x_ = weight_x; }

  void set_weight_dx(const double weight_dx) { weight_dx_ = weight_dx; }

  void set_weight_ddx(const double weight_ddx) { weight_ddx_ = weight_ddx; }

  void set_weight_dddx(const double weight_dddx) { weight_dddx_ = weight_dddx; }

  void set_x_ref(const double weight_x_ref, std::vector<double> x_ref);

  void set_end_state_ref(const std::array<double, 3>& weight_end_state,
                         const std::array<double, 3>& end_state_ref);

  void set_bound(size_t numPoints, const std::array<double, 4>& w,
                 const std::vector<double>& new_values);
  void matplot();

  ~PathOptimize() {}

 protected:
  std::vector<double> x_;
  std::vector<double> dx_;
  std::vector<double> ddx_;
  size_t num_of_knots_;

  std::array<double, 3> x_init_ = {1.875, 0.0, 0.0};
  std::vector<std::pair<double, double>> x_bounds_;
  std::vector<std::pair<double, double>> dx_bounds_;
  std::vector<std::pair<double, double>> ddx_bounds_;
  std::pair<double, double> dddx_bound_;
  std::vector<double> x_ref_;
  std::array<double, 3> weight_end_state_ = {{0.0, 0.0, 0.0}};
  std::array<double, 3> end_state_ref_ = {5.625, 0.0, 0.0};


  std::array<double, 3> scale_factor_ = {{1.0, 10.0, 100.0}};

  double weight_x_ = 0.0;
  double weight_dx_ = 0.0;
  double weight_ddx_ = 0.0;
  double weight_dddx_ = 0.0;

  double delta_s_ = 1.0;
  bool has_x_ref_ = false;
  double weight_x_ref_ = 0.0;
  bool has_end_state_ref_ = false;

  std::vector<double> inCircleY;
  std::vector<double> outCircleY;
  std::vector<double> inCircleY1;
  std::vector<double> outCircleY1;

  std::unique_ptr<Circle> innerCircle =
      std::make_unique<Circle>(2.5, 100.0, 5.0);
  std::unique_ptr<Circle> outerCircle =
      std::make_unique<Circle>(5.0, 100.0, 5.0);
};