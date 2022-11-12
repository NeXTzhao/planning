/*
 * @Author: wangdezhao
 * @Date: 2022-04-12 17:40:03
 * @LastEditTime: 2022-04-12 17:44:55
 * @FilePath: /osqp_eigen/common/math/piecewise_jerk/src/piecewise_jerk_path_problem.cc
 * @Copyright:  
 */

// #include "cyber/common/log.h"

// #include "modules/planning/common/planning_gflags.h"
// #include "modules/planning/math/piecewise_jerk/piecewise_jerk_path_problem.h"
#include "piecewise_jerk_path_problem.h"


namespace planning {

PiecewiseJerkPathProblem::PiecewiseJerkPathProblem(
    const size_t num_of_knots, const double delta_s,
    const std::array<double, 3>& x_init)
    : PiecewiseJerkProblem(num_of_knots, delta_s, x_init) {}

void PiecewiseJerkPathProblem::CalculateKernel(std::vector<c_float>* P_data,
                                               std::vector<c_int>* P_indices,
                                               std::vector<c_int>* P_indptr) {
  const int n = static_cast<int>(num_of_knots_);
  const int num_of_variables = 3 * n;
  // const int num_of_nonzeros = num_of_variables + (n - 1);
  std::vector<std::vector<std::pair<c_int, c_float>>> columns(num_of_variables);
  int value_index = 0;

  // x(i)^2 * (w_x + w_x_ref)
  for (int i = 0; i < n - 1; ++i) {
    columns[i].emplace_back(
        i, (weight_x_ + weight_x_ref_) / (scale_factor_[0] * scale_factor_[0]));
    ++value_index;
  }
  // x(n-1)^2 * (w_x + w_x_ref + w_end_x)
  columns[n - 1].emplace_back(
      n - 1, (weight_x_ + weight_x_ref_ + weight_end_state_[0]) /
                 (scale_factor_[0] * scale_factor_[0]));
  ++value_index;

  // x(i)'^2 * w_dx
  for (int i = 0; i < n - 1; ++i) {
    columns[n + i].emplace_back(
        n + i, weight_dx_ / (scale_factor_[1] * scale_factor_[1]));
    ++value_index;
  }
  // x(n-1)'^2 * (w_dx + w_end_dx)
  columns[2 * n - 1].emplace_back(2 * n - 1,
                                  (weight_dx_ + weight_end_state_[1]) /
                                      (scale_factor_[1] * scale_factor_[1]));
  ++value_index;

  auto delta_s_square = delta_s_ * delta_s_;
  // x(i)''^2 * (w_ddx + 2 * w_dddx / delta_s^2)
  columns[2 * n].emplace_back(2 * n,
                              (weight_ddx_ + weight_dddx_ / delta_s_square) /
                                  (scale_factor_[2] * scale_factor_[2]));
  ++value_index;
  for (int i = 1; i < n - 1; ++i) {
    columns[2 * n + i].emplace_back(
        2 * n + i, (weight_ddx_ + 2.0 * weight_dddx_ / delta_s_square) /
                       (scale_factor_[2] * scale_factor_[2]));
    ++value_index;
  }
  columns[3 * n - 1].emplace_back(
      3 * n - 1,
      (weight_ddx_ + weight_dddx_ / delta_s_square + weight_end_state_[2]) /
          (scale_factor_[2] * scale_factor_[2]));
  ++value_index;

  // -2 * w_dddx / delta_s^2 * x(i)'' * x(i + 1)''
  for (int i = 0; i < n - 1; ++i) {
    columns[2 * n + i].emplace_back(2 * n + i + 1,
                                    (-2.0 * weight_dddx_ / delta_s_square) /
                                        (scale_factor_[2] * scale_factor_[2]));
    ++value_index;
  }


  int ind_p = 0;
  for (int i = 0; i < num_of_variables; ++i) {
    P_indptr->push_back(ind_p);
    for (const auto& row_data_pair : columns[i]) {
      P_data->push_back(row_data_pair.second * 2.0);
      P_indices->push_back(row_data_pair.first);
      ++ind_p;
    }
  }
  P_indptr->push_back(ind_p);
}

void PiecewiseJerkPathProblem::CalculateOffset(std::vector<c_float>* q) {
  const int n = static_cast<int>(num_of_knots_);
  const int kNumParam = 3 * n;
  q->resize(kNumParam, 0.0);

  if (has_x_ref_) {
    for (int i = 0; i < n; ++i) {
      q->at(i) += -2.0 * weight_x_ref_ * x_ref_[i] / scale_factor_[0];
    }
  }

  if (has_end_state_ref_) {
    q->at(n - 1) +=
        -2.0 * weight_end_state_[0] * end_state_ref_[0] / scale_factor_[0];
    q->at(2 * n - 1) +=
        -2.0 * weight_end_state_[1] * end_state_ref_[1] / scale_factor_[1];
    q->at(3 * n - 1) +=
        -2.0 * weight_end_state_[2] * end_state_ref_[2] / scale_factor_[2];
  }
}

}  // namespace planning
