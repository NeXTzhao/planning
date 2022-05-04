/*
 * @Author: wangdezhao
 * @Date: 2022-04-13 11:44:55
 * @LastEditTime: 2022-04-30 11:44:52
 * @FilePath: /osqp_eigen/src/PathOptimize.cpp
 * @Copyright:
 */
#include "PathOptimize.hpp"

//构造p矩阵
void PathOptimize::CalculateKernel(std::vector<c_float>* P_data,
                                   std::vector<c_int>* P_indices,
                                   std::vector<c_int>* P_indptr) {
  const int n = static_cast<int>(num_of_knots_);
  const int num_of_variables = 3 * n;
  std::cout << "num_of_variables:" << num_of_variables << '\n';

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
  std::cout << "P_data:" << P_data->size() << std::endl;
  std::cout << "P_indptr:" << P_indptr->size() << std::endl;
}

//构造q矩阵
void PathOptimize::CalculateOffset(std::vector<c_float>* q) {
  const int n = static_cast<int>(num_of_knots_);
  // std::cout << "n:" << n << '\n';
  const int kNumParam = 3 * n;
  std::cout << "kNumParam:" << kNumParam << '\n';

  q->resize(kNumParam, 0.0);

  if (has_x_ref_) {
    for (int i = 0; i < n; ++i) {
      q->at(i) += -2.0 * weight_x_ref_ * x_ref_[i] / scale_factor_[0];
    }
  }
  bool has_end_state_ref_ = false;
  if (has_end_state_ref_) {
    q->at(n - 1) +=
        -2.0 * weight_end_state_[0] * end_state_ref_[0] / scale_factor_[0];
    q->at(2 * n - 1) +=
        -2.0 * weight_end_state_[1] * end_state_ref_[1] / scale_factor_[1];
    q->at(3 * n - 1) +=
        -2.0 * weight_end_state_[2] * end_state_ref_[2] / scale_factor_[2];
  }
}

//构造A矩阵及上下边界
void PathOptimize::CalculateAffineConstraint(
    std::vector<c_float>* A_data, std::vector<c_int>* A_indices,
    std::vector<c_int>* A_indptr, std::vector<c_float>* lower_bounds,
    std::vector<c_float>* upper_bounds) {
  // 3N params bounds on x, x', x''
  // 3(N-1) constraints on x, x', x''
  // 3 constraints on x_init_
  const int n = static_cast<int>(num_of_knots_);
  const int num_of_variables = 3 * n;
  const int num_of_constraints = num_of_variables + 3 * (n - 1) + 3;
  lower_bounds->resize(num_of_constraints);
  upper_bounds->resize(num_of_constraints);

  std::vector<std::vector<std::pair<c_int, c_float>>> variables(
      num_of_variables);

  int constraint_index = 0;
  // set x, x', x'' bounds
  for (int i = 0; i < num_of_variables; ++i) {
    if (i < n) {
      variables[i].emplace_back(constraint_index, 1.0);
      lower_bounds->at(constraint_index) =
          x_bounds_[i].first * scale_factor_[0];
      upper_bounds->at(constraint_index) =
          x_bounds_[i].second * scale_factor_[0];
    } else if (i < 2 * n) {
      variables[i].emplace_back(constraint_index, 1.0);

      lower_bounds->at(constraint_index) =
          dx_bounds_[i - n].first * scale_factor_[1];
      upper_bounds->at(constraint_index) =
          dx_bounds_[i - n].second * scale_factor_[1];
    } else {
      variables[i].emplace_back(constraint_index, 1.0);
      lower_bounds->at(constraint_index) =
          ddx_bounds_[i - 2 * n].first * scale_factor_[2];
      upper_bounds->at(constraint_index) =
          ddx_bounds_[i - 2 * n].second * scale_factor_[2];
    }
    ++constraint_index;
  }

  // x(i->i+1)''' = (x(i+1)'' - x(i)'') / delta_s
  for (int i = 0; i + 1 < n; ++i) {
    variables[2 * n + i].emplace_back(constraint_index, -1.0);
    variables[2 * n + i + 1].emplace_back(constraint_index, 1.0);
    lower_bounds->at(constraint_index) =
        dddx_bound_.first * delta_s_ * scale_factor_[2];
    upper_bounds->at(constraint_index) =
        dddx_bound_.second * delta_s_ * scale_factor_[2];
    ++constraint_index;
  }

  // x(i+1)' - x(i)' - 0.5 * delta_s * x(i)'' - 0.5 * delta_s * x(i+1)'' = 0
  for (int i = 0; i + 1 < n; ++i) {
    variables[n + i].emplace_back(constraint_index, -1.0 * scale_factor_[2]);
    variables[n + i + 1].emplace_back(constraint_index, 1.0 * scale_factor_[2]);
    variables[2 * n + i].emplace_back(constraint_index,
                                      -0.5 * delta_s_ * scale_factor_[1]);
    variables[2 * n + i + 1].emplace_back(constraint_index,
                                          -0.5 * delta_s_ * scale_factor_[1]);
    lower_bounds->at(constraint_index) = 0.0;
    upper_bounds->at(constraint_index) = 0.0;
    ++constraint_index;
  }

  // x(i+1) - x(i) - delta_s * x(i)'
  // - 1/3 * delta_s^2 * x(i)'' - 1/6 * delta_s^2 * x(i+1)''
  auto delta_s_sq_ = delta_s_ * delta_s_;
  for (int i = 0; i + 1 < n; ++i) {
    variables[i].emplace_back(constraint_index,
                              -1.0 * scale_factor_[1] * scale_factor_[2]);
    variables[i + 1].emplace_back(constraint_index,
                                  1.0 * scale_factor_[1] * scale_factor_[2]);
    variables[n + i].emplace_back(
        constraint_index, -delta_s_ * scale_factor_[0] * scale_factor_[2]);
    variables[2 * n + i].emplace_back(
        constraint_index,
        -delta_s_sq_ / 3.0 * scale_factor_[0] * scale_factor_[1]);
    variables[2 * n + i + 1].emplace_back(
        constraint_index,
        -delta_s_sq_ / 6.0 * scale_factor_[0] * scale_factor_[1]);

    lower_bounds->at(constraint_index) = 0.0;
    upper_bounds->at(constraint_index) = 0.0;
    ++constraint_index;
  }

  // constrain on x_init
  variables[0].emplace_back(constraint_index, 1.0);
  lower_bounds->at(constraint_index) = x_init_[0] * scale_factor_[0];
  upper_bounds->at(constraint_index) = x_init_[0] * scale_factor_[0];
  ++constraint_index;

  variables[n].emplace_back(constraint_index, 1.0);
  lower_bounds->at(constraint_index) = x_init_[1] * scale_factor_[1];
  upper_bounds->at(constraint_index) = x_init_[1] * scale_factor_[1];
  ++constraint_index;

  variables[2 * n].emplace_back(constraint_index, 1.0);
  lower_bounds->at(constraint_index) = x_init_[2] * scale_factor_[2];
  upper_bounds->at(constraint_index) = x_init_[2] * scale_factor_[2];
  ++constraint_index;

  int ind_p = 0;
  for (int i = 0; i < num_of_variables; ++i) {
    A_indptr->push_back(ind_p);
    for (const auto& variable_nz : variables[i]) {
      // coefficient
      A_data->push_back(variable_nz.second);

      // constraint index
      A_indices->push_back(variable_nz.first);
      ++ind_p;
    }
  }
  // We indeed need this line because of
  // https://github.com/oxfordcontrol/osqp/blob/master/src/cs.c#L255
  A_indptr->push_back(ind_p);
}

OSQPData* PathOptimize::FormulateProblem() {
  // calculate kernel
  std::vector<c_float> P_data;
  std::vector<c_int> P_indices;
  std::vector<c_int> P_indptr;
  CalculateKernel(&P_data, &P_indices, &P_indptr);

  // calculate affine constraints
  std::vector<c_float> A_data;
  std::vector<c_int> A_indices;
  std::vector<c_int> A_indptr;
  std::vector<c_float> lower_bounds;
  std::vector<c_float> upper_bounds;
  CalculateAffineConstraint(&A_data, &A_indices, &A_indptr, &lower_bounds,
                            &upper_bounds);

  // calculate offset
  std::vector<c_float> q;
  CalculateOffset(&q);
  OSQPData* data = reinterpret_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));

  size_t kernel_dim = 3 * num_of_knots_;
  size_t num_affine_constraint = lower_bounds.size();
  std::cout << "m:" << num_affine_constraint << '\n'
            << "n:" << kernel_dim << '\n';

  data->m = num_affine_constraint;
  data->n = kernel_dim;
  data->P = csc_matrix(kernel_dim, kernel_dim, P_data.size(), CopyData(P_data),
                       CopyData(P_indices), CopyData(P_indptr));

  data->q = CopyData(q);
  data->A =
      csc_matrix(num_affine_constraint, kernel_dim, A_data.size(),
                 CopyData(A_data), CopyData(A_indices), CopyData(A_indptr));
  data->l = CopyData(lower_bounds);
  data->u = CopyData(upper_bounds);

  return data;
}

OSQPSettings* PathOptimize::SolverDefaultSettings() {
  // Define Solver default settings
  OSQPSettings* settings = (OSQPSettings*)c_malloc(sizeof(OSQPSettings));

  if (settings) osqp_set_default_settings(settings);

  settings->alpha = 1.0;
  settings->polish = true;
  settings->verbose = true;
  settings->scaled_termination = true;
  return settings;
}

void PathOptimize::Optimize() {
  OSQPData* data = FormulateProblem();

  OSQPSettings* settings = SolverDefaultSettings();
  // settings->max_iter = 4000;

  OSQPWorkspace* osqp_work = osqp_setup(data, settings);

  osqp_solve(osqp_work);

  // auto status = osqp_work->info->status_val;

  // extract primal results
  x_.resize(num_of_knots_);
  dx_.resize(num_of_knots_);
  ddx_.resize(num_of_knots_);
  for (size_t i = 0; i < num_of_knots_; ++i) {
    x_.at(i) = osqp_work->solution->x[i] / scale_factor_[0];
    dx_.at(i) = osqp_work->solution->x[i + num_of_knots_] / scale_factor_[1];
    ddx_.at(i) =
        osqp_work->solution->x[i + 2 * num_of_knots_] / scale_factor_[2];
  }

  // Clean workspace
  osqp_cleanup(osqp_work);
  if (data) {
    if (data->A) c_free(data->A);
    if (data->P) c_free(data->P);
    c_free(data);
  }
  if (settings) c_free(settings);
}

void PathOptimize::set_x_bounds(
    std::vector<std::pair<double, double>> x_bounds) {
  x_bounds_ = std::move(x_bounds);
}

void PathOptimize::set_dx_bounds(
    std::vector<std::pair<double, double>> dx_bounds) {
  dx_bounds_ = std::move(dx_bounds);
}

void PathOptimize::set_ddx_bounds(
    std::vector<std::pair<double, double>> ddx_bounds) {
  ddx_bounds_ = std::move(ddx_bounds);
}

void PathOptimize::set_x_bounds(const double x_lower_bound,
                                const double x_upper_bound) {
  // for (int i = 0; i < (num_of_knots_ / 5); ++i) {
  /* code */
  // x_bounds_.push_back(std::make_pair(x_lower_bound,x_upper_bound));
  // replace(x_bounds_.begin(), x_bounds_.begin() + 20,
  //         std::make_pair(-kMaxVariableRange, kMaxVariableRange),
  //         std::make_pair(x_lower_bound, x_upper_bound));  //{1,8,5,8,9}
  // // }
  // replace(x_bounds_.begin()+20, x_bounds_.begin() + 40,
  //         std::make_pair(-kMaxVariableRange, kMaxVariableRange),
  //         std::make_pair(x_lower_bound, x_upper_bound));
  replace(x_bounds_.begin(), x_bounds_.begin() + 20,
          std::make_pair(-kMaxVariableRange, kMaxVariableRange),
          std::make_pair(0.0, 2.5));  //{1,8,5,8,9}

  replace(x_bounds_.begin() + 25, x_bounds_.begin() + 40,
          std::make_pair(-kMaxVariableRange, kMaxVariableRange),
          std::make_pair(-2.5, 0.0));  //{1,8,5,8,9}

  replace(x_bounds_.begin() + 40, x_bounds_.begin() + 60,
          std::make_pair(-kMaxVariableRange, kMaxVariableRange),
          std::make_pair(-2.5, 0.0));  //{1,8,5,8,9}

  replace(x_bounds_.begin() + 65, x_bounds_.begin() + 80,
          std::make_pair(-kMaxVariableRange, kMaxVariableRange),
          std::make_pair(0.0, 2.5));  //{1,8,5,8,9}

  replace(x_bounds_.begin() + 80, x_bounds_.begin() + 100,
          std::make_pair(-kMaxVariableRange, kMaxVariableRange),
          std::make_pair(0.0, 2.5));  //{1,8,5,8,9}

  std::vector<std::pair<double, double>> cr;
  for (size_t i = 0; i < inCircleY.size(); ++i) {
    cr.push_back(std::make_pair(inCircleY.at(i), outCircleY.at(i)));
    std::cout << "cr:" << cr[i].second << "," << cr[i].first << '\n';
  }

  int cit = 0;
  for (size_t it = 100; it < 100 + inCircleY.size(); ++it) {
    x_bounds_[it].first = cr[cit].second;
    x_bounds_[it].second = cr[cit].first;
    std::cout << "x_bounds_[it]:" << x_bounds_[it].first << ","
              << x_bounds_[it].second << '\n';
    ++cit;
  }
  // // for (auto& x : x_bounds_) {
  //   x.first = x_lower_bound;
  //   x.second = x_upper_bound;
  // }
}

void PathOptimize::set_dx_bounds(const double dx_lower_bound,
                                 const double dx_upper_bound) {
  for (auto& x : dx_bounds_) {
    x.first = dx_lower_bound;
    x.second = dx_upper_bound;
  }
}

void PathOptimize::set_ddx_bounds(const double ddx_lower_bound,
                                  const double ddx_upper_bound) {
  for (auto& x : ddx_bounds_) {
    x.first = ddx_lower_bound;
    x.second = ddx_upper_bound;
  }
}

void PathOptimize::set_x_ref(const double weight_x_ref,
                             std::vector<double> x_ref) {
  weight_x_ref_ = weight_x_ref;
  x_ref_ = std::move(x_ref);
  has_x_ref_ = true;
}

// void PathOptimize::set_end_state_ref(
//     const std::array<double, 3>& weight_end_state,
//     const std::array<double, 3>& end_state_ref) {
//   weight_end_state_ = weight_end_state;
//   end_state_ref_ = end_state_ref;
//   has_end_state_ref_ = true;
// }
void PathOptimize::set_bound(size_t numPoints, const std::array<double, 4>& w,
                             const std::vector<double>& new_values) {
  std::cout << "num_of_knots:" << num_of_knots_ << '\n';
  x_bounds_.resize(num_of_knots_,
                   std::make_pair(-kMaxVariableRange, kMaxVariableRange));
  std::cout << "x_bound:" << x_bounds_.size() << '\n';
  dx_bounds_.resize(num_of_knots_,
                    std::make_pair(-kMaxVariableRange, kMaxVariableRange));

  ddx_bounds_.resize(num_of_knots_,
                     std::make_pair(-kMaxVariableRange, kMaxVariableRange));
  // const double veh_param = 10.0;
  const double lat_acc_bound =
      std::tan(max_steer_angle / steer_ratio) / wheel_base;

  auto temp = innerCircle->getYvalue(100.0, 0.25);
  // auto temp1 = innerCircle->getYvalue1(100.0, 0.25);
  auto temp2 = outerCircle->getYvalue(100.0);
  // auto temp3 = outerCircle->getYvalue1(100.0);

  // auto temp1 = OuterCircle->getYvalue(100.0);
  inCircleY = temp;
  // inCircleY1 = temp1;

  // inCircleY.insert(inCircleY.end(), temp1.begin(), temp1.end());

  // auto temp1 = OuterCircle->getYvalue(100.0);
  outCircleY = temp2;
  // outCircleY1 = temp3;

  // outCircleY.insert(outCircleY.end(), temp3.begin(), temp3.end());

  set_x_bounds(-5, 5);

  set_dx_bounds(-FLAGS_lateral_derivative_bound_default,
                FLAGS_lateral_derivative_bound_default);
  set_ddx_bounds(-lat_acc_bound, lat_acc_bound);
  set_dddx_bound(-FLAGS_lateral_jerk_bound, FLAGS_lateral_jerk_bound);
  set_weight_x(w[0]);
  set_weight_dx(w[1]);
  set_weight_ddx(w[2]);
  set_weight_dddx(w[3]);
  // std::vector<double> xpoint = {1,  2,  3,  4,  5,  6,   7,   8,  9,  10,
  //                               1,  0,  2,  0,  3,  0,   3,   0,  3,  0,
  //                               10, 15, 30, 50, 60, 30,  10,  15, 0,  2,
  //                               3,  1,  5,  9,  -1, -10, -25, 30, -35};
  std::vector<double> xpoint;
  for (size_t i = 0; i < x_bounds_.size(); ++i) {
    auto item = (x_bounds_[i].first + x_bounds_[i].second) * 0.5;
    xpoint.push_back(item);
  }
  set_x_ref(500, xpoint);
}

void PathOptimize::matplot() {
  namespace plt = matplotlibcpp;

  plt::clf();
  plt::xlabel("x");
  plt::ylabel("y");
  plt::title("PathOptimize");  //图片标题
  plt::named_plot("x_optermizer", x_,
                  "bo-");  //(取名，参数，参数，离散点)
  plt::named_plot("x_optermizer", dx_,
                  "g-");  //(取名，参数，参数，离散点)
  std::vector<double> l_bound;
  std::vector<double> u_bound;

  for (const auto& item : x_bounds_) {
    l_bound.push_back(item.first);
  }
  for (const auto& item : x_bounds_) {
    u_bound.push_back(item.second);
  }

  plt::named_plot("l_bound", l_bound,
                  "r*");  //(取名，参数，参数，离散点)
  plt::named_plot("u_bound", u_bound,
                  "r*");  //(取名，参数，参数，离散点)

  // plt::named_plot("u_bound", inCircleY, "b*");  //(取名，参数，参数，离散点)
  // plt::named_plot("u_bound", inCircleY1, "b*");
  //(取名，参数，参数，离散点) plt::named_plot("u_bound", outCircleY, "r*");
  //(取名，参数，参数，离散点)
  // plt::named_plot("u_bound", outCircleY1, "r*");
  //(取名，参数，参数，离散点)

  plt::legend();
  plt::pause(0.1);
  plt::show();

  const char* filename = "../result_picture/pathoptimizer.png";
  std::cout << "Saving result to " << filename << std::endl;
  plt::save(filename);
}