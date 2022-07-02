#include "curve_creator.h"

#include <ceres/ceres.h>
#include <glog/logging.h>

namespace reference_line {
CurveCreator::CurveCreator(const std::vector<double> a_xs,
                           const std::vector<double> a_ys) {
  // reference_line::Curve curve(a_xs, a_ys);
  // auto cure_temp = std::make_shared<reference_line::Curve>(a_xs, a_ys);
  // curve_ = cure_temp;

  curve_ = std::make_shared<Curve>(a_xs, a_ys);
  // (theta kappa x y) by n and delta by n-1
  auto &thetas = curve_->thetas();
  auto &kappas = curve_->kappas();
  auto &dkappas = curve_->dkappas();
  auto &xs = curve_->xs();
  auto &ys = curve_->ys();
  nodes_.reserve(xs.size());
  for (size_t i = 0; i < xs.size(); i++) {
    nodes_.emplace_back(Node{thetas[i], kappas[i], dkappas[i], xs[i], ys[i]});
  }
  deltas_ = curve_->deltas();
}

void CurveCreator::solve() {
  using ceres::AutoDiffCostFunction;
  using ceres::CostFunction;
  using ceres::Problem;
  using ceres::Solve;
  using ceres::Solver;

  Problem problem;

  int n = nodes_.size();
// CostFunction *cost_xy_error = new xy_error_functor();
  CostFunction *cost_xy_error =
      new AutoDiffCostFunction<xy_error_functor, 2, 5, 5, 1>(
          new xy_error_functor());
  // CostFunction *cost_objective_functor =
  //     new AutoDiffCostFunction<objective_functor, 1, 5, 5, 1>(
  //         new objective_functor());
  // CostFunction *cost_objective_functor =
  //     new AutoDiffCostFunction<objective_functor, 1, 1>(
  //         new objective_functor());
  /* CostFunction *cost_connection_point =
       new AutoDiffCostFunction<connection_point_functor, 3, 5, 5, 5, 1>(
           new connection_point_functor());*/
  /* CostFunction *cost_function4 =
      new AutoDiffCostFunction<objective_functor, 1, 1>(
          new objective_functor(curve_));*/

  for (int i = 0; i + 1 < n; i++) {
    auto &node0 = nodes_[i];
    auto &node1 = nodes_[i + 1];

    problem.AddResidualBlock(cost_xy_error, nullptr, &node0[0], &node1[0],
                             &deltas_[i]);

    // problem.AddResidualBlock(cost_objective_functor, nullptr, &node0[0],
    //                          &node1[0], &deltas_[i]);
    // problem.AddResidualBlock(cost_objective_functor, nullptr, &deltas_[i]);
    //  problem.AddResidualBlock(cost_function4, nullptr, &deltas_[0]);

    //  if (i <= n - 3) {
    //   auto &node2 = nodes_[i + 2];
    //   problem.AddResidualBlock(cost_function1, nullptr, &node0[0], &node1[0],
    //                            &deltas_[i]);

    //   problem.AddResidualBlock(cost_connection_point, nullptr, &node0[0],
    //                            &node1[0], &node2[0], &deltas_[i]);
    // }
    // else if (i <= n - 2) {
    //   auto &node1 = nodes_[i + 1];
    //   problem.AddResidualBlock(cost_function1, nullptr, &node0[0], &node1[0],
    //                            &deltas_[i]);
    //   problem.AddResidualBlock(cost_function2, nullptr, &node0[0], &node1[0],
    //                            &deltas_[i]);
    // } else {
    //   break;
    // }
  }

  // for (const auto &item : deltas_) {
  //   std::cout << "deltas::" << item << "\n";
  // }
/*
  for (int i = 0; i < n; i++) {
    // address of values must be set in AddResidualBlock
    auto &node = nodes_[i];

    // Node{thetas[i], kappas[i], dkappas[i], xs[i], ys[i]};
    if (i == 0 || i == n - 1) {
      // 设置起点终点边界
      problem.SetParameterLowerBound(&node[0], 0, node[0] - 1e-6);
      problem.SetParameterUpperBound(&node[0], 0, node[0] + 1e-6);
      problem.SetParameterLowerBound(&node[0], 1, node[1] - 1e-6);
      problem.SetParameterUpperBound(&node[0], 1, node[1] + 1e-6);
      problem.SetParameterLowerBound(&node[0], 2, node[2] - 1e-6);
      problem.SetParameterUpperBound(&node[0], 2, node[2] + 1e-6);
      problem.SetParameterLowerBound(&node[0], 3, node[3] - 1e-6);
      problem.SetParameterUpperBound(&node[0], 3, node[3] + 1e-6);
      problem.SetParameterLowerBound(&node[0], 4, node[4] - 1e-6);
      problem.SetParameterUpperBound(&node[0], 4, node[4] + 1e-6);
    } else {
      auto &node0 = nodes_[i - 1];
      auto &node1 = nodes_[i + 1];
      problem.SetParameterLowerBound(&node[0], 0, node0[0] - M_PI_2);
      problem.SetParameterUpperBound(&node[0], 0, node0[0] + M_PI_2);
      // problem.SetParameterLowerBound(&node1[0], 0, node[0] - 1e-6);
      // problem.SetParameterUpperBound(&node1[0], 0, node[0] + 1e-6);

      problem.SetParameterLowerBound(&node[0], 1, node[1] - 0.25);
      problem.SetParameterUpperBound(&node[0], 1, node[1] + 0.25);
      // problem.SetParameterLowerBound(&node1[0], 1, node[0] - 1e-6);
      // problem.SetParameterUpperBound(&node1[0], 1, node[0] + 1e-6);

      problem.SetParameterLowerBound(&node[0], 2, node[2] - 0.02);
      problem.SetParameterUpperBound(&node[0], 2, node[2] + 0.02);
      // problem.SetParameterLowerBound(&node1[0], 2, node[0] - 1e-6);
      // problem.SetParameterUpperBound(&node1[0], 2, node[0] + 1e-6);

      problem.SetParameterLowerBound(&node[0], 3, node[3] - 0.1);
      problem.SetParameterUpperBound(&node[0], 3, node[3] + 0.1);

      problem.SetParameterLowerBound(&node[0], 4, node[4] - 0.1);
      problem.SetParameterUpperBound(&node[0], 4, node[4] + 0.1);

    }

    if (i <= n - 2) {
      problem.SetParameterLowerBound(&deltas_[i], 0, deltas_[i] - 2.0 * 0.1);
      problem.SetParameterUpperBound(&deltas_[i], 0, deltas_[i] * M_PI_2);
    }
  }
*/
  Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  Solver::Summary summary;
  Solve(options, &problem, &summary);

  std::cout << summary.BriefReport() << '\n';

  std::vector<double> thetas;
  std::vector<double> kappas;
  std::vector<double> dkappas;
  std::vector<double> xs;
  std::vector<double> ys;

  thetas.reserve(nodes_.size());
  kappas.reserve(nodes_.size());
  dkappas.reserve(nodes_.size());
  xs.reserve(nodes_.size());
  ys.reserve(nodes_.size());

  for (const auto &node : nodes_) {
    thetas.emplace_back(node[0]);
    kappas.emplace_back(node[1]);
    dkappas.emplace_back(node[2]);
    xs.emplace_back(node[3]);
    ys.emplace_back(node[4]);
  }
  curve_ = nullptr;
  curve_ = std::make_shared<Curve>(thetas, kappas, dkappas, xs, ys, deltas_);

  return;
}  // namespace reference_line
}  // namespace reference_line