#include "curve_creator.h"

#include <ceres/ceres.h>
#include <glog/logging.h>

namespace reference_line {
CurveCreator::CurveCreator(const std::vector<double> a_xs,
                           const std::vector<double> a_ys) {
  curve_ = std::make_shared<Curve>(a_xs, a_ys);
  // (theta kappa x y) by n and delta by n-1
  auto &thetas = curve_->thetas();
  auto &kappas = curve_->kappas();
  auto &dkappas = curve_->dkappas();
  auto &xs = curve_->xs();
  auto &ys = curve_->ys();
  nodes_.reserve(xs.size());
  deltas_ = curve_->deltas();
  for (size_t i = 0; i < xs.size(); i++) {
    nodes_.emplace_back(Node{thetas[i], kappas[i], xs[i], ys[i]});
  }
}

void CurveCreator::solve() {
  using ceres::AutoDiffCostFunction;
  using ceres::CostFunction;
  using ceres::Problem;
  using ceres::Solve;
  using ceres::Solver;

  Problem problem;

  auto n = nodes_.size();

  CostFunction *cost_xy_error = new xy_error_functor();
  for (int i = 0; i + 1 < n; i++) {
    auto &node0 = nodes_[i];
    auto &node1 = nodes_[i + 1];

    problem.AddResidualBlock(cost_xy_error, nullptr, &node0[0], &node1[0],
                             &deltas_[i]);
  }

  for (int i = 0; i < n; i++) {
    auto &node = nodes_[i];

    problem.SetParameterLowerBound(&node[0], 2, node[2] - 0.1);
    problem.SetParameterUpperBound(&node[0], 2, node[2] + 0.1);

    problem.SetParameterLowerBound(&node[0], 3, node[3] - 0.1);
    problem.SetParameterUpperBound(&node[0], 3, node[3] + 0.1);
  }

  Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  Solver::Summary summary;

  auto time1 = std::chrono::system_clock::now();
  Solve(options, &problem, &summary);
  auto time2 = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = time2 - time1;
  std::cout << "Time for solve time = " << diff.count() * 1000 << " msec.\n";

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
    xs.emplace_back(node[2]);
    ys.emplace_back(node[3]);
    dkappas.emplace_back(0.0);
  }
  curve_ = nullptr;
  curve_ = std::make_shared<Curve>(thetas, kappas, dkappas, xs, ys, deltas_);

  return;
}  // namespace reference_line
}  // namespace reference_line