#include "curve_creator.h"

#include <ceres/ceres.h>
#include <glog/logging.h>

extern "C" void open_solve(double *thetas, double *kappas, double *x, double *y,
                           double *deltas, double *getvalue, int size);

namespace reference_line {
CurveCreator::CurveCreator(const std::vector<double> &a_xs,
                           const std::vector<double> &a_ys) {
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
  auto n = nodes_.size();
  double theta[n];
  double kappa[n];
  double x[n];
  double y[n];
  double delta[n];
  double getvalue[5 * n];

  for (int i = 0; i < n; ++i) {
    theta[i] = nodes_[i][0];
    kappa[i] = nodes_[i][1];
    x[i] = nodes_[i][2];
    y[i] = nodes_[i][3];
    if (i <= n - 1) {
      delta[i] = deltas_[i];
    }
//    printf("raw_x=%f , raw_y=%f , raw_theta=%f, raw_kappa=%f s=%f\n",
//           nodes_[i][2], nodes_[i][3], nodes_[i][0], nodes_[i][1], deltas_[i]);
  }
  delta[n - 1] = delta[n - 2];
  open_solve(theta, kappa, x, y, delta, getvalue, n);

  for (int i = 0; i < n; ++i) {
    int index = i * 5;
    solve_nodes.push_back(Node1{getvalue[index], getvalue[index + 1],
                                getvalue[index + 2], getvalue[index + 3],
                                getvalue[index + 4]});
    //    printf("getvalue_x=%f,getvalue_y=%f\n", getvalue[index + 2],
    //           getvalue[index + 3]);
  }

  std::vector<double> thetas;
  std::vector<double> kappas;
  std::vector<double> dkappas;
  std::vector<double> xs;
  std::vector<double> ys;
  std::vector<double> delta1;

  thetas.reserve(solve_nodes.size());
  kappas.reserve(solve_nodes.size());
  dkappas.reserve(solve_nodes.size());
  xs.reserve(solve_nodes.size());
  ys.reserve(solve_nodes.size());

  for (int i = 0; i < solve_nodes.size(); ++i) {
    thetas.emplace_back(solve_nodes[i][0]);
    kappas.emplace_back(solve_nodes[i][1]);
    xs.emplace_back(solve_nodes[i][2]);
    ys.emplace_back(solve_nodes[i][3]);
    dkappas.emplace_back(0.0);
    if (i <= solve_nodes.size() - 2) {
      delta1.emplace_back(solve_nodes[i][4]);
    }
  }
  curve_ = nullptr;
  curve_ = std::make_shared<Curve>(thetas, kappas, dkappas, xs, ys, delta1);
} // namespace reference_line
} // namespace reference_line