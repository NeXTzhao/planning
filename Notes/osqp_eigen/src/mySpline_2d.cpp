/*
 * @Author: wangdezhao
 * @Date: 2022-04-29 21:46:38
 * @LastEditTime: 2022-05-01 10:32:37
 * @FilePath: /osqp_eigen/src/mySpline_2d.cpp
 * @Copyright:
 **/

#include "mySpline_2d.hpp"

#include "curve_math.h"
#include "matplotlibcpp.h"
#include "osqp_spline_2d_solver.h"

using apollo::common::math::Vec2d;
using Eigen::MatrixXd;
using namespace apollo;
using namespace planning;

template <typename T>
using vector_Eigen = std::vector<T, Eigen::aligned_allocator<T>>;

void matplot(std::vector<std::vector<double>>& points) {
  namespace plt = matplotlibcpp;

  plt::clf();
  plt::xlabel("x");
  plt::ylabel("y");
  plt::title("qp_spline");  //图片标题
                            // plt::named_plot("x_optermizer", x_,
  //                 "bo-");  //(取名，参数，参数，离散点)
  // plt::named_plot("y_optermizer", y_,
  //                 "g-");  //(取名，参数，参数，离散点)
  plt::named_plot("xy", points[0], points[1], "b-o");
  // plt::named_plot("ref", points[2], points[3], "r-o");

  plt::legend();
  plt::pause(0.1);
  plt::show();
}

int main() {
  std::vector<double> t_knots_;
  uint32_t num_spline = 2;
  for (std::uint32_t i = 0; i <= num_spline; ++i) {
    t_knots_.push_back(i * 1.0);
  }

  uint32_t order = 5;
  OsqpSpline2dSolver spline_solver(t_knots_, order);

  Spline2dConstraint* constraint = spline_solver.mutable_constraint();
  Spline2dKernel* kernel = spline_solver.mutable_kernel();

  std::vector<double> t_coord;
  vector_Eigen<Eigen::Vector2d> ref_ft;
  std::vector<Vec2d> ref_point;
  std::vector<double> ref_x, ref_y;
  std::vector<double> lateral_bound, longitudinal_bound;
  std::vector<double> ref_theta;
  // for (int i = 0; i < 12.5 / 0.5; ++i) {
  //   const double param = i * 0.5;
  //   t_coord.emplace_back(i * 0.5);
  //   double x1, y1;
  //   // if (i == 0 || i == 19) {
  //   //   x1 = spline(a, param);
  //   //   y1 = spline(b, param);
  //   // } else {
  //   // }
  //   x1 = spline(a, param);  // + NormalDistribution(0, 0.1)
  //   y1 = spline(b, param);  // + NormalDistribution(0, 0.1)
  //   double theta = std::atan2(spline_1st(b, param), spline_1st(a, param));
  //   std::cout << "theta:" << theta / 3.14 * 180 << std::endl;

  //   Vec2d prev_point(x1, y1);
  //   Vec2d new_point = prev_point;

  //   ref_x.push_back(new_point.x());
  //   ref_y.push_back(new_point.y());

  //   ref_point.emplace_back(new_point.x(), new_point.y());

  //   ref_theta.emplace_back(theta);
  //   longitudinal_bound.emplace_back(0.20);
  //   lateral_bound.emplace_back(0.20);
  // }

  std::vector<double> constraint_x{
      586385.8586, 586386.8586, 586387.8586, 586388.8586, 586389.8586,
      586390.8586, 586391.8586, 586392.8586, 586393.8586, 586394.8586,
      586395.8586, 586396.8586, 586397.8586, 586398.8586, 586399.8586,
      586400.8586, 586401.8586, 586402.8586, 586403.8586, 586404.8586,
      586405.8586, 586406.8586, 586407.8586, 586408.8586, 586409.8586,
      586410.8586, 586411.8586, 586412.8586, 586413.8586, 586414.8586,
      586415.8586, 586416.8586, 586417.8586, 586418.8586, 586419.8586,
      586420.8586, 586421.8586, 586422.8586, 586423.8586, 586424.8586};

  std::vector<double> constraint_y{
      4140674.736, 4140675.736, 4140676.736, 4140677.736, 4140678.736,
      4140678.736, 4140678.737, 4140679.736, 4140679.736, 4140679.736,
      4140679.736, 4140679.736, 4140680.736, 4140680.736, 4140680.736,
      4140680.736, 4140680.736, 4140680.736, 4140681.736, 4140681.736,
      4140681.736, 4140680.736, 4140680.736, 4140680.736, 4140679.736,
      4140679.736, 4140679.736, 4140679.736, 4140679.736, 4140679.736,
      4140679.736, 4140680.736, 4140680.736, 4140680.736, 4140680.736,
      4140680.736, 4140680.736, 4140680.736, 4140680.736, 4140680.736};
  std::vector<double> constraint_theta{
      2.834700688, 2.834700688, 2.834700688, 2.834700688, 2.834700688,
      2.834700688, 2.834700688, 2.834700688, 2.834700688, 2.834700688,
      2.834700688, 2.834700688, 2.834700688, 2.834700688, 2.834700688,
      2.834700688, 2.834700688, 2.834700688, 2.834700688, 2.834700688,
      2.834700688, 2.834700688, 2.834700688, 2.834700688, 2.834700688,
      2.834700688, 2.834700688, 2.834700688, 2.834700688, 2.834700688,
      2.834700688, 2.834700688, 2.834700688, 2.834700688, 2.834700688,
      2.834700688, 2.834700688, 2.834700688, 2.834700688, 2.834700688};

  for (size_t i = 0; i < constraint_x.size(); ++i) {
    t_coord.emplace_back(i * 0.01);
    ref_theta.push_back(constraint_theta[i]);
    Vec2d prev_point(constraint_x[i] - 586385.0, constraint_y[i] - 4140670.0);

    Vec2d new_point = prev_point;
    ref_x.push_back(new_point.x());
    ref_y.push_back(new_point.y());
    ref_point.emplace_back(new_point.x(), new_point.y());
    longitudinal_bound.emplace_back(0.15);
    lateral_bound.emplace_back(0.15);
  }

  constraint->Add2dBoundary(t_coord, ref_theta, ref_point, lateral_bound,
                            longitudinal_bound);
  constraint->AddThirdDerivativeSmoothConstraint();
  kernel->AddThirdOrderDerivativeMatrix(10);
  // kernel->AddSecondOrderDerivativeMatrix(100);
  // kernel->AddDerivativeKernelMatrix(100);

  kernel->AddRegularization(0.1);
  constraint->AddPointAngleConstraint(0, 2.834700688);
  // TODO(all): fix the test.
  auto start = std::chrono::system_clock::now();
  spline_solver.Solve();
  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = end - start;
  std::cout << "Time to solver is " << diff.count() << " s\n";

  std::vector<double> sloveX, sloveY;
  std::vector<double> ref_kappa, res_kappa;

  for (auto const t : t_coord) {
    auto xy = spline_solver.spline()(t);
    sloveX.emplace_back(xy.first);
    sloveY.emplace_back(xy.second);
    ref_kappa.emplace_back(CurveMath::ComputeCurvature(
        spline_solver.spline().DerivativeX(t),
        spline_solver.spline().SecondDerivativeX(t),
        spline_solver.spline().DerivativeY(t),
        spline_solver.spline().SecondDerivativeY(t)));
  }

  namespace plt = matplotlibcpp;
  plt::named_plot("sloveXY", sloveX, sloveY, "b-o");
  plt::named_plot("refXY", ref_x, ref_y, "r-o");
  plt::legend();
  plt::axis("equal");
  plt::figure();
  plt::named_plot("slove_kappa", t_coord, ref_kappa, "r-o");
  // plt::named_plot("ref_kappa", t_coord, constraint_curature, "g-o");
  plt::legend();
  plt::show();

  return 0;
};
