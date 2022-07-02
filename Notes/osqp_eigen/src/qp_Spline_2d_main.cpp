/*
 * @Author: wangdezhao
 * @Date: 2022-04-29 21:46:38
 * @LastEditTime: 2022-05-01 10:32:37
 * @FilePath: /osqp_eigen/src/mySpline_2d.cpp
 * @Copyright:
 **/

#include <chrono>
#include <iostream>
#include <vector>

#include "../math/math_method/curve_math.h"
#include "matplotlibcpp.h"
#include "osqp_spline_2d_solver.h"

using apollo::common::math::Vec2d;
using Eigen::MatrixXd;
using namespace apollo;
using namespace planning;
namespace plt = matplotlibcpp;

template <typename T>
using vector_Eigen = std::vector<T, Eigen::aligned_allocator<T>>;

void SetPoints(std::vector<Eigen::Vector2d>& raw_points_) {
  raw_points_.resize(21);
  raw_points_[0] = {4.946317773, 0.08436953512};
  raw_points_[1] = {5.017218975, 0.7205757236};
  raw_points_[2] = {4.734635316, 1.642930209};
  raw_points_[3] = {4.425064575, 2.365356462};
  raw_points_[4] = {3.960102096, 2.991632152};
  raw_points_[5] = {3.503172702, 3.44091492};
  raw_points_[6] = {2.989950824, 3.9590821};
  raw_points_[7] = {2.258523535, 4.554377368};
  raw_points_[8] = {1.562447892, 4.656801472};
  raw_points_[9] = {0.8764776599, 4.971705856};
  raw_points_[10] = {0.09899323097, 4.985845841};
  raw_points_[11] = {-0.7132021974, 5.010851105};
  raw_points_[12] = {-1.479055426, 4.680181989};
  raw_points_[13] = {-2.170306775, 4.463442715};
  raw_points_[14] = {-3.034455492, 4.074651273};
  raw_points_[15] = {-3.621987909, 3.585790302};
  raw_points_[16] = {-3.979289889, 3.014232351};
  raw_points_[17] = {-4.434628966, 2.367848826};
  raw_points_[18] = {-4.818245921, 1.467395733};
  raw_points_[19] = {-4.860190444, 0.8444358019};
  raw_points_[20] = {-5.09947597, -0.01022405467};
}

double calc_point_theta(double y_x_t, double x) {
  return std::atan2(y_x_t, x);
  // return atan(y_x_t_d / x_d);
};

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
  std::vector<double> constraint_x, constraint_y;

  /*测试大曲率*/
  /*
  std::vector<double> constraint_x{
      4.946317773,   5.017218975,   4.734635316,  4.425064575,  3.960102096,
      3.503172702,   2.989950824,   2.258523535,  1.562447892,  0.8764776599,
      0.09899323097, -0.7132021974, -1.479055426, -2.170306775, -3.034455492,
      -3.621987909,  -3.979289889,  -4.434628966, -4.818245921, -4.860190444,
      -5.09947597};

  std::vector<double> constraint_y{
      0.08436953512, 0.7205757236, 1.642930209, 2.365356462, 2.991632152,
      3.44091492,    3.9590821,    4.554377368, 4.656801472, 4.971705856,
      4.985845841,   5.010851105,  4.680181989, 4.463442715, 4.074651273,
      3.585790302,   3.014232351,  2.367848826, 1.467395733, 0.8444358019,
      -0.01022405467};
  */
  /*
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
  */
  std::vector<Eigen::Vector2d> raw_points_;
  SetPoints(raw_points_);

  constraint_x.clear();
  constraint_y.clear();

  for (size_t i = 0; i < raw_points_.size(); ++i) {
    constraint_x.push_back(raw_points_.at(i).x());
    constraint_y.push_back(raw_points_.at(i).y());

    t_coord.emplace_back(i * 0.25);
    Vec2d prev_point(constraint_x[i], constraint_y[i]);

    Vec2d new_point = prev_point;
    ref_x.push_back(new_point.x());
    ref_y.push_back(new_point.y());
    ref_point.emplace_back(new_point.x(), new_point.y());
    longitudinal_bound.emplace_back(0.2);
    lateral_bound.emplace_back(0.2);
  }

  for (size_t i = 0; i < constraint_x.size() - 1; ++i) {
    double delta_y = constraint_y.at(i + 1) - constraint_y.at(i);
    double delta_x = constraint_x.at(i + 1) - constraint_x.at(i);
    double theta = calc_point_theta(delta_y, delta_x);
    std::cout << "theta:" << theta << "\n";
    ref_theta.push_back(theta);
  }

  ref_theta.push_back(
      calc_point_theta(constraint_y.at(constraint_y.size() - 1) -
                           constraint_y.at(constraint_y.size() - 2),
                       constraint_x.at(constraint_x.size() - 1) -
                           constraint_x.at(constraint_x.size() - 2)));

  constraint->Add2dBoundary(t_coord, ref_theta, ref_point, lateral_bound,
                            longitudinal_bound);
  constraint->AddThirdDerivativeSmoothConstraint();
  kernel->AddThirdOrderDerivativeMatrix(200);
  kernel->AddSecondOrderDerivativeMatrix(100);
  kernel->AddDerivativeKernelMatrix(10);

  // 正则化
  kernel->AddRegularization(100);
  constraint->AddPointAngleConstraint(0, 1.45981);
  // TODO(all): fix the test.
  auto start = std::chrono::system_clock::now();
  if (spline_solver.Solve()) {
    std::cout << "solve successful" << std::endl;
  }
  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = end - start;
  std::cout << "Time to solver is " << diff.count() << " s\n";

  std::vector<double> sloveX, sloveY;
  std::vector<double> ref_kappa, res_kappa;

  for (auto const t : t_coord) {
    std::cout << "t is " << t << " s\n";

    auto xy = spline_solver.spline()(t);
    sloveX.emplace_back(xy.first);
    sloveY.emplace_back(xy.second);
    std::cout << "xy.first:" << xy.first << "\n";
    std::cout << "xy.second:" << xy.second << "\n";
    // ref_kappa.emplace_back(CurveMath::ComputeCurvature(
    //     spline_solver.spline().DerivativeX(t),
    //     spline_solver.spline().SecondDerivativeX(t),
    //     spline_solver.spline().DerivativeY(t),
    //     spline_solver.spline().SecondDerivativeY(t)));
  }
  /***************************************************************************/
  std::vector<double> ref_x_1;
  std::vector<double> ref_y_1;
  for (const auto& item : raw_points_) {
    ref_x_1.push_back(item.x());
    ref_y_1.push_back(item.y());
  }
  plt::named_plot("sloveXY", sloveX, sloveY, "b");
  plt::named_plot("refXY", ref_x_1, ref_y_1, "r");
  plt::legend();
  plt::axis("equal");
  // plt::figure();
  // plt::named_plot("slove_kappa", t_coord, ref_kappa, "r-o");
  // plt::named_plot("ref_kappa", t_coord, constraint_curature, "g-o");
  plt::legend();
  plt::show();

  return 0;
};