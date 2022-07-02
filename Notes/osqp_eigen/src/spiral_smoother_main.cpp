/**
 * @file spiral_smoother_main.cpp
 * @brief 对比各个平滑器的耗时以及拟合效果
 * @author Wang Dezhao (1282507109@qq.com)
 * @version 1.0
 * @date 2022-06-07 15:30:50
 *
 * @copyright Copyright (c) 2022
 */

#include <chrono>
#include <iostream>

//#include "../math/math_method/curve_math.h"
#include "curve.h"
#include "curve_creator.h"
#include "curve_segment.h"
#include "matplotlibcpp.h"
#include "osqp_spline_2d_solver.h"
#include "spiral_reference_line_smoother.h"

using apollo::common::math::Vec2d;
using Eigen::MatrixXd;
using namespace apollo;
using namespace planning;
namespace plt = matplotlibcpp;

template <typename T>
using vector_Eigen = std::vector<T, Eigen::aligned_allocator<T>>;

template <typename T>
T calc_point_theta(T y_x_t, T x) {
  return std::atan2(y_x_t, x);
};

namespace apollo::planning {
/**
 * @brief 等距弧长离散园
 * 由于圆的特殊性，其圆弧上每一点的曲率均相等，故可以将等弧长与相同的角度相对应
 * @param  x
 * @param  y
 * @param  r
 * @param  size
 */
void div_circle(std::vector<double>& x, std::vector<double>& y, double r,
                double size)  // xy对应圆心坐标,r为半径,size用于设置划分的间距
{
  double angle_step = 0;  //一小步的弧度
  angle_step = size / r;
  double x_out, y_out;

  for (int i = M_PI / angle_step; i > 0; --i) {
    x_out = r * cos(i * angle_step);
    y_out = r * sin(i * angle_step);
    x.push_back(x_out);
    y.push_back(y_out);
  }
}
}  // namespace apollo::planning

void SetPoints(std::vector<Eigen::Vector2d>& raw_points_) {
  /*
  raw_points_.resize(4);
  raw_points_[0] = {0.0, 0.0};
  raw_points_[1] = {1.0, 0.3};
  raw_points_[2] = {2.0, 0.0};
  raw_points_[3] = {3, 0.1};
  */
  std::vector<double> raw_xs, raw_ys;
  div_circle(raw_xs, raw_ys, 5, 1);

  raw_points_.resize(raw_xs.size());

  for (size_t i = 0; i < raw_xs.size(); ++i) {
    raw_points_[i] = {raw_xs[i], raw_ys[i]};
  }
  /*
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
    */
}

namespace apollo::planning {
void qp_spline_smoother(const std::vector<Eigen::Vector2d>& raw_points_,
                        std::vector<double>& sloveX,
                        std::vector<double>& sloveY) {
  // qp_spline
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
    // std::cout << "theta:" << theta << "\n";
    ref_theta.push_back(theta);
  }

  ref_theta.push_back(
      calc_point_theta(constraint_y.at(constraint_y.size() - 1) -
                           constraint_y.at(constraint_y.size() - 2),
                       constraint_x.at(constraint_x.size() - 1) -
                           constraint_x.at(constraint_x.size() - 2)));

  constraint->Add2dBoundary(t_coord, ref_theta, ref_point, longitudinal_bound,
                            lateral_bound);
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

  // 提取优化结果
  for (auto const t : t_coord) {
    auto xy = spline_solver.spline()(t);
    sloveX.emplace_back(xy.first);
    sloveY.emplace_back(xy.second);
  }
}
}  // namespace apollo::planning

namespace apollo::planning {
void spiral_smoother(std::vector<Eigen::Vector2d>& raw_points_,
                     std::vector<double>* theta, std::vector<double>* kappa,
                     std::vector<double>* dkappa, std::vector<double>* s,
                     std::vector<double>* spiral_x,
                     std::vector<double>* spiral_y) {
  SpiralReferenceLineSmoother spiral_smoother_;
  spiral_smoother_.SmoothStandAlone(raw_points_, theta, kappa, dkappa, s,
                                    spiral_x, spiral_y);
}
}  // namespace apollo::planning

namespace apollo::planning {
using namespace reference_line;
void hermit_spiral(std::vector<double>& raw_xs, std::vector<double>& raw_ys,
                   std::vector<double>& sloveX, std::vector<double>& sloveY) {
  CurveCreator creator(raw_xs, raw_ys);
  creator.solve();
  auto& curve = creator.curve();

  double length = curve->length();

  // constexpr int N = 44;
  int N = raw_xs.size() - 1;

  double stride = length / N;

  for (double i = 0; i < N + 1; i++) {
    sloveX.emplace_back(curve->x(i * stride));
    sloveY.emplace_back(curve->y(i * stride));
  }
}
}  // namespace apollo::planning

int main() {
  std::vector<Eigen::Vector2d> raw_points_;
  SetPoints(raw_points_);

  std::vector<double> theta;
  std::vector<double> kappa;
  std::vector<double> dkappa;
  std::vector<double> s;
  std::vector<double> spiral_x;
  std::vector<double> spiral_y;
  spiral_smoother(raw_points_, &theta, &kappa, &dkappa, &s, &spiral_x,
                  &spiral_y);

  /************************************************************************/

  std::vector<double> sloveX, sloveY;
  qp_spline_smoother(raw_points_, sloveX, sloveY);

  /************************************************************************/
  std::vector<double> raw_xs, raw_ys;
  div_circle(raw_xs, raw_ys, 5, 1);

  std::vector<double> hermit_sloveX, hermit_sloveY;
  hermit_spiral(raw_xs, raw_ys, hermit_sloveX, hermit_sloveY);

  /************************************************************************/
 
  std::vector<double> ref_x, ref_y;
  for (const auto& item : raw_points_) {
    ref_x.push_back(item.x());
    ref_y.push_back(item.y());
  }
  plt::named_plot("refrenceline_XY", ref_x, ref_y, "*");
  plt::named_plot("spiral_ipopt", spiral_x, spiral_y);
  plt::named_plot("qp_spline", sloveX, sloveY);
  plt::named_plot("hermit_spiral_ceres", hermit_sloveX, hermit_sloveY);
  plt::legend();
  // plt::figure();
  plt::axis("equal");
  plt::show();

  return 0;
}