#include "matplotlibcpp.h"
#include <Eigen/Dense>
#include <vector>

namespace plt = matplotlibcpp;
struct Point {
  double x;
  double y;
};
// 二阶贝塞尔曲线参数方程
Eigen::Vector2d cal_bezier_value(double t, const Eigen::Vector2d &p0,
                                 const Eigen::Vector2d &p1,
                                 const Eigen::Vector2d &p2) {
  return (1 - t) * (1 - t) * p0 + 2 * (1 - t) * t * p1 + t * t * p2;
}

// 目标函数：最小化曲线与样本点之间的距离误差
double objective_function(const Eigen::Vector2d &p1,
                          const Eigen::VectorXd &t_values,
                          const std::vector<Eigen::Vector2d> &points,
                          const std::vector<Eigen::Vector2d> &samples) {
  double error = 0;
  for (int i = 0; i < samples.size(); ++i) {
    double t = t_values(i);
    const Eigen::Vector2d &sample = samples[i];

    Eigen::Vector2d curve = cal_bezier_value(t, points[0], p1, points[1]);
    error += (sample - curve).squaredNorm();
  }
  return error;
}

// 计算目标函数关于 p1 的梯度
Eigen::Vector2d compute_gradient(const Eigen::Vector2d &p1,
                                 const Eigen::VectorXd &t_values,
                                 const std::vector<Eigen::Vector2d> &points,
                                 const std::vector<Eigen::Vector2d> &samples,
                                 double epsilon = 1e-6) {
  Eigen::Vector2d gradient;
  for (int i = 0; i < 2; ++i) {
    Eigen::Vector2d p1_plus_epsilon = p1;
    p1_plus_epsilon(i) += epsilon;

    double f_plus_epsilon =
        objective_function(p1_plus_epsilon, t_values, points, samples);
    double f = objective_function(p1, t_values, points, samples);

    gradient(i) = (f_plus_epsilon - f) / epsilon;
  }
  return gradient;
}

// 通过梯度下降法找到最佳的中间控制点 P1
std::array<Point, 3>
calBezierControlPoint(const std::vector<Eigen::Vector2d> &points,
                      const Eigen::VectorXd &t_values,
                      const std::vector<Eigen::Vector2d> &samples,
                      double learning_rate = 0.01, int max_iterations = 1000) {
  Eigen::Vector2d initial_p1 =
      (points[0] + points[1]) / 2.0;// 初始值可以取为贝塞尔曲线两端点的平均值

  Eigen::Vector2d p1 = initial_p1;

  for (int iteration = 0; iteration < max_iterations; ++iteration) {
    Eigen::Vector2d gradient = compute_gradient(p1, t_values, points, samples);
    p1 -= learning_rate * gradient;
  }
  std::array<Point, 3> control_points = {{{points[0].x(), points[0].y()},
                                          {p1.x(), p1.y()},
                                          {points[1].x(), points[1].y()}}};

  return control_points;
}

// 示例使用：
int main() {
  // 给定的贝塞尔曲线两端点
  Eigen::Vector2d p0(100, 315);
  Eigen::Vector2d p2(300, 495);

  // 样本点，这里假设已知曲线上的几个点的函数值
  std::vector<Eigen::Vector2d> samples = {
      Eigen::Vector2d(100, 315), Eigen::Vector2d(150, 400),
      Eigen::Vector2d(200, 430), Eigen::Vector2d(250, 460),
      Eigen::Vector2d(300, 495)};

  // 生成对应的 t 值
  Eigen::VectorXd t_values(samples.size());
  for (int i = 0; i < t_values.size(); ++i) {
    t_values(i) = static_cast<double>(i) / (t_values.size() - 1);
  }

  // 拟合二阶贝塞尔曲线的中间控制点 P1
  std::vector<Eigen::Vector2d> points = {p0, p2};
  auto control_point = calBezierControlPoint(points, t_values, samples);

  /**************************************************/
  std::vector<double> points_x, points_y;
  for (const auto &point : samples) {
    points_x.push_back(point.x());
    points_y.push_back(point.y());
  }
  std::vector<double> con_x, con_y;
  for (const auto &point : control_point) {
    con_x.push_back(point.x);
    con_y.push_back(point.y);
  }

  // 输出拟合得到的中间控制点 P1
  plt::named_plot("con_point", con_x, con_y, "*");
  plt::named_plot("row_point", points_x, points_y, ".");

  plt::grid(true);
  plt::axis("equal");
  plt::grid(true);
  plt::legend();
  plt::show();
  return 0;
}
