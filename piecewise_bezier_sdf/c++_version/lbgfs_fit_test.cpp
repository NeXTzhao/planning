#include "matplotlibcpp.h"
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <lbfgs.h>
#include <utility>
#include <vector>

namespace plt = matplotlibcpp;
struct Point {
  double x;
  double y;
};
void generateData(std::vector<Point> &points) {

#if 1
  std::vector<double> x, y;

  int num_points = 30;
  for (int i = 0; i < num_points; ++i) {
    double t = i * M_PI / (num_points - 1);
    x.push_back(100 * std::cos(t));
    y.push_back(100 * std::sin(t));
  }

  // 翻转 x 和 y
  std::reverse(x.begin(), x.end());
  std::reverse(y.begin(), y.end());

  // 将数据保存到 Point 结构体
  for (size_t i = 0; i < x.size(); ++i) {
    Point point = {x[i], y[i]};
    points.push_back(point);
  }
#endif
#if 0
  std::vector<double> x, y;
  std::vector<double> tdata1 = {-60, -50, -40, -30, -20, -10};
  std::vector<double> tdata2 = {-10, -20, -30, -40, -50, -60};

  for (double t1 : tdata1) {
    x.push_back(100);
    y.push_back(t1);
  }

  int num_points = 30;
  for (int i = 0; i < num_points; ++i) {
    double t = i * M_PI / (num_points - 1);
    x.push_back(100 * std::cos(t));
    y.push_back(100 * std::sin(t));
  }

  for (double t2 : tdata2) {
    x.push_back(-100);
    y.push_back(t2);
  }

  // 翻转 x 和 y
  std::reverse(x.begin(), x.end());
  std::reverse(y.begin(), y.end());

  // 将数据保存到 Point 结构体
  for (size_t i = 0; i < x.size(); ++i) {
    Point point = {x[i], y[i]};
    points.push_back(point);
  }
//  for (const auto& point : points) {
//    std::cout << "x: " << point.x << ", y: " << point.y << std::endl;
//  }
#endif
#if 0
    for (int i = 0; i <= 50; i++) {
      points.push_back({static_cast<double>(i), 0});
    }
    double h = 0;
    for (int i = 0; i < 5; i++) {
      h++;
      points.push_back({50, h});
    }
    points.pop_back();
    points.push_back({50, 5});
    for (int i = 51; i < 61; i++) {
      points.push_back({static_cast<double>(i), 5});
    }
    for (int i = 0; i < 5; i++) {
      points.push_back({61, h});
      h--;
    }

    for (int i = 61; i <= 81; i++) {
      points.push_back({static_cast<double>(i), 0});
    }

    //    for (const auto &point : points) {
    //      std::cout << "x = " << point.x << " , "
    //                << " y = " << point.y << '\n';
    //    }
#endif
}

// Define the quadratic Bezier curve parameterized by P0, P1, P2
class BezierCurve {
 public:
  Eigen::Vector2d operator()(double t) const {
    double t1 = 1.0 - t;
    return t1 * t1 * P0 + 2.0 * t1 * t * P1 + t * t * P2;
  }

  Eigen::Vector2d gradient(double t) const {
    double t1 = 1.0 - t;
    Eigen::Vector2d grad;
    grad = -2.0 * t1 * P0 + 2.0 * (t1 - t) * P1 + 2.0 * t * P2;
    return grad;
  }

  void setControlPoints(const Eigen::Vector2d &p0, const Eigen::Vector2d &p1,
                        const Eigen::Vector2d &p2) {
    P0 = p0;
    P1 = p1;
    P2 = p2;
  }

  // Generate a set of points along the Bezier curve by sampling 'num_points' times
  std::vector<Eigen::Vector2d> generateCurvePoints(int num_points) const {
    std::vector<Eigen::Vector2d> curve_points;
    for (int i = 0; i <= num_points; ++i) {
      double t = static_cast<double>(i) / static_cast<double>(num_points);
      curve_points.push_back((*this)(t));
    }
    return curve_points;
  }

 private:
  Eigen::Vector2d P0, P1, P2;
};

// Define the objective function to minimize the fitting error
class FittingObjective {
 public:
  FittingObjective(const std::vector<Eigen::Vector2d> &data,
                   BezierCurve &bezier)
      : data(data), bezier(bezier) {}
  // Calculate cumulative arc length for each point and normalize to 0 to 1
  //    double total_arc_length = calculateTotalArcLength();
  //    cumulativeArcLength.push_back(0.0);
  //    for (size_t i = 1; i < data.size(); ++i) {
  //      double distance = (data[i] - data[i - 1]).norm();
  //      cumulativeArcLength.push_back(cumulativeArcLength[i - 1]
  //                                    + distance / total_arc_length);
  //    }
  //  }

  double operator()(const Eigen::VectorXd &P1) const {
    bezier.setControlPoints(data.front(), P1, data.back());
    double total_error = 0.0;

    for (const auto &point : data) {
      double t = point[0];
      Eigen::Vector2d curve_point = bezier(t);
      total_error += (curve_point - point).squaredNorm();
    }

    return total_error;
  }

  Eigen::VectorXd gradient(const Eigen::VectorXd &P1) const {
    Eigen::VectorXd grad(2);

    // Compute the gradient analytically
    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_term = 0.0;
    for (const auto &point : data) {
      double t = point[0];
      Eigen::Vector2d curve_point = bezier(t);

      double diff_x = curve_point[0] - point[1];
      double diff_y = curve_point[1] - point[1];
      double basis = 2.0 * (1.0 - t) * t;

      sum_x += basis * diff_x;
      sum_y += basis * diff_y;
      sum_term += basis * basis;
    }

    grad[0] = sum_x / sum_term;
    grad[1] = sum_y / sum_term;

    return grad;
  }

  // Static method for L-BFGS to evaluate the objective function
  // Static method for L-BFGS to evaluate the objective function
  static lbfgsfloatval_t evaluate(void *instance, const lbfgsfloatval_t *x,
                                  lbfgsfloatval_t *g, const int n,
                                  const lbfgsfloatval_t step) {
    FittingObjective *obj = reinterpret_cast<FittingObjective *>(instance);

    // Map the input array to Eigen::VectorXd
    Eigen::Map<const Eigen::VectorXd> x_vec(x, n);

    // Compute the gradient and copy it to the output array g
    Eigen::VectorXd grad = obj->gradient(x_vec);
    std::memcpy(g, grad.data(), n * sizeof(lbfgsfloatval_t));

    // Evaluate the objective function
    return obj->operator()(x_vec);
  }

  // Static method for L-BFGS to report progress
  static int progress(void *instance, const lbfgsfloatval_t *x,
                      const lbfgsfloatval_t *g, const lbfgsfloatval_t fx,
                      const lbfgsfloatval_t xnorm, const lbfgsfloatval_t gnorm,
                      const lbfgsfloatval_t step, int n, int k, int ls) {
    std::cout << "Iteration " << k << ": f(x) = " << fx << ", xnorm = " << xnorm
              << ", gnorm = " << gnorm << ", step = " << step << std::endl;
    return 0;
  }

 private:
  const std::vector<Eigen::Vector2d> &data;
  BezierCurve &bezier;

  std::vector<double> cumulativeArcLength;

  //  double calculateTotalArcLength() const {
  //    double total_arc_length = 0.0;
  //    for (size_t i = 1; i < data.size(); ++i) {
  //      double distance = (data[i] - data[i - 1]).norm();
  //      total_arc_length += distance;
  //    }
  //    return total_arc_length;
  //  }
};

int main() {
  std::vector<Eigen::Vector2d> data;
  for (double t = 1.0; t >= 0.0; t -= 0.1) {
    //    double noise = 0.1 * ((double)std::rand() / RAND_MAX - 0.5);
    auto x = 100 * std::cos(t);
    auto y = 100 * std::sin(t);
    printf("x = %f, y = %f\n", x, y);
    Eigen::Vector2d point(x, y);
    data.push_back(point);
  }

  //  std::vector<Eigen::Vector2d> data;
  //  for (double t = 0.0; t <= 1.0; t += 0.1) {
  //    double noise = 0.1 * ((double)std::rand() / RAND_MAX - 0.5);
  //    Eigen::Vector2d point(t, t * t + noise);
  //    data.push_back(point);
  //  }

  // Set up L-BFGS parameters
  lbfgs_parameter_t lbfgs_params;
  lbfgs_parameter_init(&lbfgs_params);
  lbfgs_params.max_iterations = 100;
  lbfgs_params.linesearch = LBFGS_LINESEARCH_BACKTRACKING;

  // Initialize Bezier control points
  BezierCurve bezier;
  Eigen::Vector2d p1(0.0, 0.0);

  // Optimize using L-BFGS
  FittingObjective objective(data, bezier);
  lbfgsfloatval_t fx;
  lbfgsfloatval_t *x = p1.data();
  int ret = lbfgs(2, x, &fx, FittingObjective::evaluate,
                  FittingObjective::progress, &objective, &lbfgs_params);

  // Set optimized control points
  Eigen::Vector2d p0 = data.front();
  Eigen::Vector2d p2 = data.back();
  bezier.setControlPoints(p0, p1, p2);
  auto fit_point = bezier.generateCurvePoints(100);

  // Print the optimization result
  std::cout << "L-BFGS optimization terminated with status code " << ret
            << std::endl;
  std::cout << "Optimal control points: P0(" << p0[0] << ", " << p0[1]
            << "), P1(" << p1[0] << ", " << p1[1] << "), P2(" << p2[0] << ", "
            << p2[1] << ")" << std::endl;
  std::cout << "Optimal f(x): " << fx << std::endl;
  ////////////////////////////////////////////////
  std::vector<double> fit_x, fit_y, row_x, row_y, con_x, con_y;

  for (const auto &point : fit_point) {
    fit_x.emplace_back(point.x());
    fit_y.emplace_back(point.y());
  }

  con_x = {p0.x(), p1.x(), p2.x()};
  con_y = {p0.y(), p1.y(), p2.y()};

  for (const auto &point : data) {
    row_x.emplace_back(point.x());
    row_y.emplace_back(point.y());
  }
  plt::named_plot("fit_curve", fit_x, fit_y, "-");
  plt::named_plot("con_pt", con_x, con_y, "o");
  plt::named_plot("row_data", row_x, row_y, ".");
  plt::legend();
  plt::show();

  return 0;
}
