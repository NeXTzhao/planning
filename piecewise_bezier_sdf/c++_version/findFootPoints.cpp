#include "matplotlibcpp.h"
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <lbfgs.h>
#include <limits>
#include <vector>
namespace plt = matplotlibcpp;

struct Point {
  double x, y;
  double squaredNorm() const { return x * x + y * y; }
  double dot(const Point &other) const { return x * other.x + y * other.y; }
};

Point operator+(const Point &p1, const Point &p2) {
  return {p1.x + p2.x, p1.y + p2.y};
}

Point operator-(const Point &p1, const Point &p2) {
  return {p1.x - p2.x, p1.y - p2.y};
}

Point operator*(double scalar, const Point &p) {
  return {scalar * p.x, scalar * p.y};
}

Point operator*(const Point &p, double scalar) {
  return {scalar * p.x, scalar * p.y};
}

Point operator*(const Point &p1, const Point &p2) {
  return {p1.x * p2.x, p1.y * p2.y};
}

//double dot(const Point &p1, const Point &p2) {
//  return p1.x * p2.x + p1.y * p2.y;
//}

Point q(const std::vector<Point> &ctrlPoly, double t) {
  double t_1 = 1.0 - t;
  double t2 = t * t;
  double t_12 = t_1 * t_1;

  Point p{};
  p.x = t_12 * ctrlPoly[0].x + 2 * t_1 * t * ctrlPoly[1].x + t2 * ctrlPoly[2].x;
  p.y = t_12 * ctrlPoly[0].y + 2 * t_1 * t * ctrlPoly[1].y + t2 * ctrlPoly[2].y;
  return p;
}

// 计算二阶贝塞尔曲线的一阶导数
Point qprime(const std::vector<Point> &ctrlPoly, double t) {
  double t_1 = 1.0 - t;

  Point p{};
  p.x = 2 * t_1 * (ctrlPoly[1].x - ctrlPoly[0].x)
      + 2 * t * (ctrlPoly[2].x - ctrlPoly[1].x);
  p.y = 2 * t_1 * (ctrlPoly[1].y - ctrlPoly[0].y)
      + 2 * t * (ctrlPoly[2].y - ctrlPoly[1].y);
  return p;
}

// 计算二阶贝塞尔曲线的二阶导数
Point qprimeprime(const std::vector<Point> &ctrlPoly, double t) {
  Point p{};
  p.x = 2 * (ctrlPoly[2].x - 2 * ctrlPoly[1].x + ctrlPoly[0].x);
  p.y = 2 * (ctrlPoly[2].y - 2 * ctrlPoly[1].y + ctrlPoly[0].y);
  return p;
}
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
// Define the Bezier curve functions
Point bezier(const Point &p0, const Point &p1, const Point &p2, double t) {
  return {(1 - t) * (1 - t) * p0.x + 2 * (1 - t) * t * p1.x + t * t * p2.x,
          (1 - t) * (1 - t) * p0.y + 2 * (1 - t) * t * p1.y + t * t * p2.y};
}

Point bezier_prime(const Point &p0, const Point &p1, const Point &p2,
                   double t) {
  return {2 * (1 - t) * (p1.x - p0.x) + 2 * t * (p2.x - p1.x),
          2 * (1 - t) * (p1.y - p0.y) + 2 * t * (p2.y - p1.y)};
}

// Function to find the best t value for a data point
double findBestT(const Point &Xk, const Point &p0, const Point &p1,
                 const Point &p2, double initialT, int maxIter, double tol) {
  double tk = initialT;

  for (int i = 0; i < maxIter; ++i) {
    // Calculate foot point and tangent vector
    Point Pk = bezier(p0, p1, p2, tk);
    Point Pk_prime = bezier_prime(p0, p1, p2, tk);

    // Calculate descent direction and step size
    double delta_t = (Xk.x - Pk.x) * Pk_prime.x + (Xk.y - Pk.y) * Pk_prime.y;
    delta_t /= Pk_prime.x * Pk_prime.x + Pk_prime.y * Pk_prime.y;

    double a = 0.1;

    // Update parameter t
    tk = tk + a * delta_t;
    //    std::cout << "t" << i << " = " << tk << " , delta_t = " << delta_t
    //              << std::endl;
    // Check convergence condition
    if (std::abs((Xk.x - Pk.x) * Pk_prime.x + (Xk.y - Pk.y) * Pk_prime.y)
        < tol) {
      std::cout << "The optimization process stops" << std::endl;
      break;
    }
  }

  return tk;
}

// Function to optimize p1 using gradient descent
Point optimizeP1(const Point &p0, const Point &p2) {
  //  Point p1 = {(p0.x + p2.x) / 2, (p0.y + p2.y) / 2};// Initial guess for p1
  Point p1{};
  double segLength = std::hypot(p0.x - p2.x, p0.y - p2.y);
  double length = std::hypot(p0.x - p2.x, p0.y - p2.y);
  double leftTangent_x = (p2.x - p0.x) / length;
  double leftTangent_y = (p2.y - p0.y) / length;

  p1.x = p0.x + leftTangent_x * (segLength / 3.0);
  p1.y = p0.y + leftTangent_y * (segLength / 3.0);

  std::cout << "Optimal control points: P0(" << p0.x << ", " << p0.y << "), P1("
            << p1.x << ", " << p1.y << "), P2(" << p2.x << ", " << p2.y << ")"
            << std::endl;
  return p1;
}

// 迭代函数的实现
template<typename Func>
double newton_raphson_iterate(Func func, double initial_guess,
                              double lower_bound, double upper_bound,
                              int digits, int &it) {
  double x = initial_guess;
  double x_prev = x;
  double f_val = func(x).first;

  while (it > 0) {
    it--;

    // 计算导数
    auto [f_prime, f_double_prime] = func(x);

    // 更新 x
    x_prev = x;
    x = x - f_prime / f_double_prime;

    // 确保 x 在指定范围内
    if (x < lower_bound) x = lower_bound;
    else if (x > upper_bound)
      x = upper_bound;

    // 检查是否收敛
    if (std::abs(x - x_prev) <= std::pow(6, -digits)) break;
  }

  return x;
}

void GetFootPoint(const std::vector<Point> &bez, const Point &point,
                  const double t0, double &t) {
  int digits = std::numeric_limits<double>::digits;
  int get_digits = static_cast<int>(digits * 0.6);
  const int maxit = 20;
  int it = maxit;

  t = newton_raphson_iterate(
      [&point, &bez](const double &tau) {
        auto P_0 = q(bez, tau);
        auto P_1 = qprime(bez, tau);
        auto P_2 = qprimeprime(bez, tau);
        auto d_P = P_0 - point;
        return std::make_pair(d_P.dot(P_1), P_1.dot(P_1) + d_P.dot(P_2));
      },
      t0, fmax(t0 - 0.1, 0.0), fmin(t0 + 0.1, 1.0), get_digits, it);
}

std::vector<double> GenerateKnotsFitting(const std::vector<Point> &bez,
                                         const std::vector<Point> &data_points,
                                         const size_t match_point_size = 100) {
  size_t data_points_size = data_points.size();

  std::vector<double> knots_fitting(data_points_size);

  Eigen::VectorXd ts = Eigen::VectorXd::LinSpaced(
      static_cast<Eigen::Index>(match_point_size), 0, 1);
  std::vector<Point> point_to_match(match_point_size);

  for (auto i = 0; i < ts.size(); i++) { point_to_match[i] = q(bez, ts[i]); }

  auto lb = point_to_match.begin();
  for (size_t i = 0; i < data_points_size; i++) {
    const auto &point = data_points[i];
    auto min_iter = std::min_element(
        lb, point_to_match.end(), [&point](const Point &a, const Point &b) {
          return (a - point).squaredNorm() < (b - point).squaredNorm();
        });
    double distance = (*min_iter - point).squaredNorm();
    auto min_index = std::distance(point_to_match.begin(), min_iter);
    knots_fitting[i] = ts[min_index];
    lb = min_iter;
  }

  return knots_fitting;
}

std::vector<double> GetTValuesForPoints(const std::vector<Point> &bez,
                                        const std::vector<Point> &row_points) {
  auto initial_t_guess = GenerateKnotsFitting(bez, row_points);

  std::vector<double> tValues;

  for (int i = 0; i < row_points.size(); ++i) {
    double t;
    GetFootPoint(bez, row_points[i], initial_t_guess[i], t);
    tValues.push_back(t);
  }

  return tValues;
}

//double BezierBasis(int i, int n, double t) {
//  double coeff = 1.0;
//  for (int j = 0; j < n; ++j) {
//    if (j == i) continue;
//    coeff *= (t - j) / (i - j);
//  }
//  return coeff;
//}
//
//Point EvaluateBezier(const std::vector<Point> &control_points, double t) {
//  int n = control_points.size() - 1;
//  Point result = {0.0, 0.0};
//  for (int i = 0; i <= n; ++i) {
//    double basis = BezierBasis(i, n, t);
//    result.x += basis * control_points[i].x;
//    result.y += basis * control_points[i].y;
//  }
//  return result;
//}
//
//double FittingCost(const std::vector<Point> &data_points,
//                   const std::vector<Point> &curve_points) {
//  double cost = 0.0;
//  for (size_t i = 0; i < data_points.size(); ++i) {
//    double distance = (data_points[i], curve_points[i]).squaredNorm();
//    cost += distance;
//  }
//  return cost;
//}

int main() {
  std::vector<Point> dataPoints;
  generateData(dataPoints);
  Point p0 = {dataPoints.front().x, dataPoints.front().y};
  Point p2 = {dataPoints.back().x, dataPoints.back().y};

  // Optimize p1 for each data point and find t values
  Point P1 = optimizeP1(p0, p2);
  std::vector<Point> bez{p0, P1, p2};

  //  double initial_guess = 0.0;// 替换为合适的初始猜测值

  auto initial_guess = GenerateKnotsFitting(bez, dataPoints);

  std::vector<double> bestTValues = GetTValuesForPoints(bez, dataPoints);
  // Output t values
  for (int i = 0; i < dataPoints.size(); ++i) {
    std::cout << "Data point (" << dataPoints[i].x << ", " << dataPoints[i].y
              << ") init_guss t = " << initial_guess[i]
              << " , find best t = " << bestTValues[i] << std::endl;
  }
  ////////////////////////////////////////////////
  std::vector<double> fit_x, fit_y, row_x, row_y, con_x, con_y;

  //  for (const auto &point : fit_point) {
  //    fit_x.emplace_back(point.x());
  //    fit_y.emplace_back(point.y());
  //  }
  //
  //  con_x = {p0.x(), p1.x(), p2.x()};
  //  con_y = {p0.y(), p1.y(), p2.y()};
  //  con_x.push_back(p0.x());
  //  con_y.push_back(p0.y());

  for (const auto &point : dataPoints) {
    row_x.emplace_back(point.x);
    row_y.emplace_back(point.y);
  }
  //  plt::named_plot("fit_curve", fit_x, fit_y, "r-");
  //  plt::named_plot("con_pt", con_x, con_y, "o--");
  //  plt::named_plot("row_data", row_x, row_y, ".");
  //  plt::legend();
  //  plt::show();
  return 0;
}
