#include "piecewise_bezier_curve2.h"
#include <chrono>
#include <cmath>

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

PiecewiseBezierFit2::PiecewiseBezierFit2(const std::vector<Point> &points, int degree, double maxError) {
  degree_ = degree;
  fitCurve(points, maxError);
}

void PiecewiseBezierFit2::fitCurve(const std::vector<Point> &points, double maxError) {
  Point leftTangent = normalize(points[1] - points[0]);
  Point rightTangent = normalize(points[points.size() - 2] - points[points.size() - 1]);
  control_points_ = fitCubicBezier(points, leftTangent, rightTangent, maxError);
  composeTrimmingSdf();
  generate_bezier_curves(control_points_, 100);
}

std::vector<std::vector<Point>> PiecewiseBezierFit2::getControlPoints() const {
  return control_points_;
}

std::vector<std::vector<Point>> PiecewiseBezierFit2::getPiecewiseBezierCurvesPoints() const {
  return piecewise_bezier_curve_;
}

//Point PiecewiseBezierFit2::q(const std::array<Point, 4> &ctrlPoly, double t) {
//  double t_1 = 1.0 - t;
//  return pow(t_1, 3) * ctrlPoly[0] + 3 * pow(t_1, 2) * t * ctrlPoly[1] + 3 * t_1 * pow(t, 2) * ctrlPoly[2] + pow(t, 3) * ctrlPoly[3];
//}
//
//Point PiecewiseBezierFit2::qprime(const std::array<Point, 4> &ctrlPoly, double t) {
//  double t_1 = 1.0 - t;
//  return 3 * pow(t_1, 2) * (ctrlPoly[1] - ctrlPoly[0]) + 6 * t_1 * t * (ctrlPoly[2] - ctrlPoly[1]) + 3 * pow(t, 2) * (ctrlPoly[3] - ctrlPoly[2]);
//}
//
//Point PiecewiseBezierFit2::qprimeprime(const std::array<Point, 4> &ctrlPoly, double t) {
//  return 6 * (1.0 - t) * (ctrlPoly[2] - 2 * ctrlPoly[1] + ctrlPoly[0]) + 6 * (t) * (ctrlPoly[3] - 2 * ctrlPoly[2] + ctrlPoly[1]);
//}

Point PiecewiseBezierFit2::q(const std::vector<Point> &ctrlPoly, double t) {
  double t_1 = 1.0 - t;
  double t2 = t * t;
  double t_12 = t_1 * t_1;

  Point p{};
  p.x = t_12 * ctrlPoly[0].x + 2 * t_1 * t * ctrlPoly[1].x + t2 * ctrlPoly[2].x;
  p.y = t_12 * ctrlPoly[0].y + 2 * t_1 * t * ctrlPoly[1].y + t2 * ctrlPoly[2].y;
  return p;
}

// 计算二阶贝塞尔曲线的一阶导数
Point PiecewiseBezierFit2::qprime(const std::vector<Point> &ctrlPoly, double t) {
  double t_1 = 1.0 - t;

  Point p{};
  p.x = 2 * t_1 * (ctrlPoly[1].x - ctrlPoly[0].x) + 2 * t * (ctrlPoly[2].x - ctrlPoly[1].x);
  p.y = 2 * t_1 * (ctrlPoly[1].y - ctrlPoly[0].y) + 2 * t * (ctrlPoly[2].y - ctrlPoly[1].y);
  return p;
}

// 计算二阶贝塞尔曲线的二阶导数
Point PiecewiseBezierFit2::qprimeprime(const std::vector<Point> &ctrlPoly, double t) {
  Point p{};
  p.x = 2 * (ctrlPoly[2].x - 2 * ctrlPoly[1].x + ctrlPoly[0].x);
  p.y = 2 * (ctrlPoly[2].y - 2 * ctrlPoly[1].y + ctrlPoly[0].y);
  return p;
}

/*
 *  Bezier :
 *  	Evaluate a Bezier curve at a particular parameter value
 */
Point PiecewiseBezierFit2::BezierII(int degree, const std::vector<Point> &V, double t) {
  Point Q{};                                   // Point on curve at parameter t
  std::vector<Point> Vtemp(V.begin(), V.end());// Local copy of control points

  // Triangle computation
  for (int i = 1; i <= degree; i++) {
    for (int j = 0; j <= degree - i; j++) {
      Vtemp[j].x = (1.0 - t) * Vtemp[j].x + t * Vtemp[j + 1].x;
      Vtemp[j].y = (1.0 - t) * Vtemp[j].y + t * Vtemp[j + 1].y;
    }
  }

  Q = Vtemp[0];
  return Q;
}

// 二阶贝塞尔曲线参数方程
Eigen::Vector2d PiecewiseBezierFit2::cal_bezier_value(double t, const Eigen::Vector2d &p0, const Eigen::Vector2d &p1, const Eigen::Vector2d &p2) {
  return (1 - t) * (1 - t) * p0 + 2 * (1 - t) * t * p1 + t * t * p2;
}

// 目标函数：最小化曲线与样本点之间的距离误差
double PiecewiseBezierFit2::objective_function(const Eigen::Vector2d &p1, const Eigen::VectorXd &t_values, const std::vector<Eigen::Vector2d> &points, const std::vector<Eigen::Vector2d> &samples) {
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
Eigen::Vector2d PiecewiseBezierFit2::compute_gradient(const Eigen::Vector2d &p1, const Eigen::VectorXd &t_values, const std::vector<Eigen::Vector2d> &points, const std::vector<Eigen::Vector2d> &samples, double epsilon) {
  Eigen::Vector2d gradient;
  for (int i = 0; i < 2; ++i) {
    Eigen::Vector2d p1_plus_epsilon = p1;
    p1_plus_epsilon(i) += epsilon;

    double f_plus_epsilon = objective_function(p1_plus_epsilon, t_values, points, samples);
    double f = objective_function(p1, t_values, points, samples);

    gradient(i) = (f_plus_epsilon - f) / epsilon;
  }
  return gradient;
}

// 通过梯度下降法找到最佳的中间控制点 P1
std::vector<Point> PiecewiseBezierFit2::generateBezierControlPoint(const std::vector<Point> &control_point, const std::vector<Point> &row_points, const std::vector<double> &parameters, const Point &leftTangent, const Point &rightTangent, double learning_rate, int max_iterations) {
  std::vector<Eigen::Vector2d> points;
  for (const auto &point : control_point) {
    points.emplace_back(point.x, point.y);
  }
  std::vector<Eigen::Vector2d> row_point;
  for (const auto &point : row_points) {
    row_point.emplace_back(point.x, point.y);
  }
  Eigen::VectorXd t_value = Eigen::Map<Eigen::VectorXd>(parameters.data(), parameters.size());

  Eigen::Vector2d initial_p1 = (points[0] + points[1]) / 2.0;// 初始值可以取为贝塞尔曲线两端点的平均值

  Eigen::Vector2d p1 = initial_p1;

  for (int iteration = 0; iteration < max_iterations; ++iteration) {
    Eigen::Vector2d gradient = compute_gradient(p1, t_value, points, row_point);
    p1 -= learning_rate * gradient;
  }
  std::vector<Point> control_points = {{{points[0].x(), points[0].y()}, {p1.x(), p1.y()}, {points[1].x(), points[1].y()}}};

  return control_points;
}

Point PiecewiseBezierFit2::normalize(const Point &v) {
  double length = sqrt(v.x * v.x + v.y * v.y);
  return {v.x / length, v.y / length};
}

std::vector<double> PiecewiseBezierFit2::chordLengthParameterize(const std::vector<Point> &points) {
  std::vector<double> u;
  u.push_back(0.0);
  for (size_t i = 1; i < points.size(); ++i) {
    double segmentLength = sqrt(pow(points[i].x - points[i - 1].x, 2) + pow(points[i].y - points[i - 1].y, 2));
    u.push_back(u[i - 1] + segmentLength);
  }

  for (size_t i = 0; i < u.size(); ++i) {
    u[i] /= u.back();
  }

  return u;
}

double PiecewiseBezierFit2::dot(const Point &vec1, const Point &vec2) {
  return vec1.x * vec2.x + vec1.y * vec2.y;
}

std::vector<std::vector<Point>> PiecewiseBezierFit2::fitCubicBezier(const std::vector<Point> &points, const Point &leftTangent, const Point &rightTangent, double error) {
  std::vector<std::vector<Point>> bezCtlPts;
  if (points.size() == 2) {
    double dist = std::sqrt(std::pow(points[0].x - points[1].x, 2) + std::pow(points[0].y - points[1].y, 2)) / 3.0;
    std::vector<Point> control_point = {points[0], points[0] + dist * leftTangent, points[1] + dist * rightTangent, points[1]};
    bezCtlPts.emplace_back(control_point);
    return bezCtlPts;
  }

  std::vector<double> u = chordLengthParameterize(points);
  auto control_point = generateBezierControlPoint(points, u, leftTangent, rightTangent);
  auto error_split = computeMaxError(points, control_point, u);
  double point2CurveMaxError = error_split.first;
  int splitPoint = error_split.second;

  if (point2CurveMaxError < error) {
    bezCtlPts.emplace_back(control_point);
    return bezCtlPts;
  }

  if (point2CurveMaxError < error * error) {
    for (int i = 0; i < 2; i++) {
      auto uPrime = reparameterize(control_point, points, u);
      control_point = generateBezierControlPoint(points, uPrime, leftTangent, rightTangent);
      error_split = computeMaxError(points, control_point, uPrime);
      point2CurveMaxError = error_split.first;
      splitPoint = error_split.second;
      if (point2CurveMaxError < error) {
        bezCtlPts.emplace_back(control_point);
        return bezCtlPts;
      }
      u = uPrime;
    }
  }

  // 把切分出切线方向
  Point centerTangent = normalize(points[splitPoint - 1] - points[splitPoint + 1]);
  auto negativeCenterTangent = Point{-centerTangent.x, -centerTangent.y};

  // 切分point
  std::vector<Point> leftPoints(points.begin(), points.begin() + splitPoint + 1);
  std::vector<Point> rightPoints(points.begin() + splitPoint, points.end());

  // 左右两边递归
  auto leftBeziers = fitCubicBezier(leftPoints, leftTangent, centerTangent, error);
  bezCtlPts.insert(bezCtlPts.end(), leftBeziers.begin(), leftBeziers.end());

  auto rightBeziers = fitCubicBezier(rightPoints, negativeCenterTangent, rightTangent, error);
  bezCtlPts.insert(bezCtlPts.end(), rightBeziers.begin(), rightBeziers.end());

  return bezCtlPts;
}

std::vector<double> PiecewiseBezierFit2::reparameterize(const std::vector<Point> &bezier, const std::vector<Point> &points, const std::vector<double> &parameters) {
  std::vector<double> newParameters;
  for (size_t i = 0; i < points.size(); ++i) {
    double u = parameters[i];
    Point point = points[i];
    double newU = newtonRaphsonRootFind(bezier, point, u);
    newParameters.push_back(newU);
  }
  return newParameters;
}

double PiecewiseBezierFit2::newtonRaphsonRootFind(const std::vector<Point> &bez, const Point &point, double u) {
  Point err = q(bez, u) - point;
  Point d_curve = qprime(bez, u);
  Point dd_curve = qprimeprime(bez, u);
  /*
  牛顿迭代法（又名：牛顿-拉弗森法）
  不断地计算 u = u - f(u)/f'(u)
    1. f(u) = (x-x_p)* x' + (y - y_p)* y'
    2. f'(u) = (x')^2 + (y')^2 + (x-x_p)* x'' + (y - y_p)* y''
    3. if abs(f'(u)) < eplison; return
    4. else u - f(u)/f'(u)
    最终得到一个u，使得目标带点距离曲线最近
*/
  //    double numerator = err.x * d_curve.x + err.y * d_curve.y;
  //    double denominator = d_curve.x * d_curve.x + d_curve.y * d_curve.y + err.x * dd_curve.x + err.y * dd_curve.y;

  auto n = err * d_curve;
  auto den = d_curve * d_curve + err * dd_curve;

  double numerator = sqrt(n.x * n.x + n.y * n.y);
  double denominator = sqrt(den.x * den.x + den.y * den.y);

  if (std::abs(denominator) < 1e-3) {
    return u;
  } else {
    return u - numerator / denominator;
  }
}

std::pair<double, int> PiecewiseBezierFit2::computeMaxError(const std::vector<Point> &points, const std::vector<Point> &bez,
                                                            const std::vector<double> &parameters) {
  double maxDist = 0.0;
  int splitPoint = static_cast<int>(points.size() / 2);

  for (int i = 0; i < points.size(); ++i) {
    double dx = q(bez, parameters[i]).x - points[i].x;
    double dy = q(bez, parameters[i]).y - points[i].y;
    double dist = dx * dx + dy * dy;// Square the distance
    if (dist > maxDist) {
      maxDist = dist;
      splitPoint = i;
    }
  }

  return std::make_pair(maxDist, splitPoint);
}

// 计算二项式系数
double PiecewiseBezierFit2::binomial_coefficient(int n, int k) {
  double result = 1.0;
  for (int i = 1; i <= k; i++) {
    result *= static_cast<double>(n - i + 1) / i;
  }
  return result;
}

// 贝塞尔曲线生成函数
std::vector<Point> PiecewiseBezierFit2::bezier_curve(const std::vector<Point> &control_points, int num_points) {
  int n = control_points.size() - 1;
  std::vector<Point> curve_points(num_points);

  for (int i = 0; i < num_points; i++) {
    double t = static_cast<double>(i) / (num_points - 1);
    double one_minus_t = 1.0 - t;

    for (int j = 0; j <= n; j++) {
      double coeff = binomial_coefficient(n, j) * pow(one_minus_t, n - j) * pow(t, j);
      curve_points[i].x += coeff * control_points[j].x;
      curve_points[i].y += coeff * control_points[j].y;
    }
  }

  return curve_points;
}

// 生成多组贝塞尔曲线
void PiecewiseBezierFit2::generate_bezier_curves(const std::vector<std::vector<Point>> &control_points_list, int num_points) {
  std::vector<std::vector<Point>> curves;

  // 对每组控制点生成曲线并存储在 curves 中
  curves.reserve(control_points_list.size());
  for (const auto &control_points : control_points_list) {
    curves.push_back(bezier_curve(control_points, num_points));
  }
  piecewise_bezier_curve_ = curves;
}

void PiecewiseBezierFit2::composeTrimmingSdf() {
  auto start_sdf_map = std::chrono::high_resolution_clock::now();
  RFuns.clear();
  for (const auto &con_point : control_points_) {
    std::unique_ptr<Bezier2Poly> bezier2poly = std::make_unique<Bezier2Poly>(con_point, degree_);
    auto px = bezier2poly->getXCoefficients();
    auto py = bezier2poly->getYCoefficients();
    std::unique_ptr<RFunction> r_fun = std::make_unique<RFunction>(px, py, con_point);
    RFuns.push_back(std::move(r_fun));
  }
  auto end_sdf_map = std::chrono::high_resolution_clock::now();
  auto duration_sdf_map = std::chrono::duration_cast<std::chrono::microseconds>(end_sdf_map - start_sdf_map).count();
  std::cout << "com dis time: " << duration_sdf_map / 1000.0 << "ms\n";
}

double PiecewiseBezierFit2::getSdfDis(double x, double y, int p) {
  double result = RFuns[0]->trimmingArea(x, y);
  double result_pow_p = std::pow(result, p);// Cache result^p
  for (size_t i = 1; i < RFuns.size(); ++i) {
    double sdf = RFuns[i]->trimmingArea(x, y);
    double sdf_pow_p = std::pow(sdf, p);// Cache sdf^p
    double sum_powers = result_pow_p + sdf_pow_p;
    double denominator = std::pow(sum_powers, 1.0 / p);
    result = result + sdf - denominator;
    result_pow_p = std::pow(result, p);// Update cached result^p for next iteration
  }

  return result;
}
