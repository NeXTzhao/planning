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

PiecewiseBezierFit2::PiecewiseBezierFit2(std::vector<Point> &points, int degree,
                                         double maxError) {
  degree_ = degree;
  fitCurve(points, maxError);
}

void PiecewiseBezierFit2::fitCurve(std::vector<Point> &points,
                                   double maxError) {
  Point leftTangent = normalize(points[1] - points[0]);
  Point rightTangent =
      normalize(points[points.size() - 2] - points[points.size() - 1]);
  //  start_velocity_ = Eigen::Vector2d{leftTangent.x, leftTangent.y};
  //  end_velocity_ = Eigen::Vector2d{rightTangent.x, rightTangent.y};

  control_points_ = fitCubicBezier(points, leftTangent, rightTangent, maxError);
  composeTrimmingSdf();
  generate_bezier_curves(control_points_, 50);
}

std::vector<std::vector<Point>> PiecewiseBezierFit2::getControlPoints() const {
  return control_points_;
}

std::vector<std::vector<Point>>
PiecewiseBezierFit2::getPiecewiseBezierCurvesPoints() const {
  return piecewise_bezier_curve_;
}

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
Point PiecewiseBezierFit2::qprime(const std::vector<Point> &ctrlPoly,
                                  double t) {
  double t_1 = 1.0 - t;

  Point p{};
  p.x = 2 * t_1 * (ctrlPoly[1].x - ctrlPoly[0].x)
      + 2 * t * (ctrlPoly[2].x - ctrlPoly[1].x);
  p.y = 2 * t_1 * (ctrlPoly[1].y - ctrlPoly[0].y)
      + 2 * t * (ctrlPoly[2].y - ctrlPoly[1].y);
  return p;
}

// 计算二阶贝塞尔曲线的二阶导数
Point PiecewiseBezierFit2::qprimeprime(const std::vector<Point> &ctrlPoly,
                                       double t) {
  Point p{};
  p.x = 2 * (ctrlPoly[2].x - 2 * ctrlPoly[1].x + ctrlPoly[0].x);
  p.y = 2 * (ctrlPoly[2].y - 2 * ctrlPoly[1].y + ctrlPoly[0].y);
  return p;
}

/*
 *  Bezier :
 *  	Evaluate a Bezier curve at a particular parameter value
 */
Point PiecewiseBezierFit2::BezierII(int degree, const std::vector<Point> &V,
                                    double t) {
  Point Q{};// Point on curve at parameter t
  std::vector<Point> Vtemp(V.begin(),
                           V.end());// Local copy of control points

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

// 通过梯度下降法找到最佳的中间控制点 P1
std::vector<Point> PiecewiseBezierFit2::generateBezierControlPoint_GD(
    std::vector<Point> &control_points, std::vector<Point> &row_points,
    std::vector<double> &parameters, double learning_rate, int max_iterations,
    double tolerance) {
  std::vector<Eigen::Vector2d> con_pt(control_points.size());
  std::vector<Eigen::Vector2d> row_point(row_points.size());

  std::transform(control_points.begin(), control_points.end(), con_pt.begin(),
                 [](const Point &p) { return Eigen::Vector2d(p.x, p.y); });

  std::transform(row_points.begin(), row_points.end(), row_point.begin(),
                 [](const Point &p) { return Eigen::Vector2d(p.x, p.y); });

  Eigen::VectorXd t_value =
      Eigen::Map<Eigen::VectorXd>(parameters.data(), parameters.size());

  // Initial value can be taken as the average of the endpoints of the Bezier
  // curve
  Eigen::Vector2d initial_p1{0.0, 0.0};
  Point leftTangent = normalize(row_points[1] - row_points[0]);

  double segLength = std::sqrt(
      std::pow(row_points[0].x - row_points[row_points.size() - 1].x, 2)
      + std::pow(row_points[0].y - row_points[row_points.size() - 1].y, 2));

  Point projection{};
  projection.x = segLength * leftTangent.x;
  projection.y = segLength * leftTangent.y;
  double l = std::hypot(projection.x, projection.y);
  initial_p1.x() = control_points[0].x + l;
  initial_p1.y() = control_points[0].y + l;
  //    Eigen::Vector2d initial_p1 = (con_pt[0] + con_pt[1]) / 2.0;

  Eigen::Vector2d p1 = initial_p1;

  for (int iteration = 0; iteration < max_iterations; ++iteration) {
    Eigen::Vector2d gradient = compute_gradient(p1, t_value, con_pt, row_point);
    p1 -= learning_rate * gradient;
    if (gradient.lpNorm<2>() < tolerance) {
      //      printf("iteration = %d\n ", iteration);
      break;
    }
  }

  std::vector<Point> conpt = {{con_pt[0].x(), con_pt[0].y()},
                              {p1.x(), p1.y()},
                              {con_pt[1].x(), con_pt[1].y()}};

  return conpt;
}

Eigen::Vector2d PiecewiseBezierFit2::cal_bezier_value(
    double t, const Eigen::Vector2d &p0, const Eigen::Vector2d &p1,
    const Eigen::Vector2d &p2) {
  //  return (1 - t) * (1 - t) * p0 + 2 * (1 - t) * t * p1 + t * t * p2;
  double t_2 = t * t;
  double t_1_minus_t = (1 - t);

  return t_1_minus_t * t_1_minus_t * p0 + 2 * t_1_minus_t * t * p1 + t_2 * p2;
}

// 计算目标函数关于 p1 的梯度
Eigen::Vector2d PiecewiseBezierFit2::compute_gradient(
    const Eigen::Vector2d &p1, const Eigen::VectorXd &t_values,
    const std::vector<Eigen::Vector2d> &points,
    const std::vector<Eigen::Vector2d> &samples, double epsilon) {
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

double PiecewiseBezierFit2::objective_function(
    const Eigen::Vector2d &p1, const Eigen::VectorXd &t_values,
    const std::vector<Eigen::Vector2d> &control_points,
    const std::vector<Eigen::Vector2d> &row_points) {
  double error = 0;
  const Eigen::Vector2d &p0 = control_points[0];
  const Eigen::Vector2d &p2 = control_points[1];
  for (int i = 0; i < row_points.size(); ++i) {
    double t = t_values(i);
    const Eigen::Vector2d &sample = row_points[i];
    Eigen::Vector2d curve = cal_bezier_value(t, p0, p1, p2);
    error += (sample - curve).squaredNorm();
  }
  return error;
}
// L-BFGS objective function
lbfgsfloatval_t PiecewiseBezierFit2::lbfgs_objective(void *instance,
                                                     const lbfgsfloatval_t *x,
                                                     lbfgsfloatval_t *g, int n,
                                                     lbfgsfloatval_t step) {
  auto *fit = reinterpret_cast<PiecewiseBezierFit2 *>(instance);
  Eigen::Vector2d p1(x[0], x[1]);
  double f =
      fit->objective_function(p1, fit->t_value, fit->con_pt, fit->row_point);
  Eigen::Vector2d gradient =
      fit->compute_gradient(p1, fit->t_value, fit->con_pt, fit->row_point);
  g[0] = gradient.x();
  g[1] = gradient.y();
  return f;
}
std::vector<Point> PiecewiseBezierFit2::generateBezierControlPoint_LBFGS(
    std::vector<Point> &control_points, std::vector<Point> &row_points,
    std::vector<double> &parameters, double learning_rate, int max_iterations,
    double tolerance) {
  // Convert control_points and row_points to Eigen::Vector2d
  std::vector<Eigen::Vector2d> con_pt(control_points.size());
  std::vector<Eigen::Vector2d> row_pt(row_points.size());

  std::transform(control_points.begin(), control_points.end(), con_pt.begin(),
                 [](const Point &p) { return Eigen::Vector2d(p.x, p.y); });

  std::transform(row_points.begin(), row_points.end(), row_pt.begin(),
                 [](const Point &p) { return Eigen::Vector2d(p.x, p.y); });

  Eigen::VectorXd t_scale =
      Eigen::Map<Eigen::VectorXd>(parameters.data(), parameters.size());

  this->t_value = t_scale;
  this->con_pt = con_pt;
  this->row_point = row_pt;

  // Initial value can be taken as the average of the endpoints of the Bezier
  // curve
  Point leftTangent = normalize(row_points[1] - row_points[0]);
  double segLength = std::sqrt(
      std::pow(row_points[0].x - row_points[row_points.size() - 1].x, 2)
      + std::pow(row_points[0].y - row_points[row_points.size() - 1].y, 2));
  Eigen::Vector2d initial_p1{0.0, 0.0};
  //  initial_p1.x() = control_points[0].x + leftTangent.x * (segLength
  //  / 2.0); initial_p1.y() = control_points[0].y + leftTangent.y *
  //  (segLength / 2.0);
  //      Eigen::Vector2d initial_p1 = (con_pt[0] + con_pt[1]) / 2.0;
  //    Eigen::Vector2d p1 = initial_p1;

  lbfgsfloatval_t fx;
  lbfgsfloatval_t *init_p1 = lbfgs_malloc(2);

  if (init_p1 == nullptr) {
    std::cerr << "Failed to allocate memory for L-BFGS" << std::endl;
    exit(1);
  }

  // Initialize x with initial_p1 values
  init_p1[0] = initial_p1.x();
  init_p1[1] = initial_p1.y();

  lbfgs_parameter_t param;
  lbfgs_parameter_init(&param);
  // 设置最大迭代次数（max_iterations）
  param.max_iterations = max_iterations;
  // 设置求解精度（tolerance）
  param.epsilon = tolerance;
  param.linesearch = LBFGS_LINESEARCH_BACKTRACKING;
  param.max_linesearch = 1000;
  param.wolfe = 0.9;// Adjust this value to influence Wolfe condition

  // Set up lbfgs instance
  lbfgsfloatval_t *minimized_p1 = nullptr;
  int ret = lbfgs(2, init_p1, &fx, lbfgs_objective, nullptr, this, &param);
  if (ret < 0) {
    std::cerr << "L-BFGS optimization failed: " << lbfgs_strerror(ret)
              << std::endl;
  } else {
    minimized_p1 = init_p1;
  }

  std::vector<Point> conpt = {{con_pt[0].x(), con_pt[0].y()},
                              {minimized_p1[0], minimized_p1[1]},
                              {con_pt[1].x(), con_pt[1].y()}};

  lbfgs_free(init_p1);
  return conpt;
}

Point PiecewiseBezierFit2::normalize(const Point &v) {
  double length = sqrt(v.x * v.x + v.y * v.y);
  return {v.x / length, v.y / length};
}

// double PiecewiseBezierFit2::dot(const Point &vec1, const Point &vec2) {
//   return vec1.x * vec2.x + vec1.y * vec2.y;
// }

std::vector<std::vector<Point>> PiecewiseBezierFit2::fitCubicBezier(
    std::vector<Point> &points, const Point &leftTangent,
    const Point &rightTangent, double error) {
  std::vector<std::vector<Point>> bezCtlPts;
  if ((int) points.size() == 3) {
    double dist = std::sqrt(std::pow(points[0].x - points[1].x, 2)
                            + std::pow(points[0].y - points[1].y, 2))
        / 2.0;
    std::vector<Point> control_point = {
        points[0], points[0] + dist * leftTangent, points[2]};
    bezCtlPts.emplace_back(control_point);

    return bezCtlPts;
  }
  std::vector<Point> con_pt{points[0], points[points.size() - 1]};
  std::vector<double> u = chordLengthParameterize(points);

  auto control_point = generateBezierControlPoint_GD(con_pt, points, u);
  //    auto control_point = generateBezierControlPoint_LBFGS(con_pt, points,
  //    u);

  auto error_split = computeMaxError(points, control_point, u);
  double point2CurveMaxError = error_split.first;
  int splitPoint = error_split.second;

  if (point2CurveMaxError < error * error) {
    bezCtlPts.emplace_back(control_point);
    return bezCtlPts;
  }
#if 0
  if (point2CurveMaxError < error * error) {
    for (int i = 0; i < 20; i++) {
      auto uPrime = reparameterize(control_point, points, u);
      control_point = generateBezierControlPoint_GD(con_pt, points, u);
      //             control_point = generateBezierControlPoint_LBFGS(con_pt, points, u);

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
#endif
  // 把切分出切线方向
  Point centerTangent =
      normalize(points[splitPoint - 1] - points[splitPoint + 1]);
  auto negativeCenterTangent = Point{-centerTangent.x, -centerTangent.y};

  std::vector<Point> leftPoints(points.begin(),
                                points.begin() + splitPoint + 1);
  auto leftBeziers =
      fitCubicBezier(leftPoints, leftTangent, centerTangent, error);
  bezCtlPts.insert(bezCtlPts.end(), leftBeziers.begin(), leftBeziers.end());

  std::vector<Point> rightPoints(points.begin() + splitPoint, points.end());
  auto rightBeziers =
      fitCubicBezier(rightPoints, negativeCenterTangent, rightTangent, error);
  bezCtlPts.insert(bezCtlPts.end(), rightBeziers.begin(), rightBeziers.end());

  return bezCtlPts;
}

std::vector<double> PiecewiseBezierFit2::reparameterize(
    const std::vector<Point> &bezier, const std::vector<Point> &points,
    const std::vector<double> &parameters) {
  std::vector<double> newParameters;
  for (size_t i = 0; i < points.size(); ++i) {
    double u = parameters[i];
    Point point = points[i];
    double newU = newtonRaphsonRootFind(bezier, point, u);
    newParameters.push_back(newU);
  }
  return newParameters;
}

double PiecewiseBezierFit2::newtonRaphsonRootFind(const std::vector<Point> &bez,
                                                  const Point &point,
                                                  double u) {
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

std::vector<double> PiecewiseBezierFit2::chordLengthParameterize(
    const std::vector<Point> &points) {
  std::vector<double> u(points.size(),
                        0.0);// 初始化参数 t 的向量，并设置初始值为 0

  for (size_t i = 1; i < points.size(); ++i) {
    double segmentLength = std::hypot(points[i].x - points[i - 1].x,
                                      points[i].y - points[i - 1].y);
    //        std::sqrt(std::pow(points[i].x - points[i - 1].x, 2)
    //                  + std::pow(points[i].y - points[i - 1].y, 2));
    u[i] = u[i - 1] + segmentLength;
  }

  for (size_t i = 0; i < u.size(); ++i) { u[i] /= u.back(); }

  return u;
}

std::pair<double, int> PiecewiseBezierFit2::computeMaxError(
    const std::vector<Point> &points, const std::vector<Point> &bez,
    const std::vector<double> &parameters) {
  double maxDistSquared = 0.0;// 使用距离的平方作为比较，避免开方运算
  int splitPoint = static_cast<int>(points.size() / 2);
  //  std::vector<double> ts = GetTValuesForPoints(bez, points);

  for (int i = 0; i < points.size(); ++i) {
    //        double x_diff = q(bez, ts[i]).x - points[i].x;
    //        double y_diff = q(bez, ts[i]).y - points[i].y;
    double x_diff = q(bez, parameters[i]).x - points[i].x;
    double y_diff = q(bez, parameters[i]).y - points[i].y;
    double distSquared = x_diff * x_diff + y_diff * y_diff;
    if (distSquared > maxDistSquared) {
      maxDistSquared = distSquared;
      splitPoint = i;
    }
  }

  return std::make_pair(std::sqrt(maxDistSquared),
                        splitPoint);// 返回距离的平方根作为最大误差
}

// 计算二项式系数
double PiecewiseBezierFit2::binomial_coefficient(int n, int k) {
  double result = 1.0;
  for (int i = 1; i <= k; i++) { result *= static_cast<double>(n - i + 1) / i; }
  return result;
}

// 贝塞尔曲线生成函数
std::vector<Point> PiecewiseBezierFit2::bezier_curve(
    const std::vector<Point> &control_points, int num_points) {
  int n = control_points.size() - 1;
  std::vector<Point> curve_points(num_points);

  for (int i = 0; i < num_points; i++) {
    double t = static_cast<double>(i) / (num_points - 1);
    double one_minus_t = 1.0 - t;

    for (int j = 0; j <= n; j++) {
      double coeff =
          binomial_coefficient(n, j) * pow(one_minus_t, n - j) * pow(t, j);
      curve_points[i].x += coeff * control_points[j].x;
      curve_points[i].y += coeff * control_points[j].y;
    }
  }

  return curve_points;
}

// 生成多组贝塞尔曲线
void PiecewiseBezierFit2::generate_bezier_curves(
    const std::vector<std::vector<Point>> &control_points_list,
    int num_points) {
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

  std::vector<std::shared_ptr<Bezier2Poly>> bezier2poly_list;
  bezier2poly_list.reserve(control_points_.size());

  // 预分配对象池，避免频繁的内存分配
  std::vector<std::shared_ptr<RFunction>> r_fun_pool;
  r_fun_pool.reserve(control_points_.size());

  // 预先计算 px 和 py，避免重复计算
  for (const auto &con_point : control_points_) {
    bezier2poly_list.emplace_back(
        std::make_shared<Bezier2Poly>(con_point, degree_));
    auto px = bezier2poly_list.back()->getXCoefficients();
    auto py = bezier2poly_list.back()->getYCoefficients();
    r_fun_pool.emplace_back(std::make_shared<RFunction>(px, py, con_point));
  }

  // 移动 shared_ptr 到 RFuns 中
  for (auto &r_fun : r_fun_pool) { RFuns.emplace_back(std::move(r_fun)); }

  auto end_sdf_map = std::chrono::high_resolution_clock::now();
  auto duration_sdf_map = std::chrono::duration_cast<std::chrono::microseconds>(
                              end_sdf_map - start_sdf_map)
                              .count();
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
    result_pow_p =
        std::pow(result, p);// Update cached result^p for next iteration
  }

  return result;
}

void PiecewiseBezierFit2::GetFootPoint(const std::vector<Point> &bez,
                                       const Point &point, const double t0,
                                       double &t) {
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
};

std::vector<double> PiecewiseBezierFit2::GenerateKnotsFitting(
    const std::vector<Point> &bez, const std::vector<Point> &data_points,
    const size_t match_point_size) {
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
};

std::vector<double> PiecewiseBezierFit2::GetTValuesForPoints(
    const std::vector<Point> &bez, const std::vector<Point> &row_points) {
  auto initial_t_guess = GenerateKnotsFitting(bez, row_points);

  std::vector<double> tValues;

  for (int i = 0; i < row_points.size(); ++i) {
    double t;
    GetFootPoint(bez, row_points[i], initial_t_guess[i], t);
    tValues.push_back(t);
  }

  return tValues;
};