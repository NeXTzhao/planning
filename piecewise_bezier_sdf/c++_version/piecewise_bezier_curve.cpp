#include "piecewise_bezier_curve.h"
#include <cmath>
#include <iostream>

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

BezierFitter::BezierFitter(const std::vector<Point> &points, double maxError) {
  maxError_ = maxError;
  fitCurve(points);
}

void BezierFitter::fitCurve(const std::vector<Point> &points) {
  Point leftTangent = normalize(points[1] - points[0]);
  Point rightTangent = normalize(points[points.size() - 2] - points[points.size() - 1]);
  controlPoints = fitCubic(points, leftTangent, rightTangent, maxError_);
  generate_bezier_curves(controlPoints, 100);
}

std::vector<std::array<Point, 4>> BezierFitter::getControlPoints() const {
  return controlPoints;
}

std::vector<std::vector<Point>> BezierFitter::getPiecewiseBezierCurves() const {
  return piecewise_bezier_curve;
}

double BezierFitter::dot(const Point &p1, const Point &p2) {
  return p1.x * p2.x + p1.y * p2.y;
}

Point BezierFitter::q(const std::array<Point, 4> &ctrlPoly, double t) {
  double t_1 = 1.0 - t;
  return pow(t_1, 3) * ctrlPoly[0] + 3 * pow(t_1, 2) * t * ctrlPoly[1] + 3 * t_1 * pow(t, 2) * ctrlPoly[2] + pow(t, 3) * ctrlPoly[3];
}

Point BezierFitter::qprime(const std::array<Point, 4> &ctrlPoly, double t) {
  double t_1 = 1.0 - t;
  return 3 * pow(t_1, 2) * (ctrlPoly[1] - ctrlPoly[0]) + 6 * t_1 * t * (ctrlPoly[2] - ctrlPoly[1]) + 3 * pow(t, 2) * (ctrlPoly[3] - ctrlPoly[2]);
}

Point BezierFitter::qprimeprime(const std::array<Point, 4> &ctrlPoly, double t) {
  return 6 * (1.0 - t) * (ctrlPoly[2] - 2 * ctrlPoly[1] + ctrlPoly[0]) + 6 * (t) * (ctrlPoly[3] - 2 * ctrlPoly[2] + ctrlPoly[1]);
}

Point BezierFitter::normalize(const Point &v) {
  double length = sqrt(v.x * v.x + v.y * v.y);
  return {v.x / length, v.y / length};
}

std::vector<double> BezierFitter::chordLengthParameterize(const std::vector<Point> &points) {
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

/**
 * @brief Use least-squares method to find Bezier control points for region.
 * @param bez
 * @param parameters
 * @param leftTangent
 * @param rightTangent
 * @return
 */
std::array<Point, 4> BezierFitter::generateBezierControlPoint(const std::vector<Point> &points, const std::vector<double> &parameters, const Point &leftTangent, const Point &rightTangent) {
  std::array<Point, 4> bezCurve{};
  bezCurve[0] = points[0];
  bezCurve[3] = points[points.size() - 1];

  int n = parameters.size();
  std::vector<Point> A(n, {0.0, 0.0});

  for (int i = 0; i < n; ++i) {
    double u = parameters[i];
    A[i].x = leftTangent.x * 3 * std::pow((1 - u), 2) * u;
    A[i].y = leftTangent.y * 3 * std::pow((1 - u), 2) * u;
    A[i].x = rightTangent.x * 3 * (1 - u) * std::pow(u, 2);
    A[i].y = rightTangent.y * 3 * (1 - u) * std::pow(u, 2);
  }

  double C00 = 0.0, C01 = 0.0, C10 = 0.0, C11 = 0.0;
  double X0 = 0.0, X1 = 0.0;

  for (int i = 0; i < n; ++i) {
    C00 += A[i].x * A[i].x;
    C01 += A[i].x * A[i].y;
    C10 += A[i].x * A[i].y;
    C11 += A[i].y * A[i].y;

    Point tmp = {points[i].x - q({points[0], points[0], points[points.size() - 1], points[points.size() - 1]}, parameters[i]).x,
                 points[i].y - q({points[0], points[0], points[points.size() - 1], points[points.size() - 1]}, parameters[i]).y};
    X0 += A[i].x * tmp.x;
    X1 += A[i].y * tmp.y;
  }

  double det_C0_C1 = C00 * C11 - C10 * C01;
  double det_C0_X = C00 * X1 - C10 * X0;
  double det_X_C1 = X0 * C11 - X1 * C01;

  double alpha_l = (det_C0_C1 == 0) ? 0.0 : det_X_C1 / det_C0_C1;
  double alpha_r = (det_C0_C1 == 0) ? 0.0 : det_C0_X / det_C0_C1;

  double segLength = std::sqrt(std::pow(points[0].x - points[points.size() - 1].x, 2) + std::pow(points[0].y - points[points.size() - 1].y, 2));
  double epsilon = 1.0e-6 * segLength;

  if (alpha_l < epsilon || alpha_r < epsilon) {
    bezCurve[1].x = bezCurve[0].x + leftTangent.x * (segLength / 3.0);
    bezCurve[1].y = bezCurve[0].y + leftTangent.y * (segLength / 3.0);
    bezCurve[2].x = bezCurve[3].x + rightTangent.x * (segLength / 3.0);
    bezCurve[2].y = bezCurve[3].y + rightTangent.y * (segLength / 3.0);
  } else {
    bezCurve[1].x = bezCurve[0].x + leftTangent.x * alpha_l;
    bezCurve[1].y = bezCurve[0].y + leftTangent.y * alpha_l;
    bezCurve[2].x = bezCurve[3].x + rightTangent.x * alpha_r;
    bezCurve[2].y = bezCurve[3].y + rightTangent.y * alpha_r;
  }

  return bezCurve;
}

std::vector<std::array<Point, 4>> BezierFitter::fitCubic(const std::vector<Point> &points, const Point &leftTangent, const Point &rightTangent, double error) {
  std::vector<std::array<Point, 4>> bezCtlPts;
  if (points.size() == 2) {
    double dist = std::sqrt(std::pow(points[0].x - points[1].x, 2) + std::pow(points[0].y - points[1].y, 2)) / 3.0;
    std::array<Point, 4> bezCtlPt = {points[0], points[0] + dist * leftTangent, points[1] + dist * rightTangent, points[1]};
    bezCtlPts.emplace_back(bezCtlPt);
    return bezCtlPts;
  }

  std::vector<double> u = chordLengthParameterize(points);
  auto bezCtlPt = generateBezierControlPoint(points, u, leftTangent, rightTangent);
  auto error_split = computeMaxError(points, bezCtlPt, u);
  double maxError = error_split.first;
  int splitPoint = error_split.second;

  if (maxError < error) {
    bezCtlPts.emplace_back(bezCtlPt);
    return bezCtlPts;
  }

  if (maxError < error * error) {
    for (int i = 0; i < 20; i++) {
      auto uPrime = reparameterize(bezCtlPt, points, u);
      bezCtlPt = generateBezierControlPoint(points, u, leftTangent, rightTangent);
      error_split = computeMaxError(points, bezCtlPt, u);
      maxError = error_split.first;
      splitPoint = error_split.second;
      if (maxError < error) {
        bezCtlPts.emplace_back(bezCtlPt);
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
  std::vector<std::array<Point, 4>> leftBeziers = fitCubic(leftPoints, leftTangent, centerTangent, error);
  bezCtlPts.insert(bezCtlPts.end(), leftBeziers.begin(), leftBeziers.end());

  std::vector<std::array<Point, 4>> rightBeziers = fitCubic(rightPoints, negativeCenterTangent, rightTangent, error);
  bezCtlPts.insert(bezCtlPts.end(), rightBeziers.begin(), rightBeziers.end());

  return bezCtlPts;
}

std::vector<double> BezierFitter::reparameterize(const std::array<Point, 4> &bezier, const std::vector<Point> &points, const std::vector<double> &parameters) {
  std::vector<double> newParameters;
  newParameters.reserve(points.size());

  for (size_t i = 0; i < points.size(); ++i) {
    double u = parameters[i];
    Point point = points[i];
    double newU = newtonRaphsonRootFind(bezier, point, u);
    newParameters.push_back(newU);
  }

  return newParameters;
}

double BezierFitter::newtonRaphsonRootFind(const std::array<Point, 4> &bez, const Point &point, double u) {
  Point d = q(bez, u) - point;
  Point qprimeU = qprime(bez, u);
  Point qprimeprimeU = qprimeprime(bez, u);

  /*
  牛顿迭代法（又名：牛顿-拉弗森法）
  不断地计算 u = u - f(u)/f'(u)
    1. f(u) = (x-x_p)* x' + (y - y_p)* y'
    2. f'(u) = (x')^2 + (y')^2 + (x-x_p)* x'' + (y - y_p)* y''
    3. if abs(f'(u)) < eplison; return
    4. else u - f(u)/f'(u)
    最终得到一个u，使得目标带点距离曲线最近
*/
  double numerator = d.x * qprimeU.x + d.y * qprimeU.y;
  double denominator = qprimeU.x * qprimeU.x + qprimeU.y * qprimeU.y + d.x * qprimeprimeU.x + d.y * qprimeprimeU.y;

  if (std::abs(denominator) < 1e-6) {
    return u;
  } else {
    return u - numerator / denominator;
  }
}

std::pair<double, int> BezierFitter::computeMaxError(const std::vector<Point> &points, const std::array<Point, 4> &bez,
                                                     const std::vector<double> &parameters) {
  double maxDist = 0.0;
  int splitPoint = static_cast<int>(points.size()) / 2;

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
double BezierFitter::binomial_coefficient(int n, int k) {
  double result = 1.0;
  for (int i = 1; i <= k; i++) {
    result *= static_cast<double>(n - i + 1) / i;
  }
  return result;
}

// 贝塞尔曲线生成函数
std::vector<Point> BezierFitter::bezier_curve(const std::array<Point, 4> &control_points, int num_points) {
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
void BezierFitter::generate_bezier_curves(const std::vector<std::array<Point, 4>> &control_points_list, int num_points) {
  std::vector<std::vector<Point>> curves;

  // 对每组控制点生成曲线并存储在 curves 中
  curves.reserve(control_points_list.size());
  for (const auto &control_points : control_points_list) {
    curves.push_back(bezier_curve(control_points, num_points));
  }
  piecewise_bezier_curve = curves;
}