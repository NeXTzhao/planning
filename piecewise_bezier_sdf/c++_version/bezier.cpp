#include "bezier.h"
#include <cmath>

Bezier2Poly::Point Bezier2Poly::Point::operator-(const Point &other) const {
  return {x - other.x, y - other.y};
}
std::vector<Bezier2Poly::Point> Bezier2Poly::fitCurve(const std::vector<Point> &points, double maxError, std::vector<std::vector<Point>> &controlPoints, std::vector<std::vector<Point>> &curves) {
  Point leftTangent = normalize({points[1].x - points[0].x, points[1].y - points[0].y});
  Point rightTangent = normalize({points[points.size() - 2].x - points[points.size() - 1].x,
                                  points[points.size() - 2].y - points[points.size() - 1].y});
  controlPoints = fitCubic(points, leftTangent, rightTangent, maxError);
  curves = getFitCurves(controlPoints, points);
}

Bezier2Poly::Point Bezier2Poly::q(const std::vector<Point> &ctrlPoly, double t) const {
  double oneMinusT = 1.0 - t;
  double oneMinusTSquared = oneMinusT * oneMinusT;
  double tSquared = t * t;
  return {
      oneMinusTSquared * oneMinusT * ctrlPoly[0].x + 3 * oneMinusTSquared * t * ctrlPoly[1].x + 3 * oneMinusT * tSquared * ctrlPoly[2].x + tSquared * t * ctrlPoly[3].x,
      oneMinusTSquared * oneMinusT * ctrlPoly[0].y + 3 * oneMinusTSquared * t * ctrlPoly[1].y + 3 * oneMinusT * tSquared * ctrlPoly[2].y + tSquared * t * ctrlPoly[3].y};
}

Bezier2Poly::Point Bezier2Poly::qprime(const std::vector<Point> &ctrlPoly, double t) const {
  double oneMinusT = 1.0 - t;
  return {
      3 * oneMinusT * oneMinusT * (ctrlPoly[1].x - ctrlPoly[0].x) + 6 * oneMinusT * t * (ctrlPoly[2].x - ctrlPoly[1].x) + 3 * t * t * (ctrlPoly[3].x - ctrlPoly[2].x),
      3 * oneMinusT * oneMinusT * (ctrlPoly[1].y - ctrlPoly[0].y) + 6 * oneMinusT * t * (ctrlPoly[2].y - ctrlPoly[1].y) + 3 * t * t * (ctrlPoly[3].y - ctrlPoly[2].y)};
}

Bezier2Poly::Point Bezier2Poly::qprimeprime(const std::vector<Point> &ctrlPoly, double t) const {
  return {
      6 * (1.0 - t) * (ctrlPoly[2].x - 2 * ctrlPoly[1].x + ctrlPoly[0].x) + 6 * t * (ctrlPoly[3].x - 2 * ctrlPoly[2].x + ctrlPoly[1].x),
      6 * (1.0 - t) * (ctrlPoly[2].y - 2 * ctrlPoly[1].y + ctrlPoly[0].y) + 6 * t * (ctrlPoly[3].y - 2 * ctrlPoly[2].y + ctrlPoly[1].y)};
}

std::vector<std::vector<Bezier2Poly::Point>> Bezier2Poly::fitCubic(const std::vector<Point> &points,
                                                                                 Point leftTangent,
                                                                                 Point rightTangent,
                                                                                 double error) {
  // Use heuristic if region only has two points in it
  if (points.size() == 2) {
    double dist = std::sqrt(std::pow(points[0].x - points[1].x, 2) + std::pow(points[0].y - points[1].y, 2)) / 3.0;
    std::vector<Point> bezCurve = {points[0], {points[0].x + leftTangent.x * dist, points[0].y + leftTangent.y * dist}, {points[1].x + rightTangent.x * dist, points[1].y + rightTangent.y * dist}, points[1]};
    return {bezCurve};
  }

  // Parameterize points, and attempt to fit curve
  std::vector<double> u = chordLengthParameterize(points);
  std::vector<Point> bezCurve = generateBezier(points, u, leftTangent, rightTangent);
  // Find max deviation of points to fitted curve
  double maxError;
  int splitPoint;
  std::tie(maxError, splitPoint) = computeMaxError(points, bezCurve, u);

  if (maxError < error) {
    return {bezCurve};
  }

  // If error not too large, try some reparameterization and iteration
  if (maxError < error * error) {
    for (int i = 0; i < 20; ++i) {
      std::vector<double> uPrime = reparameterize(bezCurve, points, u);
      bezCurve = generateBezier(points, uPrime, leftTangent, rightTangent);
      std::tie(maxError, splitPoint) = computeMaxError(points, bezCurve, uPrime);
      if (maxError < error) {
        return {bezCurve};
      }
      u = uPrime;
    }
  }

  // Fitting failed -- split at max error point and fit recursively
  Point centerTangent = normalize({points[splitPoint - 1].x - points[splitPoint + 1].x,
                                   points[splitPoint - 1].y - points[splitPoint + 1].y});

  std::vector<std::vector<Point>> beziers;
  std::vector<Point> leftPoints(points.begin(), points.begin() + splitPoint + 1);
  std::vector<Point> rightPoints(points.begin() + splitPoint, points.end());
  std::vector<std::vector<Point>> leftBeziers = fitCubic(leftPoints, leftTangent, centerTangent, error);
  std::vector<std::vector<Point>> rightBeziers = fitCubic(rightPoints, centerTangent, rightTangent, error);
  beziers.insert(beziers.end(), leftBeziers.begin(), leftBeziers.end());
  beziers.insert(beziers.end(), rightBeziers.begin(), rightBeziers.end());

  return beziers;
}

Bezier2Poly::Point Bezier2Poly::normalize(const Point &v) {
  double length = std::sqrt(v.x * v.x + v.y * v.y);
  if (length > 0.0) {
    return {v.x / length, v.y / length};
  }
  // 处理长度为零的情况（避免除以零）
  return {0.0, 0.0};
}

std::vector<double> Bezier2Poly::chordLengthParameterize(const std::vector<Point> &points) {
  std::vector<double> u(points.size(), 0.0);
  for (size_t i = 1; i < points.size(); ++i) {
    u[i] = u[i - 1] + std::sqrt(std::pow(points[i].x - points[i - 1].x, 2) + std::pow(points[i].y - points[i - 1].y, 2));
  }

  for (size_t i = 0; i < u.size(); ++i) {
    u[i] /= u.back();
  }

  return u;
}

std::vector<Bezier2Poly::Point> Bezier2Poly::generateBezier(const std::vector<Point> &points,
                                                                          const std::vector<double> &parameters,
                                                                          Point leftTangent, Point rightTangent) {
  std::vector<Bezier2Poly::Point> bezCurve = {points[0], {}, {}, points[points.size() - 1]};

  std::vector<std::vector<Point>> A(parameters.size(), std::vector<Point>(2));
  for (size_t i = 0; i < parameters.size(); ++i) {
    double oneMinusT = 1.0 - parameters[i];
    A[i][0] = {leftTangent.x * 3 * oneMinusT * oneMinusT * parameters[i],
               leftTangent.y * 3 * oneMinusT * oneMinusT * parameters[i]};
    A[i][1] = {rightTangent.x * 3 * parameters[i] * parameters[i] * oneMinusT,
               rightTangent.y * 3 * parameters[i] * parameters[i] * oneMinusT};
  }

  Point C[2] = {{0.0, 0.0}, {0.0, 0.0}};
  Point X = {0.0, 0.0};

  for (size_t i = 0; i < parameters.size(); ++i) {
    C[0].x += A[i][0].x * A[i][0].x;
    C[0].y += A[i][0].y * A[i][0].y;
    C[1].x += A[i][0].x * A[i][1].x;
    C[1].y += A[i][0].y * A[i][1].y;
    C[0].y += A[i][1].x * A[i][1].x;
    C[1].y += A[i][1].y * A[i][1].y;

    Point tmp = {points[i].x - q({points[0], points[0], points[points.size() - 1], points[points.size() - 1]}, parameters[i]).x,
                 points[i].y - q({points[0], points[0], points[points.size() - 1], points[points.size() - 1]}, parameters[i]).y};

    X.x += A[i][0].x * tmp.x + A[i][0].y * tmp.y;
    X.y += A[i][1].x * tmp.x + A[i][1].y * tmp.y;
  }

  // Compute the determinants of C and X
  double det_C0_C1 = C[0].x * C[1].y - C[1].x * C[0].y;
  double det_C0_X = C[0].x * X.y - C[1].x * X.x;
  double det_X_C1 = X.x * C[1].y - X.y * C[0].y;

  // Finally, derive alpha values
  double alphaL = (det_C0_C1 == 0) ? 0.0 : det_X_C1 / det_C0_C1;
  double alphaR = (det_C0_C1 == 0) ? 0.0 : det_C0_X / det_C0_C1;

  double segLength = std::sqrt(std::pow(points[0].x - points[points.size() - 1].x, 2) + std::pow(points[0].y - points[points.size() - 1].y, 2));
  double epsilon = 1.0e-6 * segLength;

  if (alphaL < epsilon || alphaR < epsilon) {
    // Fall back on standard (probably inaccurate) formula, and subdivide further if needed.
    bezCurve[1] = {bezCurve[0].x + leftTangent.x * (segLength / 3.0), bezCurve[0].y + leftTangent.y * (segLength / 3.0)};
    bezCurve[2] = {bezCurve[3].x + rightTangent.x * (segLength / 3.0), bezCurve[3].y + rightTangent.y * (segLength / 3.0)};
  } else {
    bezCurve[1] = {bezCurve[0].x + leftTangent.x * alphaL, bezCurve[0].y + leftTangent.y * alphaL};
    bezCurve[2] = {bezCurve[3].x + rightTangent.x * alphaR, bezCurve[3].y + rightTangent.y * alphaR};
  }

  return bezCurve;
}

std::vector<double> Bezier2Poly::reparameterize(const std::vector<Point> &bezier, const std::vector<Point> &points,
                                                       const std::vector<double> &parameters) {
  std::vector<double> u(parameters.size(), 0.0);
  for (size_t i = 0; i < parameters.size(); ++i) {
    u[i] = newtonRaphsonRootFind(bezier, points[i], parameters[i]);
  }
  return u;
}

double Bezier2Poly::newtonRaphsonRootFind(const std::vector<Point> &bez, const Point &point, double u) {
  //  Point d = Point{q(bez, u).x - point.x, q(bez, u).y - point.y};
  Point d = q(bez, u) - point;
  double numerator = d.x * qprime(bez, u).x + d.y * qprime(bez, u).y;
  double denominator = std::pow(qprime(bez, u).x, 2) + std::pow(qprime(bez, u).y, 2) + d.x * qprimeprime(bez, u).x + d.y * qprimeprime(bez, u).y;

  if (denominator == 0.0) {
    return u;
  } else {
    return u - numerator / denominator;
  }
}
std::tuple<double, int> Bezier2Poly::computeMaxError(const std::vector<Point> &points,
                                                            const std::vector<Point> &bez, const std::vector<double> &parameters) {
  double maxDist = 0.0;
  int splitPoint = points.size() / 2;

  for (size_t i = 0; i < points.size(); ++i) {
    double distSquared = std::pow(q(bez, parameters[i]).x - points[i].x, 2) + std::pow(q(bez, parameters[i]).y - points[i].y, 2);
    if (distSquared > maxDist) {
      maxDist = distSquared;
      splitPoint = i;
    }
  }
  return std::make_tuple(maxDist, splitPoint);
}

std::vector<std::vector<Bezier2Poly::Point>> Bezier2Poly::getFitCurves(
    const std::vector<std::vector<Point>> &controlPoints, const std::vector<Point> &points) {
  std::vector<std::vector<Point>> curves;
  for (const auto &con : controlPoints) {
    curves.push_back(generateBezier(con, chordLengthParameterize(points), {0.0, 0.0}, {0.0, 0.0}));
  }
  return curves;
}
