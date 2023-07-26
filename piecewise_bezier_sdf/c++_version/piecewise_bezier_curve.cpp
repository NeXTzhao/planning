#include "piecewise_bezier_curve.h"
#include <cmath>

BezierFitter::Point operator+(const BezierFitter::Point &p1, const BezierFitter::Point &p2) {
  return {p1.x + p2.x, p1.y + p2.y};
}

BezierFitter::Point operator-(const BezierFitter::Point &p1, const BezierFitter::Point &p2) {
  return {p1.x - p2.x, p1.y - p2.y};
}

BezierFitter::Point operator*(double scalar, const BezierFitter::Point &p) {
  return {scalar * p.x, scalar * p.y};
}

BezierFitter::Point operator*(const BezierFitter::Point &p, double scalar) {
  return {scalar * p.x, scalar * p.y};
}

BezierFitter::BezierFitter(double maxError) : maxError(maxError) {}

void BezierFitter::fitCurve(const std::vector<Point> &points) {
  Point leftTangent = normalize(points[1] - points[0]);
  Point rightTangent = normalize(points[points.size() - 2] - points[points.size() - 1]);
  controlPoints = fitCubic(points, leftTangent, rightTangent, maxError);
  curves = get_fit_curves(controlPoints, points);
}

std::vector<BezierFitter::Point> BezierFitter::getControlPoints() const {
  return controlPoints;
}

std::vector<std::vector<BezierFitter::Point>> BezierFitter::getCurves() const {
  return curves;
}

double BezierFitter::dot(const Point &p1, const Point &p2) {
  return p1.x * p2.x + p1.y * p2.y;
}

BezierFitter::Point BezierFitter::q(const std::array<Point, 4> &ctrlPoly, double t) {
  double t_1 = 1.0 - t;
  return pow(t_1, 3) * ctrlPoly[0] + 3 * pow(t_1, 2) * t * ctrlPoly[1] + 3 * t_1 * pow(t, 2) * ctrlPoly[2] + pow(t, 3) * ctrlPoly[3];
}

BezierFitter::Point BezierFitter::qprime(const std::array<Point, 4> &ctrlPoly, double t) {
  double t_1 = 1.0 - t;
  return 3 * pow(t_1, 2) * (ctrlPoly[1] - ctrlPoly[0]) + 6 * t_1 * t * (ctrlPoly[2] - ctrlPoly[1]) + 3 * pow(t, 2) * (ctrlPoly[3] - ctrlPoly[2]);
}

BezierFitter::Point BezierFitter::qprimeprime(const std::array<Point, 4> &ctrlPoly, double t) {
  return 6 * (1.0 - t) * (ctrlPoly[2] - 2 * ctrlPoly[1] + ctrlPoly[0]) + 6 * (t) * (ctrlPoly[3] - 2 * ctrlPoly[2] + ctrlPoly[1]);
}

BezierFitter::Point BezierFitter::normalize(const Point &v) {
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

double BezierFitter::binomialCoefficient(int n, int k) {
  double result = 1.0;
  for (int i = 1; i <= k; ++i) {
    result *= (n - i + 1) / static_cast<double>(i);
  }
  return result;
}

BezierFitter::Point BezierFitter::generateBezier(const std::array<Point, 4> &bez, const std::vector<double> &parameters, const Point &leftTangent, const Point &rightTangent) {
  std::array<std::array<Point, 2>, 2> A{};
  std::array<Point, 2> C{};
  Point X{};

  for (size_t i = 0; i < parameters.size(); ++i) {
    double u = parameters[i];
    double u2 = u * u;
    double u3 = u2 * u;
    double oneMinusU = 1.0 - u;
    double oneMinusU2 = oneMinusU * oneMinusU;
    double oneMinusU3 = oneMinusU2 * oneMinusU;

    A[0][0] = leftTangent * (3.0 * oneMinusU2);
    A[0][1] = leftTangent * (3.0 * oneMinusU * u);
    A[1][0] = rightTangent * (3.0 * u2);
    A[1][1] = rightTangent * (3.0 * u * oneMinusU);

    C[0] = C[0] + bez[i] * oneMinusU3;
    C[1] = C[1] + bez[i] * u3;

    X = X + (A[0][0] + A[0][1] + A[1][0] + A[1][1]) * bez[i] * u2;
  }

  // Calculate the determinant
  double detA = A[0][0].x * A[1][1].y - A[1][0].x * A[0][1].y;

  // Invert the determinant to calculate alpha values
  double alpha1 = std::abs(detA) < 1e-6 ? 0.0 : (X.x * A[1][1].y - X.y * A[1][1].x) / detA;
  double alpha2 = std::abs(detA) < 1e-6 ? 0.0 : (X.y * A[0][0].x - X.x * A[0][0].y) / detA;

  // Calculate the final control points of the cubic Bezier curve
  std::array<Point, 4> bezCurve = {bez[0], bez[0] + alpha1 * leftTangent, bez[3] + alpha2 * rightTangent, bez[3]};

  return bezCurve[3];
}

std::vector<BezierFitter::Point> BezierFitter::bezierCurve(const std::vector<Point> &control_points, int num_points) {
  int n = control_points.size() - 1;
  std::vector<double> t(num_points);
  for (int i = 0; i < num_points; ++i) {
    t[i] = static_cast<double>(i) / (num_points - 1);
  }

  std::vector<Point> curve_points(num_points, {0.0, 0.0});

  for (int i = 0; i < num_points; ++i) {
    for (int j = 0; j <= n; ++j) {
      double coeff = binomialCoefficient(n, j) * pow(1 - t[i], n - j) * pow(t[i], j);
      curve_points[i] = curve_points[i] + coeff * control_points[j];
    }
  }

  return curve_points;
}

std::vector<BezierFitter::Point> BezierFitter::fitCubic(const std::vector<Point> &points, const Point &leftTangent, const Point &rightTangent, double error) {
  if (points.size() == 2) {
    double dist = std::sqrt(std::pow(points[0].x - points[1].x, 2) + std::pow(points[0].y - points[1].y, 2)) / 3.0;
    Point bezCurve[4] = {points[0], points[0] + dist * leftTangent, points[1] + dist * rightTangent, points[1]};
    return {bezCurve[0], bezCurve[1], bezCurve[2], bezCurve[3]};
  }

  std::vector<double> u = chordLengthParameterize(points);
  Point bezCurve[4] = {points[0], leftTangent, rightTangent, points.back()};
  double maxError, splitPoint;

  do {
    maxError = 0.0;
    splitPoint = points.size() / 2;
    for (size_t i = 1; i < points.size() - 1; ++i) {
      Point bez1[4], bez2[4];
      Point centerTangent = normalize(points[i - 1] - points[i + 1]);
      bez1[0] = bezCurve[0];
      bez1[1] = bezCurve[1];
      bez1[2] = bezCurve[2];
      bez1[3] = bezCurve[3];
      bez2[0] = bezCurve[3];
      bez2[1] = bezCurve[2];
      bez2[2] = bezCurve[1];
      bez2[3] = bezCurve[0];

      double uPrime = reparameterize(bez1, points[i], u[i]);
      Point bez1OnCurve = generateBezier(bez1, {uPrime, 1.0}, bez1[1] - bez1[0], bez1[2] - bez1[3]);

      uPrime = reparameterize(bez2, points[i], 1.0 - u[i]);
      Point bez2OnCurve = generateBezier(bez2, {uPrime, 1.0}, bez2[1] - bez2[0], bez2[2] - bez2[3]);

      double dist1 = std::sqrt(std::pow(points[i].x - bez1OnCurve.x, 2) + std::pow(points[i].y - bez1OnCurve.y, 2));
      double dist2 = std::sqrt(std::pow(points[i].x - bez2OnCurve.x, 2) + std::pow(points[i].y - bez2OnCurve.y, 2));

      if (dist1 < dist2) {
        bezCurve[1] = bez1[1];
        bezCurve[2] = bez1[2];
        u[i] = uPrime;
      } else {
        bezCurve[1] = bez2[1];
        bezCurve[2] = bez2[2];
        u[i] = 1.0 - uPrime;
      }

      double errorDist = std::sqrt(std::pow(points[i].x - bezCurve[1].x, 2) + std::pow(points[i].y - bezCurve[1].y, 2));
      if (errorDist > maxError) {
        maxError = errorDist;
        splitPoint = i;
      }
    }
  } while (maxError > error);

  std::vector<Point> leftCurve = fitCubic(std::vector<Point>(points.begin(), points.begin() + splitPoint + 1), leftTangent, normalize(bezCurve[2] - bezCurve[1]), error);
  std::vector<Point> rightCurve = fitCubic(std::vector<Point>(points.begin() + splitPoint, points.end()), normalize(bezCurve[1] - bezCurve[2]), rightTangent, error);

  std::vector<Point> result(leftCurve.begin(), leftCurve.end() - 1);
  result.insert(result.end(), rightCurve.begin(), rightCurve.end());

  return result;
}

std::vector<std::vector<BezierFitter::Point>> BezierFitter::get_fit_curves(const std::vector<Point> &control_points, const std::vector<Point> &original_points) {
  int num_points = original_points.size();
  std::vector<std::vector<Point>> curves;

  for (size_t i = 0; i < control_points.size() - 1; ++i) {
    Point left_tangent = (i == 0) ? control_points[i + 1] - control_points[i] : control_points[i] - control_points[i - 1];
    Point right_tangent = (i == control_points.size() - 2) ? control_points[i + 1] - control_points[i] : control_points[i + 2] - control_points[i + 1];
    std::vector<Point> curve_points = bezierCurve(control_points.data() + i, 4, num_points, left_tangent, right_tangent);
    curves.push_back(curve_points);
  }

  return curves;
}

std::vector<double> BezierFitter::reparameterize(const std::array<Point, 4>& bezier, const std::vector<Point>& points, const std::vector<double>& parameters) {
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

double BezierFitter::newtonRaphsonRootFind(const std::array<Point, 4>& bez, const Point& point, double u) {
  Point d = q(bez, u) - point;
  Point qprimeU = qprime(bez, u);
  Point qprimeprimeU = qprimeprime(bez, u);

  double numerator = d.x * qprimeU.x + d.y * qprimeU.y;
  double denominator = qprimeU.x * qprimeU.x + qprimeU.y * qprimeU.y + d.x * qprimeprimeU.x + d.y * qprimeprimeU.y;

  if (std::abs(denominator) < 1e-6) {
    return u;
  } else {
    return u - numerator / denominator;
  }
}


// Rest of the private member functions implementation
