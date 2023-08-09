#include "bezier.h"

#include <cmath>
#include <stdexcept>

Point::Point(double x_, double y_) : x(x_), y(y_) {}

double Point::norm() const { return std::sqrt(x * x + y * y); }

Point Point::normalized() const {
  double length = norm();
  if (length == 0.0) {
    throw std::runtime_error("Normalization failed: Zero length vector");
  }
  return *this * (1.0 / length);
}

double Point::squaredNorm() const { return x * x + y * y; }

double Point::dot(const Point &other) const {
  return x * other.x + y * other.y;
}

double Point::cross(const Point &other) const {
  return x * other.y - y * other.x;
}

Point &Point::operator+=(const Point &other) {
  x += other.x;
  y += other.y;
  return *this;
}

Point &Point::operator-=(const Point &other) {
  x -= other.x;
  y -= other.y;
  return *this;
}

Point &Point::operator*=(double scalar) {
  x *= scalar;
  y *= scalar;
  return *this;
}

Point &Point::operator/=(double scalar) {
  if (scalar == 0.0) { throw std::runtime_error("Division by zero"); }
  x /= scalar;
  y /= scalar;
  return *this;
}

Point Point::operator+(const Point &other) const {
  return {x + other.x, y + other.y};
}

Point Point::operator-(const Point &other) const {
  return {x - other.x, y - other.y};
}

Point Point::operator*(double scalar) const { return {x * scalar, y * scalar}; }

Point Point::operator/(double scalar) const {
  if (scalar == 0.0) { throw std::runtime_error("Division by zero"); }
  return {x / scalar, y / scalar};
}

Point operator*(double scalar, const Point &p) {
  return {scalar * p.x, scalar * p.y};
}

Point operator*(const Point &p1, const Point &p2) {
  return {p1.x * p2.x, p1.y * p2.y};
}

BezierCurve::BezierCurve(const std::vector<Point> &controlPoints)
    : controlPoints_(controlPoints) {
  degree_ = controlPoints_.size() - 1;
}

Point BezierCurve::evaluate(double t) const {
  Point result = {0.0, 0.0};
  for (int i = 0; i <= degree_; ++i) {
    double blend = binomialCoefficient(degree_, i) * std::pow(t, i)
        * std::pow(1 - t, degree_ - i);
    result = result + (controlPoints_[i] * blend);
  }
  return result;
}

Point BezierCurve::firstDerivative(double t) const {
  Point result = {0.0, 0.0};
  for (int i = 0; i <= degree_ - 1; ++i) {
    double blend = binomialCoefficient(degree_ - 1, i) * (degree_ - 1)
        * (std::pow(t, i) * std::pow(1 - t, degree_ - 1 - i));
    result = result + ((controlPoints_[i + 1] - controlPoints_[i]) * blend);
  }
  return result;
}

Point BezierCurve::secondDerivative(double t) const {
  Point result = {0.0, 0.0};
  for (int i = 0; i <= degree_ - 2; ++i) {
    double blend = binomialCoefficient(degree_ - 2, i) * (degree_ - 1)
        * (degree_ - 2) * (std::pow(t, i) * std::pow(1 - t, degree_ - 2 - i));
    result = result
        + ((controlPoints_[i + 2] - 2 * controlPoints_[i + 1]
            + controlPoints_[i])
           * blend);
  }
  return result;
}

Point BezierCurve::thirdDerivative(double t) const {
  Point result = {0.0, 0.0};
  for (int i = 0; i <= degree_ - 3; ++i) {
    double blend = binomialCoefficient(degree_ - 3, i) * (degree_ - 1)
        * (degree_ - 2) * (degree_ - 3)
        * (std::pow(t, i) * std::pow(1 - t, degree_ - 3 - i));
    result = result
        + ((controlPoints_[i + 3] - 3 * controlPoints_[i + 2]
            + 3 * controlPoints_[i + 1] - controlPoints_[i])
           * blend);
  }
  return result;
}

int BezierCurve::binomialCoefficient(int n, int k) const {
  if (k == 0 || k == n) { return 1; }
  return binomialCoefficient(n - 1, k - 1) + binomialCoefficient(n - 1, k);
}

double BezierCurve::kappa(double t) const {
  Point first_derivative = firstDerivative(t);
  Point second_derivative = secondDerivative(t);

  double cross_product = first_derivative.x * second_derivative.y
      - first_derivative.y * second_derivative.x;
  double squared_norm_first = first_derivative.squaredNorm();

  double curvature = cross_product / std::pow(squared_norm_first, 1.5);
  return curvature;
}

double BezierCurve::dkappa(double t) const {
  Point first_derivative = firstDerivative(t);
  Point second_derivative = secondDerivative(t);
  Point third_derivative = thirdDerivative(t);

  double val1 = first_derivative.cross(third_derivative);
  double val2 = first_derivative.squaredNorm();

  double val3 = first_derivative.cross(second_derivative);
  double val4 = first_derivative.dot(second_derivative);

  double dkappa_res = (val1 * val2 - 3 * val3 * val4) / std::pow(val2, 2.5);
  return dkappa_res;
}

void BezierCurve::getBezierCurve(int num_points) {
  curvePoints_.resize(num_points);
  curveKappa_.resize(num_points);
  curveDkappa_.resize(num_points);

  for (int i = 0; i < num_points; i++) {
    double t = static_cast<double>(i) / (num_points - 1);
    double one_minus_t = 1.0 - t;

    for (int j = 0; j <= degree_; j++) {
      double coeff = binomialCoefficient(degree_, j)
          * pow(one_minus_t, degree_ - j) * pow(t, j);
      curvePoints_[i].x += coeff * controlPoints_[j].x;
      curvePoints_[i].y += coeff * controlPoints_[j].y;
    }

    curveKappa_[i] = kappa(t);
    curveDkappa_[i] = dkappa(t);
  }
}
