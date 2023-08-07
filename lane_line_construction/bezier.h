#pragma once

#include <cmath>
#include <vector>

//struct Point {
//  double x, y;
//
//  double norm() const { return std::sqrt(x * x + y * y); }
//  Point normalized() const { return {x / norm(), y / norm()}; }
//  double squaredNorm() const { return x * x + y * y; }
//  double dot(const Point &other) const { return x * other.x + y * other.y; }
//  double cross(const Point &other) const { return x * other.y - y * other.x; }
//
//};
//
//Point operator+(const Point &p1, const Point &p2) {
//  return {p1.x + p2.x, p1.y + p2.y};
//}
//
//Point operator-(const Point &p1, const Point &p2) {
//  return {p1.x - p2.x, p1.y - p2.y};
//}
//
//Point operator*(double scalar, const Point &p) {
//  return {scalar * p.x, scalar * p.y};
//}
//
//Point operator*(const Point &p, double scalar) {
//  return {scalar * p.x, scalar * p.y};
//}
//
//Point operator*(const Point &p1, const Point &p2) {
//  return {p1.x * p2.x, p1.y * p2.y};
//}

struct Point {
  double x, y;

  Point() = default;
  Point(double x_, double y_);

  double norm() const;
  Point normalized() const;
  double squaredNorm() const;
  double dot(const Point &other) const;
  double cross(const Point &other) const;

  Point &operator+=(const Point &other);
  Point &operator-=(const Point &other);
  Point &operator*=(double scalar);
  Point &operator/=(double scalar);

  Point operator+(const Point &other) const;
  Point operator-(const Point &other) const;
  Point operator*(double scalar) const;
  Point operator/(double scalar) const;
};

Point operator*(double scalar, const Point &p);
Point operator*(const Point &p1, const Point &p2);

class BezierCurve {
 private:
  std::vector<Point> controlPoints_;
  int degree_;

  int binomialCoefficient(int n, int k) const;

 public:
  explicit BezierCurve(const std::vector<Point> &controlPoints);

  Point evaluate(double t) const;

  Point firstDerivative(double t) const;
  Point secondDerivative(double t) const;
  Point thirdDerivative(double t) const;

  double kappa(double t) const;
  double dkappa(double t) const;

  std::vector<Point> getBezierCurvePoints(
      const std::vector<Point> &control_points, int num_points = 50);
};
