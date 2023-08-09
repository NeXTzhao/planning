#pragma once

#include <cmath>
#include <vector>
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

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
  std::vector<Point> curvePoints_;
  std::vector<double> curveKappa_;
  std::vector<double> curveDkappa_;
  int binomialCoefficient(int n, int k) const;

 protected:
  int degree_{};
  std::vector<Point> controlPoints_;

 public:
  explicit BezierCurve(const std::vector<Point> &controlPoints);
  BezierCurve() = default;

  Point evaluate(double t) const;

  Point firstDerivative(double t) const;
  Point secondDerivative(double t) const;
  Point thirdDerivative(double t) const;

  double kappa(double t) const;
  double dkappa(double t) const;

  std::vector<Point> getCurvePoints() { return curvePoints_; };
  std::vector<double> getCurveKappa() { return curveKappa_; };
  std::vector<double> getCurveDkappa() { return curveDkappa_; };

  void getBezierCurve(int num_points = 50) ;

  void vis_curvature() {
    std::vector<double> x, y, kappa, dkappa;
    auto points = this->getCurvePoints();
    auto k = this->getCurveKappa();
    auto dk = this->getCurveDkappa();

    for (int i = 0; i < points.size(); ++i) {
      x.push_back(points.at(i).x);
      y.push_back(points.at(i).y);

      kappa.push_back(k[i]);
      dkappa.push_back(dk[i]);
    }
    plt::named_plot("local trajectory", x, y, "r");
//    plt::figure();
//    plt::named_plot("local trajectory kappa", kappa, "b-");
//    plt::named_plot("local trajectory dkappa", dkappa, "r-");
    plt::grid(true);
    plt::legend();
  }
};
