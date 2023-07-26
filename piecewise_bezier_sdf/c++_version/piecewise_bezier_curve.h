#ifndef BEZIERFITTER_H
#define BEZIERFITTER_H

#include <array>
#include <vector>

class BezierFitter {
 public:
  struct Point {
    double x, y;
  };

  BezierFitter(double maxError);

  void fitCurve(const std::vector<Point> &points);

  std::vector<Point> getControlPoints() const;
  std::vector<std::vector<Point>> getCurves() const;

 private:
  double maxError;
  std::vector<Point> controlPoints;
  std::vector<std::vector<Point>> curves;

  //  Point operator+(const Point &p1, const Point &p2);
  //  Point operator-(const Point &p1, const Point &p2);
  //  Point operator*(double scalar, const Point &p);
  double dot(const Point &p1, const Point &p2);

  Point q(const std::array<Point, 4>&ctrlPoly, double t);
  Point qprime(const std::array<Point, 4> &ctrlPoly, double t);
  Point qprimeprime(const std::array<Point, 4> &ctrlPoly, double t);
  Point normalize(const Point &v);

  std::vector<double> chordLengthParameterize(const std::vector<Point> &points);
  double binomialCoefficient(int n, int k);

  static BezierFitter::Point generateBezier(const std::array<Point, 4> &bez, const std::vector<double> &parameters, const Point &leftTangent, const Point &rightTangent);
  std::vector<Point> bezierCurve(const std::vector<Point> &control_points, int num_points);
  std::vector<Point> fitCubic(const std::vector<Point> &points, const Point &leftTangent, const Point &rightTangent, double error);

  std::vector<std::vector<BezierFitter::Point>> get_fit_curves(const std::vector<Point> &control_points, const std::vector<Point> &original_points);

  double reparameterize(const std::array<Point, 4>& bezier, const std::vector<Point>& points, const std::vector<double>& parameters);
  double newtonRaphsonRootFind(const std::array<Point, 4> &bez, const Point &point, double u);
};

#endif// BEZIERFITTER_H
