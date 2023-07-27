#ifndef BEZIERFITTER_H
#define BEZIERFITTER_H

#include <array>
#include <vector>
struct Point {
  double x, y;
};

class BezierFitter {
 public:
  BezierFitter(const std::vector<Point> &points,double maxError);

  void fitCurve(const std::vector<Point> &points);

  std::vector<std::array<Point, 4>> getControlPoints() const;
  std::vector<std::vector<Point>> getPiecewiseBezierCurves() const;

 private:
  double maxError_;
  std::vector<std::array<Point, 4>> controlPoints;
  std::vector<std::vector<Point>> piecewise_bezier_curve;

  double dot(const Point &p1, const Point &p2);

  Point q(const std::array<Point, 4> &ctrlPoly, double t);
  Point qprime(const std::array<Point, 4> &ctrlPoly, double t);
  Point qprimeprime(const std::array<Point, 4> &ctrlPoly, double t);
  Point normalize(const Point &v);

  std::vector<double> chordLengthParameterize(const std::vector<Point> &points);

  std::array<Point, 4> generateBezierControlPoint(const std::vector<Point> &points, const std::vector<double> &parameters, const Point &leftTangent, const Point &rightTangent);
  std::vector<std::array<Point, 4>> fitCubic(const std::vector<Point> &points, const Point &leftTangent, const Point &rightTangent, double error);

  std::vector<double> reparameterize(const std::array<Point, 4> &bezier, const std::vector<Point> &points, const std::vector<double> &parameters);
  double newtonRaphsonRootFind(const std::array<Point, 4> &bez, const Point &point, double u);
  std::pair<double, int> computeMaxError(const std::vector<Point> &points, const std::array<Point, 4> &bez,
                                         const std::vector<double> &parameters);
  double binomial_coefficient(int n, int k);
  std::vector<Point> bezier_curve(const std::array<Point, 4> &control_points, int num_points);
  void generate_bezier_curves(const std::vector<std::array<Point, 4>> &control_points_list, int num_points);
};

#endif// BEZIERFITTER_H
