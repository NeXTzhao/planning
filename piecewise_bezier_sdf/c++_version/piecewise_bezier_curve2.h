#pragma once

#include <Eigen/Dense>
#include <array>
#include <memory>
#include <vector>

#include "r_function.h"
//struct Point {
//  double x, y;
//};

class PiecewiseBezierFit2 {
 public:
  PiecewiseBezierFit2(std::vector<Point> &points, int degree, double maxError);

  void fitCurve(std::vector<Point> &points, double maxError);

  //  std::vector<std::array<Point, 3>> getControlPoints() const;
  std::vector<std::vector<Point>> getControlPoints() const;
  std::vector<std::vector<Point>> getPiecewiseBezierCurvesPoints() const;

 private:
  int degree_;
  //  std::vector<std::array<Point, 3>> control_points_;
  std::vector<std::vector<Point>> control_points_;
  std::vector<std::vector<Point>> piecewise_bezier_curve_;

 public:
  std::vector<std::shared_ptr<RFunction>> RFuns;

  static double dot(const Point &p1, const Point &p2);

  static Point q(const std::vector<Point> &ctrlPoly, double t);
  static Point qprime(const std::vector<Point> &ctrlPoly, double t);
  static Point qprimeprime(const std::vector<Point> &ctrlPoly, double t);
  static Point normalize(const Point &v);

  static std::vector<double>
  chordLengthParameterize(const std::vector<Point> &points);

  std::vector<std::vector<Point>> fitCubicBezier(std::vector<Point> &points,
                                                 const Point &leftTangent,
                                                 const Point &rightTangent,
                                                 double error);

  std::vector<double> reparameterize(const std::vector<Point> &bezier,
                                     const std::vector<Point> &points,
                                     const std::vector<double> &parameters);
  double newtonRaphsonRootFind(const std::vector<Point> &bez,
                               const Point &point, double u);
  std::pair<double, int> computeMaxError(const std::vector<Point> &points,
                                         const std::vector<Point> &bez,
                                         const std::vector<double> &parameters);
  static double binomial_coefficient(int n, int k);
  std::vector<Point> bezier_curve(const std::vector<Point> &control_points,
                                  int num_points);
  void generate_bezier_curves(
      const std::vector<std::vector<Point>> &control_points_list,
      int num_points);
  double getSdfDis(double x, double y, int p = 2);
  void composeTrimmingSdf();
  //  Point q2(const std::array<Point, 3> &ctrlPoly, double t);
  //  Point qprime2(const std::array<Point, 3> &ctrlPoly, double t);
  //  Point qprimeprime2(const std::array<Point, 3> &ctrlPoly, double t);
  //  std::array<Point, 3> computeControlPoint2(const std::vector<Point> &points, const Point &leftTangent, const Point &rightTangent);
  Point BezierII(int degree, const std::vector<Point> &V, double t);
  double objective_function(const Eigen::Vector2d &p1,
                            const Eigen::VectorXd &t_values,
                            const std::vector<Eigen::Vector2d> &points,
                            const std::vector<Eigen::Vector2d> &samples);
  Eigen::Vector2d cal_bezier_value(double t, const Eigen::Vector2d &p0,
                                   const Eigen::Vector2d &p1,
                                   const Eigen::Vector2d &p2);
  Eigen::Vector2d compute_gradient(const Eigen::Vector2d &p1,
                                   const Eigen::VectorXd &t_values,
                                   const std::vector<Eigen::Vector2d> &points,
                                   const std::vector<Eigen::Vector2d> &samples,
                                   double epsilon = 1e-6);
  //  std::vector<Point> generateBezierControlPoint(const std::vector<Eigen::Vector2d> &points, const Eigen::VectorXd &t_values, const std::vector<Eigen::Vector2d> &samples, double learning_rate = 0.01, int max_iterations = 1000);
  std::vector<Point> generateBezierControlPoint(
      std::vector<Point> &control_points, std::vector<Point> &row_points,
      std::vector<double> &parameters, const Point &leftTangent,
      const Point &rightTangent, double learning_rate = 0.01,
      int max_iterations = 50);
};
