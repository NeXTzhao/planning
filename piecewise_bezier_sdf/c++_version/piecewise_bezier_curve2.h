#pragma once

#include <ceres/ceres.h>

#include <Eigen/Dense>
#include <array>
#include <memory>
#include <vector>

#include "lbfgs.h"
#include "r_function.h"

class PiecewiseBezierFit2 {
 public:
  PiecewiseBezierFit2(std::vector<Point> &points, int degree, double maxError);

  void fitCurve(std::vector<Point> &points, double maxError);

  std::vector<std::vector<Point>> getControlPoints() const;
  std::vector<std::vector<Point>> getPiecewiseBezierCurvesPoints() const;

 private:
  int degree_;
  std::vector<std::vector<Point>> control_points_;
  std::vector<std::vector<Point>> piecewise_bezier_curve_;

 public:
  std::vector<std::shared_ptr<RFunction>> RFuns;

  //  static double dot(const Point &p1, const Point &p2);

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
  static std::pair<double, int>
  computeMaxError(const std::vector<Point> &points,
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
  Point BezierII(int degree, const std::vector<Point> &V, double t);
  static double
  objective_function(const Eigen::Vector2d &p1, const Eigen::VectorXd &t_values,
                     const std::vector<Eigen::Vector2d> &control_points,
                     const std::vector<Eigen::Vector2d> &row_points);
  static Eigen::Vector2d cal_bezier_value(double t, const Eigen::Vector2d &p0,
                                          const Eigen::Vector2d &p1,
                                          const Eigen::Vector2d &p2);
  static Eigen::Vector2d
  compute_gradient(const Eigen::Vector2d &p1, const Eigen::VectorXd &t_values,
                   const std::vector<Eigen::Vector2d> &points,
                   const std::vector<Eigen::Vector2d> &samples,
                   double epsilon = 1e-3);
  static std::vector<Point> generateBezierControlPoint_GD(
      std::vector<Point> &control_points, std::vector<Point> &row_points,
      std::vector<double> &parameters, double learning_rate = 0.5,
      int max_iterations = 2000, double tolerance = 1e-1);
  std::vector<Point> generateBezierControlPoint_LBFGS(
      std::vector<Point> &control_points, std::vector<Point> &row_points,
      std::vector<double> &parameters, double learning_rate = 0.5,
      int max_iterations = 1000, double tolerance = 1);

 public:
  static lbfgsfloatval_t lbfgs_objective(void *instance,
                                         const lbfgsfloatval_t *x,
                                         lbfgsfloatval_t *g, int n,
                                         lbfgsfloatval_t step);


 private:
  Eigen::VectorXd t_value;
  std::vector<Eigen::Vector2d> con_pt;
  std::vector<Eigen::Vector2d> row_point;
//  Eigen::Vector2d start_velocity_;
//  Eigen::Vector2d end_velocity_;


 public:
  // 迭代函数的实现
  template<typename Func>
  double newton_raphson_iterate(Func func, double initial_guess,
                                double lower_bound, double upper_bound,
                                int digits, int &it) {
    double x = initial_guess;
    double x_prev = x;
    double f_val = func(x).first;

    while (it > 0) {
      it--;

      // 计算导数
      auto [f_prime, f_double_prime] = func(x);

      // 更新 x
      x_prev = x;
      x = x - f_prime / f_double_prime;

      // 确保 x 在指定范围内
      if (x < lower_bound) x = lower_bound;
      else if (x > upper_bound)
        x = upper_bound;

      // 检查是否收敛
      if (std::abs(x - x_prev) <= std::pow(6, -digits)) break;
    }

    return x;
  };

  void GetFootPoint(const std::vector<Point> &bez, const Point &point,
                    double t0, double &t);

  std::vector<double>
  GenerateKnotsFitting(const std::vector<Point> &bez,
                       const std::vector<Point> &data_points,
                       size_t match_point_size = 100);

  std::vector<double> GetTValuesForPoints(const std::vector<Point> &bez,
                                          const std::vector<Point> &row_points);
};