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
    Point BezierII(int degree, const std::vector<Point> &V, double t);
    static double objective_function(const Eigen::Vector2d &p1,
                              const Eigen::VectorXd &t_values,
                              const std::vector<Eigen::Vector2d> &control_points,
                              const std::vector<Eigen::Vector2d> &row_points);
    static Eigen::Vector2d cal_bezier_value(double t, const Eigen::Vector2d &p0,
                                     const Eigen::Vector2d &p1,
                                     const Eigen::Vector2d &p2);
    static Eigen::Vector2d compute_gradient(const Eigen::Vector2d &p1,
                                     const Eigen::VectorXd &t_values,
                                     const std::vector<Eigen::Vector2d> &points,
                                     const std::vector<Eigen::Vector2d> &samples,
                                     double epsilon = 1e-6);
    std::vector<Point> generateBezierControlPoint_GD(
        std::vector<Point> &control_points, std::vector<Point> &row_points,
        std::vector<double> &parameters, double learning_rate = 0.5,
        int max_iterations = 3000, double tolerance = 1e-6);
    std::vector<Point> generateBezierControlPoint_LBFGS(
        std::vector<Point> &control_points, std::vector<Point> &row_points,
        std::vector<double> &parameters, double learning_rate = 0.5,
        int max_iterations = 1000, double tolerance = 1);

    double calculate_projection_distance(const Eigen::Vector2d &point,
                                         const Eigen::Vector2d &p0,
                                         const Eigen::Vector2d &p1,
                                         const Eigen::Vector2d &p2);
    double binary_search_for_t(const Eigen::Vector2d &point,
                               const Eigen::Vector2d &p0,
                               const Eigen::Vector2d &p1,
                               const Eigen::Vector2d &p2);
    double projection_objective_function(
        const Eigen::Vector2d &p1, const Eigen::VectorXd &t_values,
        const std::vector<Eigen::Vector2d> &control_points,
        const std::vector<Eigen::Vector2d> &row_points);

public:
    static lbfgsfloatval_t lbfgs_objective(void *instance,
                                           const lbfgsfloatval_t *x,
                                           lbfgsfloatval_t *g, int n,
                                           lbfgsfloatval_t step);
//    static std::vector<Point> generateBezierControlPoint_Ceres(
//        std::vector<Point> &control_points, std::vector<Point> &row_points,
//        std::vector<double> &parameters, double learning_rate = 0.5,
//        int max_iterations = 100, double tolerance = 0.1);

private:
    Eigen::VectorXd t_value;
    std::vector<Eigen::Vector2d> con_pt;
    std::vector<Eigen::Vector2d> row_point;
    Eigen::Vector2d start_velocity_;
    Eigen::Vector2d end_velocity_;

public:
    Eigen::Vector2d EvaluateWithExtrapolation(double t,
                                              const Eigen::Vector2d &p0,
                                              const Eigen::Vector2d &p1,
                                              const Eigen::Vector2d &p2);
    static Eigen::Vector2d cal_bezier_derivative(double t, const Eigen::Vector2d &p0,
                                       const Eigen::Vector2d &p1,
                                       const Eigen::Vector2d &p2);
};

//struct BezierResidual {
//    BezierResidual(const Eigen::Vector2d &sample, const Eigen::Vector2d &p0,
//                   const Eigen::Vector2d &p2, const Eigen::VectorXd &t_value)
//        : sample_(sample), p0_(p0), p2_(p2), t_value_(t_value) {}
//
//    template <typename T>
//    bool operator()(const double *constp1, T *residual) const {
//        Eigen::Matrix<T, 2, 1> curve =
//            cal_bezier_value(T(t_value_(0)), p0_, p2_);
//        residual[0] = curve.x() - T(sample_.x());
//        residual[1] = curve.y() - T(sample_.y());
//        return true;
//    }
//
//private:
//    template <typename T>
//    static Eigen::Vector2d cal_bezier_value(const T t,
//                                            const Eigen::Vector2d &p0,
//                                            const Eigen::Vector2d &p1,
//                                            const Eigen::Vector2d &p2) {
//        T t_2         = t * t;
//        T t_1_minus_t = T(1.0) - t;
//
//        return t_1_minus_t * t_1_minus_t * p0 + T(2.0) * t_1_minus_t * t * p1 + t_2 * p2;
//    }
//
//    const Eigen::Vector2d &sample_;
//    const Eigen::Vector2d &p0_;
//    const Eigen::Vector2d &p2_;
//    const Eigen::VectorXd &t_value_;
//};
