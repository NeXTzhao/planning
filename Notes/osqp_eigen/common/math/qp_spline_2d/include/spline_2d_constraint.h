/*
 * @Author: wangdezhao
 * @Date: 2022-04-28 15:48:22
 * @LastEditTime: 2022-04-28 18:01:49
 * @FilePath: /osqp_eigen/common/math/qp_spline_2d/include/spline_2d_constraint.h
 * @Copyright:
 */
/**
 * @file spline_2d_constraint.h
 **/

#pragma once

#include <vector>

#include "Eigen/Core"
#include "affine_constraint.h"
#include "spline_2d.h"
#include "vec2d.h"

namespace apollo {
namespace planning {

class Spline2dConstraint {
 public:
  Spline2dConstraint(const std::vector<double>& t_knots, const uint32_t order);

  // direct method
  bool AddInequalityConstraint(const Eigen::MatrixXd& constraint_matrix,
                               const Eigen::MatrixXd& constraint_boundary);
  bool AddEqualityConstraint(const Eigen::MatrixXd& constraint_matrix,
                             const Eigen::MatrixXd& constraint_boundary);

  // preset method
  /**
   *   @brief: inequality boundary constraints
   *   if no boundary, do specify either by std::infinity or let vector.size() =
   *0
   **/
  /**
   * @description: 采样点边界约束
   *    f_i(t) -x_i < boundary
   *    g_i(t) -y_i < boundary
   * @param {*}
   * @return {*}
   */
  bool Add2dBoundary(const std::vector<double>& t_coord,
                     const std::vector<double>& angle,
                     const std::vector<apollo::common::math::Vec2d>& ref_point,
                     const std::vector<double>& longitudinal_bound,
                     const std::vector<double>& lateral_bound);

  // ref point refer to derivative reference point
  bool Add2dDerivativeBoundary(
      const std::vector<double>& t_coord, const std::vector<double>& angle,
      const std::vector<apollo::common::math::Vec2d>& ref_point,
      const std::vector<double>& longitudinal_bound,
      const std::vector<double>& lateral_bound);

  // ref point refer to second derivative ref point
  bool Add2dSecondDerivativeBoundary(
      const std::vector<double>& t_coord, const std::vector<double>& angle,
      const std::vector<apollo::common::math::Vec2d>& ref_point,
      const std::vector<double>& longitudinal_bound,
      const std::vector<double>& lateral_bound);

  // ref point refer to third derivative ref point
  bool Add2dThirdDerivativeBoundary(
      const std::vector<double>& t_coord, const std::vector<double>& angle,
      const std::vector<apollo::common::math::Vec2d>& ref_point,
      const std::vector<double>& longitudinal_bound,
      const std::vector<double>& lateral_bound);

  bool AddPointConstraint(const double t, const double x, const double y);
  bool AddPointSecondDerivativeConstraint(const double t, const double ddx,
                                          const double ddy);
  bool AddPointThirdDerivativeConstraint(const double t, const double dddx,
                                         const double dddy);

  /**
   * @description: 第一个anchor
   * point的heading应该和第一段的多项式函数f1和g1的偏导数方向一致，
   *                大小可以不一致。也就是： heading = arctan(g1'(s), f1'(s))
   * @param {double} t
   * @param {double} angle
   * @return {*}
   */
  bool AddPointAngleConstraint(const double t, const double angle);

  // guarantee up to values are joint
  bool AddSmoothConstraint();

  // guarantee up to derivative are joint
  bool AddDerivativeSmoothConstraint();

  // guarantee up to second order derivative are joint
  bool AddSecondDerivativeSmoothConstraint();

  // guarantee up to third order derivative are joint
  bool AddThirdDerivativeSmoothConstraint();

  /**
   *   @brief: output interface inequality constraint
   **/
  const AffineConstraint& inequality_constraint() const;
  const AffineConstraint& equality_constraint() const;

 private:
  uint32_t FindIndex(const double t) const;

  /*添加平滑节点的系数矩阵*/
  /**
   * 作为样条曲线，要保证各段线段的衔接处连接且平滑
   *
   * f_k(s_k) = f_k+1(s0)
   * f'_k(s_k) = f'_k+1(s0)
   * f''_k(s_k) = f''_k+1(s0)
   *
   * 1  s  s^2  s^3   s^4    s^5    0  0  0    0     0     0      -1     ...
   * 0  1  2s   3s^2  4s^3   5s^4   0  0  0    0     0     0      0      -1 ...
   * 0  0  2    6s    12s^2  20s^3  0  0  0    0     0     0      0      0    -2
   * ... 0  0  0    0     0      0      1  s  s^2  s^3   s^4   s^5    0      0
   * 0    0   0  0  -1  ... 0  0  0    0     0      0      0  1  2s   3s^2  4s^3
   * 5s^4   0      0    0    0   0  0  0   -1   ... 0  0  0    0     0      0 0
   * 0  0    2     6s    12s^2  20s^3  0    0    0   0  0  0   0    0    -2  ...
   *
   *
   */
  std::vector<double> AffineCoef(const double angle, const double t) const;
  std::vector<double> AffineDerivativeCoef(const double angle,
                                           const double t) const;
  std::vector<double> AffineSecondDerivativeCoef(const double angle,
                                                 const double t) const;
  std::vector<double> AffineThirdDerivativeCoef(const double angle,
                                                const double t) const;
  /* Coefficients of polynomial 多项式系数 */
  std::vector<double> PolyCoef(const double t) const;
  std::vector<double> DerivativeCoef(const double t) const;
  std::vector<double> SecondDerivativeCoef(const double t) const;
  std::vector<double> ThirdDerivativeCoef(const double t) const;

  /**
   * @description: 用向量点乘计算距离
   * @param {Vec2d&} xy_point
   * @param {double} angle
   *
   * InnerProd(double x0, double y0, double x1, double y1)
   * @return -x*sina + y*cosa
   */
  double SignDistance(const apollo::common::math::Vec2d& xy_point,
                      const double angle) const;
  bool AddPointKthOrderDerivativeConstraint(
      const double t, const double x_kth_derivative,
      const double y_kth_derivative, const std::vector<double>& kth_coeff);

 private:
  AffineConstraint inequality_constraint_;
  AffineConstraint equality_constraint_;
  std::vector<double> t_knots_;
  uint32_t spline_order_;
  uint32_t total_param_;
};

}  // namespace planning
}  // namespace apollo
