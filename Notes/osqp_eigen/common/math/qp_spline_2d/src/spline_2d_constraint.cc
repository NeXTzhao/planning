/**
 * @file : spline_smoother_constraint.cc
 **/

#include "spline_2d_constraint.h"

#include <algorithm>

// #include "cyber/common/log.h"
#include "angle.h"
#include "math_utils.h"

namespace apollo {
namespace planning {

using apollo::common::math::Vec2d;

Spline2dConstraint::Spline2dConstraint(const std::vector<double>& t_knots,
                                       const uint32_t order)
    : t_knots_(t_knots), spline_order_(order) {
  inequality_constraint_.SetIsEquality(false);
  equality_constraint_.SetIsEquality(true);
  total_param_ =
      2 * (spline_order_ + 1) * (static_cast<uint32_t>(t_knots.size()) - 1);
}

// direct method
bool Spline2dConstraint::AddInequalityConstraint(
    const Eigen::MatrixXd& constraint_matrix,
    const Eigen::MatrixXd& constraint_boundary) {
  return inequality_constraint_.AddConstraint(constraint_matrix,
                                              constraint_boundary);
}

bool Spline2dConstraint::AddEqualityConstraint(
    const Eigen::MatrixXd& constraint_matrix,
    const Eigen::MatrixXd& constraint_boundary) {
  return equality_constraint_.AddConstraint(constraint_matrix,
                                            constraint_boundary);
}

// preset method
/**
 *   @brief: inequality boundary constraints
 *   if no boundary, do specify either by std::infinity or let vector.size() = 0
 *
 *
 **/
bool Spline2dConstraint::Add2dBoundary(
    const std::vector<double>& t_coord, const std::vector<double>& angle,
    const std::vector<Vec2d>& ref_point,
    const std::vector<double>& longitudinal_bound,
    const std::vector<double>& lateral_bound) {
  if (t_coord.size() != angle.size() || angle.size() != ref_point.size() ||
      ref_point.size() != lateral_bound.size() ||
      lateral_bound.size() != longitudinal_bound.size()) {
    return false;
  }
  Eigen::MatrixXd affine_inequality =
      Eigen::MatrixXd::Zero(4 * t_coord.size(), total_param_);
  Eigen::MatrixXd affine_boundary =
      Eigen::MatrixXd::Zero(4 * t_coord.size(), 1);
  for (uint32_t i = 0; i < t_coord.size(); ++i) {
    //横向偏差
    const double d_lateral = SignDistance(ref_point[i], angle[i]);
    //纵向偏差
    const double d_longitudinal =
        SignDistance(ref_point[i], angle[i] - M_PI / 2.0);
    const uint32_t index = FindIndex(t_coord[i]);
    const double rel_t = t_coord[i] - t_knots_[index];
    const uint32_t index_offset = 2 * index * (spline_order_ + 1);
    std::vector<double> longi_coef = AffineCoef(angle[i], rel_t);
    std::cout << "longi_coef:"<<longi_coef.size()<<"total_param_"<<total_param_<<std::endl;

    std::vector<double> longitudinal_coef =
        AffineCoef(angle[i] - M_PI / 2, rel_t);
    for (uint32_t j = 0; j < 2 * (spline_order_ + 1); ++j) {
      // upper longi
      affine_inequality(4 * i, index_offset + j) = longi_coef[j];
      // lower longi
      affine_inequality(4 * i + 1, index_offset + j) = -longi_coef[j];
      // upper longitudinal
      affine_inequality(4 * i + 2, index_offset + j) = longitudinal_coef[j];
      // lower longitudinal
      affine_inequality(4 * i + 3, index_offset + j) = -longitudinal_coef[j];
    }

    affine_boundary(4 * i, 0) = d_lateral - lateral_bound[i];
    affine_boundary(4 * i + 1, 0) = -d_lateral - lateral_bound[i];
    affine_boundary(4 * i + 2, 0) = d_longitudinal - longitudinal_bound[i];
    affine_boundary(4 * i + 3, 0) = -d_longitudinal - longitudinal_bound[i];
  }
  return AddInequalityConstraint(affine_inequality, affine_boundary);
}

bool Spline2dConstraint::Add2dDerivativeBoundary(
    const std::vector<double>& t_coord, const std::vector<double>& angle,
    const std::vector<Vec2d>& ref_point,
    const std::vector<double>& longitudinal_bound,
    const std::vector<double>& lateral_bound) {
  if (t_coord.size() != angle.size() || angle.size() != ref_point.size() ||
      ref_point.size() != lateral_bound.size() ||
      lateral_bound.size() != longitudinal_bound.size()) {
    return false;
  }
  Eigen::MatrixXd affine_inequality =
      Eigen::MatrixXd::Zero(4 * t_coord.size(), total_param_);
  Eigen::MatrixXd affine_boundary =
      Eigen::MatrixXd::Zero(4 * t_coord.size(), 1);
  for (uint32_t i = 0; i < t_coord.size(); ++i) {
    const double d_lateral = SignDistance(ref_point[i], angle[i]);
    const double d_longitudinal =
        SignDistance(ref_point[i], angle[i] - M_PI / 2.0);
    const uint32_t index = FindIndex(t_coord[i]);
    const double rel_t = t_coord[i] - t_knots_[index];
    const uint32_t index_offset = 2 * index * (spline_order_ + 1);
    std::vector<double> longi_coef = AffineDerivativeCoef(angle[i], rel_t);
    std::vector<double> longitudinal_coef =
        AffineDerivativeCoef(angle[i] - M_PI / 2, rel_t);
    for (uint32_t j = 0; j < 2 * (spline_order_ + 1); ++j) {
      // upper longi 设置L轴上界不等式系数
      affine_inequality(4 * i, index_offset + j) = longi_coef[j];
      // lower longi 设置L轴下界不等式系数
      affine_inequality(4 * i + 1, index_offset + j) = -longi_coef[j];
      // upper longitudinal 设置F轴上界不等式系数
      affine_inequality(4 * i + 2, index_offset + j) = longitudinal_coef[j];
      // lower longitudinal 设置F轴下界不等式系数
      affine_inequality(4 * i + 3, index_offset + j) = -longitudinal_coef[j];
    }

    affine_boundary(4 * i, 0) =
        d_lateral - lateral_bound[i];  //设置L轴上界不等式的边界
    affine_boundary(4 * i + 1, 0) =
        -d_lateral - lateral_bound[i];  //设置L轴下界不等式的边界
    affine_boundary(4 * i + 2, 0) =
        d_longitudinal - longitudinal_bound[i];  //设置F轴上界不等式的边界
    affine_boundary(4 * i + 3, 0) =
        -d_longitudinal - longitudinal_bound[i];  //设置F轴下界不等式的边界
  }
  return AddInequalityConstraint(affine_inequality, affine_boundary);
}

bool Spline2dConstraint::Add2dSecondDerivativeBoundary(
    const std::vector<double>& t_coord, const std::vector<double>& angle,
    const std::vector<Vec2d>& ref_point,
    const std::vector<double>& longitudinal_bound,
    const std::vector<double>& lateral_bound) {
  if (t_coord.size() != angle.size() || angle.size() != ref_point.size() ||
      ref_point.size() != lateral_bound.size() ||
      lateral_bound.size() != longitudinal_bound.size()) {
    return false;
  }
  Eigen::MatrixXd affine_inequality =
      Eigen::MatrixXd::Zero(4 * t_coord.size(), total_param_);
  Eigen::MatrixXd affine_boundary =
      Eigen::MatrixXd::Zero(4 * t_coord.size(), 1);
  for (uint32_t i = 0; i < t_coord.size(); ++i) {
    const double d_lateral = SignDistance(ref_point[i], angle[i]);
    const double d_longitudinal =
        SignDistance(ref_point[i], angle[i] - M_PI / 2.0);
    const uint32_t index = FindIndex(t_coord[i]);
    const double rel_t = t_coord[i] - t_knots_[index];
    const uint32_t index_offset = 2 * index * (spline_order_ + 1);
    std::vector<double> longi_coef =
        AffineSecondDerivativeCoef(angle[i], rel_t);
    std::vector<double> longitudinal_coef =
        AffineSecondDerivativeCoef(angle[i] - M_PI / 2, rel_t);
    for (uint32_t j = 0; j < 2 * (spline_order_ + 1); ++j) {
      // upper longi
      affine_inequality(4 * i, index_offset + j) = longi_coef[j];
      // lower longi
      affine_inequality(4 * i + 1, index_offset + j) = -longi_coef[j];
      // upper longitudinal
      affine_inequality(4 * i + 2, index_offset + j) = longitudinal_coef[j];
      // lower longitudinal
      affine_inequality(4 * i + 3, index_offset + j) = -longitudinal_coef[j];
    }

    affine_boundary(4 * i, 0) = d_lateral - lateral_bound[i];
    affine_boundary(4 * i + 1, 0) = -d_lateral - lateral_bound[i];
    affine_boundary(4 * i + 2, 0) = d_longitudinal - longitudinal_bound[i];
    affine_boundary(4 * i + 3, 0) = -d_longitudinal - longitudinal_bound[i];
  }
  return AddInequalityConstraint(affine_inequality, affine_boundary);
}

bool Spline2dConstraint::Add2dThirdDerivativeBoundary(
    const std::vector<double>& t_coord, const std::vector<double>& angle,
    const std::vector<Vec2d>& ref_point,
    const std::vector<double>& longitudinal_bound,
    const std::vector<double>& lateral_bound) {
  if (t_coord.size() != angle.size() || angle.size() != ref_point.size() ||
      ref_point.size() != lateral_bound.size() ||
      lateral_bound.size() != longitudinal_bound.size()) {
    return false;
  }
  Eigen::MatrixXd affine_inequality =
      Eigen::MatrixXd::Zero(4 * t_coord.size(), total_param_);
  Eigen::MatrixXd affine_boundary =
      Eigen::MatrixXd::Zero(4 * t_coord.size(), 1);
  for (uint32_t i = 0; i < t_coord.size(); ++i) {
    const double d_lateral = SignDistance(ref_point[i], angle[i]);
    const double d_longitudinal =
        SignDistance(ref_point[i], angle[i] - M_PI / 2.0);
    const uint32_t index = FindIndex(t_coord[i]);
    const double rel_t = t_coord[i] - t_knots_[index];
    const uint32_t index_offset = 2 * index * (spline_order_ + 1);
    std::vector<double> longi_coef = AffineThirdDerivativeCoef(angle[i], rel_t);
    std::vector<double> longitudinal_coef =
        AffineThirdDerivativeCoef(angle[i] - M_PI / 2, rel_t);
    for (uint32_t j = 0; j < 2 * (spline_order_ + 1); ++j) {
      // upper longi
      affine_inequality(4 * i, index_offset + j) = longi_coef[j];
      // lower longi
      affine_inequality(4 * i + 1, index_offset + j) = -longi_coef[j];
      // upper longitudinal
      affine_inequality(4 * i + 2, index_offset + j) = longitudinal_coef[j];
      // lower longitudinal
      affine_inequality(4 * i + 3, index_offset + j) = -longitudinal_coef[j];
    }

    affine_boundary(4 * i, 0) = d_lateral - lateral_bound[i];
    affine_boundary(4 * i + 1, 0) = -d_lateral - lateral_bound[i];
    affine_boundary(4 * i + 2, 0) = d_longitudinal - longitudinal_bound[i];
    affine_boundary(4 * i + 3, 0) = -d_longitudinal - longitudinal_bound[i];
  }
  return AddInequalityConstraint(affine_inequality, affine_boundary);
}

bool Spline2dConstraint::AddPointConstraint(const double t, const double x,
                                            const double y) {
  const uint32_t index = FindIndex(t);
  const double rel_t = t - t_knots_[index];
  std::vector<double> coef = PolyCoef(rel_t);
  return AddPointKthOrderDerivativeConstraint(t, x, y, coef);
}

bool Spline2dConstraint::AddPointSecondDerivativeConstraint(const double t,
                                                            const double ddx,
                                                            const double ddy) {
  const size_t index = FindIndex(t);
  const double rel_t = t - t_knots_[index];
  std::vector<double> coef = SecondDerivativeCoef(rel_t);
  return AddPointKthOrderDerivativeConstraint(t, ddx, ddy, coef);
}

bool Spline2dConstraint::AddPointThirdDerivativeConstraint(const double t,
                                                           const double dddx,
                                                           const double dddy) {
  const size_t index = FindIndex(t);
  const double rel_t = t - t_knots_[index];
  std::vector<double> coef = ThirdDerivativeCoef(rel_t);
  return AddPointKthOrderDerivativeConstraint(t, dddx, dddy, coef);
}

bool Spline2dConstraint::AddPointKthOrderDerivativeConstraint(
    const double t, const double x_kth_derivative,
    const double y_kth_derivative, const std::vector<double>& coef) {
  const uint32_t num_params = spline_order_ + 1;
  Eigen::MatrixXd affine_equality = Eigen::MatrixXd::Zero(2, total_param_);
  Eigen::MatrixXd affine_boundary = Eigen::MatrixXd::Zero(2, 1);
  affine_boundary << x_kth_derivative, y_kth_derivative;
  const size_t index = FindIndex(t);
  const size_t index_offset = index * 2 * num_params;
  for (size_t i = 0; i < num_params; ++i) {
    affine_equality(0, i + index_offset) = coef[i];
    affine_equality(1, i + num_params + index_offset) = coef[i];
  }
  return AddEqualityConstraint(affine_equality, affine_boundary);
}

bool Spline2dConstraint::AddPointAngleConstraint(const double t,
                                                 const double angle) {
  // 所属的分段曲线索引index，也就是判断它属于哪一条曲线
  const uint32_t index = FindIndex(t);
  // 曲线的参数个数
  const uint32_t num_params = spline_order_ + 1;
  // ndex_offset是该段函数参数(Ai,Bi)所在的位置偏移
  const uint32_t index_offset = index * 2 * num_params;
  // rel_t是第一个点的自变量，归一化到[0,1]
  const double rel_t = t - t_knots_[index];

  // add equality constraint
  Eigen::MatrixXd affine_equality = Eigen::MatrixXd::Zero(1, total_param_);
  Eigen::MatrixXd affine_boundary = Eigen::MatrixXd::Zero(1, 1);
  //一阶导数系数 方向一致等价于：斜率在L轴方向上的分量为0
  std::vector<double> line_derivative_coef = AffineDerivativeCoef(angle, rel_t);
  for (uint32_t i = 0; i < line_derivative_coef.size(); ++i) {
    affine_equality(0, i + index_offset) = line_derivative_coef[i];
  }

  // add inequality constraint
  Eigen::MatrixXd affine_inequality = Eigen::MatrixXd::Zero(2, total_param_);
  const Eigen::MatrixXd affine_inequality_boundary =
      Eigen::MatrixXd::Zero(2, 1);
  std::vector<double> t_coef = DerivativeCoef(rel_t);
  int x_sign = 1;
  int y_sign = 1;
  //角度归一化处理 将角度限制在(-π，π)之间
  double normalized_angle = fmod(angle, M_PI * 2);
  /**
   * 如果正则化heading在[0, pi/2]: sgn = [1, 1]
   * 如果正则化heading在[pi/2, pi]: sgn = [-1, 1]
   * 如果正则化heading在[pi,3*pi/2]: sgn = [-1, -1]
   * 如果正则化heading在[3*pi/2, 2*pi]: sgn = [1, -1]
   *
   */
  if (normalized_angle < 0) {
    normalized_angle += M_PI * 2;
  }

  if (normalized_angle > (M_PI / 2) && normalized_angle < (M_PI * 1.5)) {
    x_sign = -1;
  }

  if (normalized_angle >= M_PI) {
    y_sign = -1;
  }

  for (uint32_t i = 0; i < t_coef.size(); ++i) {
    affine_inequality(0, i + index_offset) = t_coef[i] * x_sign;
    affine_inequality(1, i + index_offset + num_params) = t_coef[i] * y_sign;
  }
  if (!AddEqualityConstraint(affine_equality, affine_boundary)) {
    return false;
  }
  return AddInequalityConstraint(affine_inequality, affine_inequality_boundary);
}

// guarantee up to values are joint
bool Spline2dConstraint::AddSmoothConstraint() {
  if (t_knots_.size() < 3) {
    return false;
  }
  Eigen::MatrixXd affine_equality =
      Eigen::MatrixXd::Zero(2 * (t_knots_.size() - 2), total_param_);
  Eigen::MatrixXd affine_boundary =
      Eigen::MatrixXd::Zero(2 * (t_knots_.size() - 2), 1);
  for (uint32_t i = 0; i + 2 < t_knots_.size(); ++i) {
    const double rel_t = t_knots_[i + 1] - t_knots_[i];
    const uint32_t num_params = spline_order_ + 1;
    const uint32_t index_offset = 2 * i * num_params;
    std::vector<double> power_t = PolyCoef(rel_t);

    for (uint32_t j = 0; j < num_params; ++j) {
      affine_equality(2 * i, j + index_offset) = power_t[j];
      affine_equality(2 * i + 1, j + index_offset + num_params) = power_t[j];
    }
    affine_equality(2 * i, index_offset + 2 * num_params) = -1.0;
    affine_equality(2 * i + 1, index_offset + 3 * num_params) = -1.0;
  }
  return AddEqualityConstraint(affine_equality, affine_boundary);
}

// guarantee up to derivative are joint
bool Spline2dConstraint::AddDerivativeSmoothConstraint() {
  if (t_knots_.size() < 3) {
    return true;
  }
  Eigen::MatrixXd affine_equality =
      Eigen::MatrixXd::Zero(4 * (t_knots_.size() - 2), total_param_);
  Eigen::MatrixXd affine_boundary =
      Eigen::MatrixXd::Zero(4 * (t_knots_.size() - 2), 1);

  for (uint32_t i = 0; i + 2 < t_knots_.size(); ++i) {
    const double rel_t = t_knots_[i + 1] - t_knots_[i];
    const uint32_t num_params = spline_order_ + 1;
    const uint32_t index_offset = 2 * i * num_params;
    std::vector<double> power_t = PolyCoef(rel_t);
    std::vector<double> derivative_t = DerivativeCoef(rel_t);
    for (uint32_t j = 0; j < num_params; ++j) {
      affine_equality(4 * i, j + index_offset) = power_t[j];
      affine_equality(4 * i + 1, j + index_offset) = derivative_t[j];
      affine_equality(4 * i + 2, j + index_offset + num_params) = power_t[j];
      affine_equality(4 * i + 3, j + index_offset + num_params) =
          derivative_t[j];
    }
    affine_equality(4 * i, index_offset + 2 * num_params) = -1.0;
    affine_equality(4 * i + 1, index_offset + 2 * num_params + 1) = -1.0;
    affine_equality(4 * i + 2, index_offset + 3 * num_params) = -1.0;
    affine_equality(4 * i + 3, index_offset + 3 * num_params + 1) = -1.0;
  }
  return AddEqualityConstraint(affine_equality, affine_boundary);
}

// guarantee up to second order derivative are joint
bool Spline2dConstraint::AddSecondDerivativeSmoothConstraint() {
  if (t_knots_.size() < 3) {
    return true;
  }
  // 6个等式，affine_equality是系数，affine_boundary是值。约束函数数量：
  // 6 * (n-1), n=t_knots_.size()-1
  Eigen::MatrixXd affine_equality =
      Eigen::MatrixXd::Zero(6 * (t_knots_.size() - 2), total_param_);
  Eigen::MatrixXd affine_boundary =
      Eigen::MatrixXd::Zero(6 * (t_knots_.size() - 2), 1);
  // 相邻两个knots对之间的多项式拟合函数进行约束
  for (uint32_t i = 0; i + 2 < t_knots_.size(); ++i) {
    // 计算第一个曲线的自变量：t_knots[i+1].s-t_knots[i].s
    const double rel_t = t_knots_[i + 1] - t_knots_[i];
    const uint32_t num_params = spline_order_ + 1;
    const uint32_t index_offset = 2 * i * num_params;
    // 函数值系数: [1, s, s^2, s^3, s^4, s^5]
    std::vector<double> power_t = PolyCoef(rel_t);
    // 一阶导系数: [0, 1, 2s, 3s^2, 4s^3, 5s^4]
    std::vector<double> derivative_t = DerivativeCoef(rel_t);
    // 二阶导系数: [0, 0, 2, 6s, 12s^2,20s^3]
    std::vector<double> second_derivative_t = SecondDerivativeCoef(rel_t);
    for (uint32_t j = 0; j < num_params; ++j) {
      affine_equality(6 * i, j + index_offset) =
          power_t[j];  // 第一个多项式x曲线终点函数值
      affine_equality(6 * i + 1, j + index_offset) =
          derivative_t[j];  // 第一个多项式x曲线终点一阶导
      affine_equality(6 * i + 2, j + index_offset) =
          second_derivative_t[j];  // 第一个多项式x曲线终点二阶导
      affine_equality(6 * i + 3, j + index_offset + num_params) =
          power_t[j];  // 第二个多项式y曲线终点函数值
      affine_equality(6 * i + 4, j + index_offset + num_params) =
          derivative_t[j];  // 第二个多项式y曲线终点一阶导
      affine_equality(6 * i + 5, j + index_offset + num_params) =
          second_derivative_t[j];  // 第二个多项式y曲线终点二阶导
    }
    //后一段曲线的起始点 s=0 
    affine_equality(6 * i, index_offset + 2 * num_params) =
        -1.0;  // 第一个多项式x曲线终点函数值 - 第二个多项式x曲线起点函数值
    affine_equality(6 * i + 1, index_offset + 2 * num_params + 1) =
        -1.0;  // 第一个多项式x曲线终点一阶导 -
               // 第二个多项式x曲线起点一阶导(速度一致)
    affine_equality(6 * i + 2, index_offset + 2 * num_params + 2) =
        -2.0;  // 第一个多项式x曲线终点二阶导 -
               // 第二个多项式x曲线起点二阶导(加速度一致)
    affine_equality(6 * i + 3, index_offset + 3 * num_params) =
        -1.0;  // 第一个多项式y曲线终点函数值 - 第二个多项式y曲线起点函数值
    affine_equality(6 * i + 4, index_offset + 3 * num_params + 1) =
        -1.0;  // 第一个多项式y曲线终点一阶导 -
               // 第二个多项式y曲线起点一阶导(速度一致)
    affine_equality(6 * i + 5, index_offset + 3 * num_params + 2) =
        -2.0;  // 第一个多项式y曲线终点二阶导 -
               // 第二个多项式y曲线起点二阶导(加速度一致)
  }
  return AddEqualityConstraint(affine_equality, affine_boundary);
}

// guarantee up to third order derivative are joint
bool Spline2dConstraint::AddThirdDerivativeSmoothConstraint() {
  if (t_knots_.size() < 3) {
    return false;
  }
  Eigen::MatrixXd affine_equality =
      Eigen::MatrixXd::Zero(8 * (t_knots_.size() - 2), total_param_);
  Eigen::MatrixXd affine_boundary =
      Eigen::MatrixXd::Zero(8 * (t_knots_.size() - 2), 1);

  for (uint32_t i = 0; i + 2 < t_knots_.size(); ++i) {
    const double rel_t = t_knots_[i + 1] - t_knots_[i];
    const uint32_t num_params = spline_order_ + 1;
    const uint32_t index_offset = 2 * i * num_params;
    std::vector<double> power_t = PolyCoef(rel_t);
    std::vector<double> derivative_t = DerivativeCoef(rel_t);
    std::vector<double> second_derivative_t = SecondDerivativeCoef(rel_t);
    std::vector<double> third_derivative_t = ThirdDerivativeCoef(rel_t);
    for (uint32_t j = 0; j < num_params; ++j) {
      affine_equality(8 * i, j + index_offset) = power_t[j];
      affine_equality(8 * i + 1, j + index_offset) = derivative_t[j];
      affine_equality(8 * i + 2, j + index_offset) = second_derivative_t[j];
      affine_equality(8 * i + 3, j + index_offset) = third_derivative_t[j];
      affine_equality(8 * i + 4, j + index_offset + num_params) = power_t[j];
      affine_equality(8 * i + 5, j + index_offset + num_params) =
          derivative_t[j];
      affine_equality(8 * i + 6, j + index_offset + num_params) =
          second_derivative_t[j];
      affine_equality(8 * i + 7, j + index_offset + num_params) =
          third_derivative_t[j];
    }
    affine_equality(8 * i, index_offset + 2 * num_params) = -1.0;
    affine_equality(8 * i + 1, index_offset + 2 * num_params + 1) = -1.0;
    affine_equality(8 * i + 2, index_offset + 2 * num_params + 2) = -2.0;
    affine_equality(8 * i + 3, index_offset + 2 * num_params + 3) = -6.0;
    affine_equality(8 * i + 4, index_offset + 3 * num_params) = -1.0;
    affine_equality(8 * i + 5, index_offset + 3 * num_params + 1) = -1.0;
    affine_equality(8 * i + 6, index_offset + 3 * num_params + 2) = -2.0;
    affine_equality(8 * i + 7, index_offset + 3 * num_params + 3) = -6.0;
  }
  return AddEqualityConstraint(affine_equality, affine_boundary);
}

/**
 *   @brief: output interface inequality constraint
 *           输出不等式约束
 **/
const AffineConstraint& Spline2dConstraint::inequality_constraint() const {
  return inequality_constraint_;
}

const AffineConstraint& Spline2dConstraint::equality_constraint() const {
  return equality_constraint_;
}

uint32_t Spline2dConstraint::FindIndex(const double t) const {
  // ForwardIt upper_bound( ForwardIt first, ForwardIt last, const T& value );
  // 返回指向范围 [first, last) 中首个大于 value
  // 的元素的迭代器，或若找不到这种元素则返回 last
  auto upper_bound = std::upper_bound(t_knots_.begin() + 1, t_knots_.end(), t);
  // 找到相对时间
  return std::min(static_cast<uint32_t>(t_knots_.size() - 1),
                  static_cast<uint32_t>(upper_bound - t_knots_.begin())) -
         1;
}

/**
 * 一般车辆在规划起始位置的朝向确定，所以新生成的曲线还要保证在起始点的朝向约束
 *
 * 设朝向角为a
 * tana = sina / cosa = dy / dx = f'_y(s) / f'_x(s)
   ==>  -f'_x(s) * sina + f'_y(s) * cosa = 0
 *
 */

/* result = [-sina, -t*sina, -t^2*sina, -t^3*sina, -t^4*sina, -t^5*sina,
              cosa,  t*cosa,  t^2*cosa,  t^3*cosa,  t^4*cosa,  t^5*cosa]
*/
std::vector<double> Spline2dConstraint::AffineCoef(const double angle,
                                                   const double t) const {
  const uint32_t num_params = spline_order_ + 1;
  std::vector<double> result(num_params * 2, 0.0);
  double x_coef = -common::math::sin(common::math::Angle16::from_rad(angle));
  double y_coef = common::math::cos(common::math::Angle16::from_rad(angle));
  for (uint32_t i = 0; i < num_params; ++i) {
    result[i] = x_coef;
    result[i + num_params] = y_coef;
    x_coef *= t;
    y_coef *= t;
  }
  return result;
}

// result = [0.0, -sina, -2*s*sina, -3*s^2*sina, -4*s^3*sina, -5*s^4*sina,
//           0.0,  cosa,  2*s*cosa,  3*s^2*cosa,  4*s^3*cosa,  5*s^4*cosa]
std::vector<double> Spline2dConstraint::AffineDerivativeCoef(
    const double angle, const double t) const {
  const uint32_t num_params = spline_order_ + 1;
  std::vector<double> result(num_params * 2, 0.0);
  double x_coef = -common::math::sin(common::math::Angle16::from_rad(angle));
  double y_coef = common::math::cos(common::math::Angle16::from_rad(angle));
  std::vector<double> power_t = PolyCoef(t);
  for (uint32_t i = 1; i < num_params; ++i) {
    result[i] = x_coef * power_t[i - 1] * i;
    result[i + num_params] = y_coef * power_t[i - 1] * i;
  }
  return result;
}

// result = [0.0, 0.0, -2*sina, -3*2*s*sina, -4*3*s^2*sina, -5*4*s^3*sina,
//           0.0, 0.0,  2*cosa,  3*2*s*cosa,  4*3*s^2*cosa,  5*4*s^3*cosa]
std::vector<double> Spline2dConstraint::AffineSecondDerivativeCoef(
    const double angle, const double t) const {
  const uint32_t num_params = spline_order_ + 1;
  std::vector<double> result(num_params * 2, 0.0);
  double x_coef = -common::math::sin(common::math::Angle16::from_rad(angle));
  double y_coef = common::math::cos(common::math::Angle16::from_rad(angle));
  std::vector<double> power_t = PolyCoef(t);
  for (uint32_t i = 2; i < num_params; ++i) {
    result[i] = x_coef * power_t[i - 2] * i * (i - 1);
    result[i + num_params] = y_coef * power_t[i - 2] * i * (i - 1);
  }
  return result;
}

// result = [0.0, 0.0, 0.0, -3*2*sina, -4*3*2*s*sina, -5*4*3*s^2*sina,
//           0.0, 0.0, 0.0,  3*2*cosa,  4*3*2*s*cosa,  5*4*3*s^2*cosa]
std::vector<double> Spline2dConstraint::AffineThirdDerivativeCoef(
    const double angle, const double t) const {
  const uint32_t num_params = spline_order_ + 1;
  std::vector<double> result(num_params * 2, 0.0);
  double x_coef = -common::math::sin(common::math::Angle16::from_rad(angle));
  double y_coef = common::math::cos(common::math::Angle16::from_rad(angle));
  std::vector<double> power_t = PolyCoef(t);
  for (uint32_t i = 3; i < num_params; ++i) {
    result[i] = x_coef * power_t[i - 3] * i * (i - 1) * (i - 2);
    result[i + num_params] = y_coef * power_t[i - 3] * i * (i - 1) * (i - 2);
  }
  return result;
}

double Spline2dConstraint::SignDistance(const Vec2d& xy_point,
                                        const double angle) const {
  return common::math::InnerProd(
      xy_point.x(), xy_point.y(),
      -common::math::sin(common::math::Angle16::from_rad(angle)),
      common::math::cos(common::math::Angle16::from_rad(angle)));
}

// 1 ,s_0 ,s_0^2 ,s_0^3 ,s_0^4 ,s_0^5
std::vector<double> Spline2dConstraint::PolyCoef(const double t) const {
  std::vector<double> result(spline_order_ + 1, 1.0);
  for (uint32_t i = 1; i < result.size(); ++i) {
    result[i] = result[i - 1] * t;
  }
  return result;
}

// 0 ,1 ,2s_0 ,3s_0^2 ,4s_0^3 ,5s_0^4
std::vector<double> Spline2dConstraint::DerivativeCoef(const double t) const {
  std::vector<double> result(spline_order_ + 1, 0.0);
  std::vector<double> power_t = PolyCoef(t);
  for (uint32_t i = 1; i < result.size(); ++i) {
    result[i] = power_t[i - 1] * i;
  }
  return result;
}

// 0 ,0 ,2 ,3*2s_0 ,4*3s_0^2 ,5*4s_0^3
std::vector<double> Spline2dConstraint::SecondDerivativeCoef(
    const double t) const {
  std::vector<double> result(spline_order_ + 1, 0.0);
  std::vector<double> power_t = PolyCoef(t);
  for (uint32_t i = 2; i < result.size(); ++i) {
    result[i] = power_t[i - 2] * i * (i - 1);
  }
  return result;
}

// 0, 0, 0, 3*2*1, 4*3*2s_0, 5*4*3s_0^2
std::vector<double> Spline2dConstraint::ThirdDerivativeCoef(
    const double t) const {
  std::vector<double> result(spline_order_ + 1, 0.0);
  std::vector<double> power_t = PolyCoef(t);
  for (uint32_t i = 3; i < result.size(); ++i) {
    result[i] = power_t[i - 3] * i * (i - 1) * (i - 2);
  }
  return result;
}

}  // namespace planning
}  // namespace apollo
