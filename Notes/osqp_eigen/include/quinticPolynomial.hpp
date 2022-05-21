/**
 * @file quinticPolynomial.hpp
 * @brief
 * @author Wang Dezhao (1282507109@qq.com)
 * @version 1.0
 * @date 2022-05-02 10:25:03
 *
 * @copyright Copyright (c) 2022
 */

#pragma once

#include <Eigen/Eigen>
#include <array>
#include <chrono>
#include <cmath>
#include <ctime>
#include <iostream>
#include <string>
#include <vector>

#include "frenet_path_double.h"

struct PointState {
  double xy;  //位置
  double v;   //速度
  double a;   //加速度
};

namespace cpprobotics {
/**
 *@brief 五次多项式
 *
 */
class QuinticPolynomial {
 public:
  QuinticPolynomial(PointState x_start, PointState x_end, PointState y_start,
                    PointState y_end, double TotalTimes);

  /**
   * @brief 获得轨迹信息
   * @param  fp
   * @param  DT
   */
  void getPloyPath(FrenetPath& fp, const double DT);

  /**
   * @brief  寻找匹配点即距离最短的点
   * @param  fp
   * @param  current_post_x
   * @param  current_post_y
   * @param  index
   * @return void
   */
  void matchPoint(const FrenetPath& fp, const double current_post_x,
                  const double current_post_y, int pre_index, int& index);

 private:
  /**
   * @brief 求解纵向轨迹多项式系数
   */
  void cal_longiPoly_coff();

  /**
   * @brief 求解横向向轨迹多项式系数
   */
  void cal_latPoly_coff();

  /**
   * @brief x(t)  y(x)_t
   */
  double calc_point_x(double t) {
    return a0 + a1 * t + a2 * std::pow(t, 2) + a3 * std::pow(t, 3) +
           a4 * std::pow(t, 4) + a5 * std::pow(t, 5);
  };

  double calc_point_y_x(double x) {
    return b0 + b1 * x + b2 * std::pow(x, 2) + b3 * std::pow(x, 3) +
           b4 * std::pow(x, 4) + b5 * std::pow(x, 5);
  };

  /**
   * @brief  x(t)_d   y(x)_d  y(t)_d
   */
  double calc_point_xd(double t) {
    return a1 + 2 * a2 * t + 3 * a3 * std::pow(t, 2) + 4 * a4 * std::pow(t, 3) +
           5 * a5 * std::pow(t, 4);
  };

  double calc_point_y_x_d(double x) {
    return b1 + 2 * b2 * x + 3 * b3 * std::pow(x, 2) + 4 * a4 * std::pow(x, 3) +
           5 * b5 * std::pow(x, 4);
  };

  double calc_point_y_t_d(double y_x_d, double x_d) { return y_x_d * x_d; };

  /**
   * @brief  x(t)_dd  y(x)_dd  y(t)_dd
   */
  double calc_point_xdd(double t) {
    return 2 * a2 + 6 * a3 * t + 12 * a4 * std::pow(t, 2) +
           20 * a5 * std::pow(t, 3);
  };

  double calc_point_y_x_dd(double x) {
    return 2 * b2 + 6 * b3 * x + 12 * b4 * std::pow(x, 2) +
           20 * b5 * std::pow(x, 3);
  };

  double calc_point_y_t_dd(double y_x_dd, double xd, double y_x_d, double xdd) {
    return y_x_dd * std::pow(xd, 2) + y_x_d * xdd;
  };

  /**
   * @brief x(t)_ddd  y(x)_ddd  y(t)_ddd
   */

  double calc_point_xddd(double t) {
    return 6 * a3 + 12 * 2 * a4 * t + 20 * 3 * a5 * std::pow(t, 2);
  };

  double calc_point_y_x_ddd(double x) {
    return 6 * b3 + 12 * 2 * b4 * x + 20 * 3 * b5 * std::pow(x, 2);
  };

  // y(t)_ddd = y'''(x) * xd^3 + y''(x) * 2* xd* xdd + y''(x) * xdd * xd +y'(x)
  // * xdd
  double calc_point_y_t_ddd(double y_x_ddd, double y_x_dd, double y_x_d,
                            double xddd, double xdd, double xd) {
    return y_x_ddd * std::pow(xd, 3) + y_x_dd * 2 * xd * xdd +
           y_x_dd * xdd * xd + y_x_d * xddd;
  };

  /**
   * @brief 计算方向角theta
   *      atan2结果以弧度表示并介于 -pi 到 pi 之间（不包括 -pi)
   * @param  y_x_t_d
   * @param  x_d
   * @return double
   */
  double calc_point_theta(double y_x_t, double x) {
    return atan2(y_x_t, x);
    // return atan(y_x_t_d / x_d);
  };

  /**
   * @brief  计算k 曲率
   * @param  y_x_dd
   * @param  y_x_d
   * @return double
   */
  double calc_point_k(double y_x_dd, double y_x_d) {
    return y_x_dd / std::pow((1 + std::pow(y_x_d, 2)), 1.5);
  }

  /**
   * @brief 计算曲率 k 的导数
   * Compute the curvature change rate w.r.t. curve length (dkappa) given
   * curve X = (x(t), y(t)) which t is an arbitrary parameter.
   * @param dx dx / dt
   * @param d2x d(dx) / dt
   * @param dy dy / dt
   * @param d2y d(dy) / dt
   * @param d3x d(d2x) / dt
   * @param d3y d(d2y) / dt
   * @return the curvature change rate
   */
  double cal_point_k_derivative(const double xd, const double xdd,
                                const double xddd, const double y_x_t_d,
                                const double y_x_t_dd, const double y_x_t_ddd) {
    // const double a = dx * d2y - dy * d2x;
    const double a = xd * y_x_t_dd - y_x_t_d * xdd;
    // const double b = dx * d3y - dy * d3x;
    const double b = xd * y_x_t_ddd - y_x_t_d * xddd;
    // const double c = dx * d2x + dy * d2y;
    const double c = xd * xdd + y_x_t_d * y_x_t_dd;
    // const double d = dx * dx + dy * dy;
    const double d = xd * xd + y_x_t_d * y_x_t_d;

    return (b * d - 3.0 * a * c) / (d * d * d);
  }

  /**
   * @brief  计算弧长s
   *        此函数的目的是增加固定长度的弧长l,来进行离散化，或者获得轨迹的累计弧长
   * @param  delta_y_t
   * @param  delta_x_t
   * @return double
   */
  double calc_ploynomial_s(double delta_y_t, double delta_x_t) {
    if (delta_x_t == 0) return 0.0;
    double tan_theta = delta_y_t / delta_x_t;

    return delta_x_t * std::pow((1 + std::pow(tan_theta, 2)), 1.5);
  }

 private:
  /*起始点和终点的状态*/
  PointState x_start_;
  PointState x_end_;
  PointState y_start_;
  PointState y_end_;

  //行驶完轨迹所需要的总时间
  double TotalTimes_;
  // 多项式系数coff
  double a0, a1, a2, b0, b1, b2;
  double a3, a4, a5, b3, b4, b5;
};
};  // namespace cpprobotics
