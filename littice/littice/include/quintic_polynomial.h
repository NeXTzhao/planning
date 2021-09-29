/*
 * @Author: your name
 * @Date: 2021-09-20 16:49:20
 * @LastEditTime: 2021-09-22 14:57:27
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /littice/include/quintic_polynomial.h
 */

#ifndef _QUINTIC_POLYNOMIAL_H
#define _QUINTIC_POLYNOMIAL_H

#include <iostream>
#include <vector>
#include <array>
#include <string>
#include <cmath>
#include <Eigen/Eigen>

namespace cpprobotics
{
  /**
 * @brief 五次多项式
 * 
 */
  class QuinticPolynomial
  {
  public:
    // current parameter at t=0
    float xs;
    float vxs;
    float axs;

    // parameters at target t=t_j
    // 目标 t=t_j 处的参数
    float xe;
    float vxe;
    float axe;

    // function parameters
    float a0, a1, a2, a3, a4, a5;

    QuinticPolynomial(){};

    // polynomial parameters  多项式参数
    // 构造函数后加冒号是初始化表达式,冒号后面的内容是初始化类的数据成员 
    QuinticPolynomial(float xs_, float vxs_, float axs_, float xe_, float vxe_, float axe_, float T) : xs(xs_), vxs(vxs_), axs(axs_), xe(xe_), vxe(vxe_), axe(axe_), a0(xs_), a1(vxs_), a2(axs_ / 2.0)
    {
      Eigen::Matrix3f A;
      // std::pow(x, y)计算x的y次幂
      A << std::pow(T, 3), std::pow(T, 4), std::pow(T, 5),
          3 * std::pow(T, 2), 4 * std::pow(T, 3), 5 * std::pow(T, 4),
          6 * T, 12 * std::pow(T, 2), 20 * std::pow(T, 3);
      Eigen::Vector3f B;
      B << xe - a0 - a1 * T - a2 * std::pow(T, 2),
          vxe - a1 - 2 * a2 * T,
          axe - 2 * a2;

      Eigen::Vector3f c_eigen = A.colPivHouseholderQr().solve(B);
      a3 = c_eigen[0];
      a4 = c_eigen[1];
      a5 = c_eigen[2];
    };
    // x(t)
    float calc_point(float t)
    {
      return a0 + a1 * t + a2 * std::pow(t, 2) + a3 * std::pow(t, 3) + a4 * std::pow(t, 4) + a5 * std::pow(t, 5);
    };
    // v(t)
    float calc_first_derivative(float t)
    {
      return a1 + 2 * a2 * t + 3 * a3 * std::pow(t, 2) + 4 * a4 * std::pow(t, 3) + a5 * std::pow(t, 4);
    };
    // a(t)
    float calc_second_derivative(float t)
    {
      return 2 * a2 + 6 * a3 * t + 12 * a4 * std::pow(t, 2) + 20 * a5 * std::pow(t, 3);
    };
    // da(t)
    float calc_third_derivative(float t)
    {
      return 6 * a3 + 24 * a4 * t + 60 * a5 * std::pow(t, 2);
    };
  };
}
#endif
