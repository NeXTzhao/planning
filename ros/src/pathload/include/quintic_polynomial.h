#ifndef _QUINTIC_POLYNOMIAL_H
#define _QUINTIC_POLYNOMIAL_H

#include <Eigen/Eigen>
#include <array>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

namespace cpprobotics {
/**
*@brief 五次多项式
*
*/
class QuinticPolynomial {
 public:
  // current parameter at t=0
  float xs;
  float vxs;
  float axs;

  float ys;
  float vys;
  float ays;

  // parameters at target t=t_j
  float xe;
  float vxe;
  float axe;

  float xe_temp;
  float ye;
  float vye;
  float aye;

  // function parameters
  float a0, a1, a2, a3, a4, a5;
  float b0, b1, b2, b3, b4, b5;

  QuinticPolynomial(){};

  // 纵向
  QuinticPolynomial(float xs_, float vxs_, float axs_, float xe_, float vxe_,
                    float axe_, float T)
      : xs(xs_),
        vxs(vxs_),
        axs(axs_),
        xe(xe_),
        vxe(vxe_),
        axe(axe_),
        a0(xs_),
        a1(vxs_),
        a2(axs_ / 2.0) {
    // 起始和终点的矩阵方程表达式
    Eigen::Matrix3f A;
    // std::pow(x, y)计算x的y次幂
    A << std::pow(T, 3), std::pow(T, 4), std::pow(T, 5), 3 * std::pow(T, 2),
        4 * std::pow(T, 3), 5 * std::pow(T, 4), 6 * T, 12 * std::pow(T, 2),
        20 * std::pow(T, 3);
    Eigen::Vector3f B;
    B << xe - a0 - a1 * T - a2 * std::pow(T, 2), vxe - a1 - 2 * a2 * T,
        axe - 2 * a2;
    // Eigen矩阵colPivHouseholderQr().solve()求解Ax=b
    Eigen::Vector3f c_eigen = A.colPivHouseholderQr().solve(B);
    a3 = c_eigen[0];
    a4 = c_eigen[1];
    a5 = c_eigen[2];
  };

  // 横向
  // 求y(t)首先会得到y(x),在通过求偏导得到y(t)_d,y(t)_dd
  QuinticPolynomial(float ys_, float vys_, float ays_, float ye_, float vye_,
                    float aye_, float T, float xe_)
      : ys(ys_),
        vys(vys_),
        ays(ays_),
        ye(ye_),
        vye(vye_),
        aye(aye_),
        b0(ys_),
        b1(vys_),
        b2(ays_ / 2.0),
        xe_temp(xe_) {
    // 起始和终点的矩阵方程表达式
    Eigen::Matrix3f C;
    // std::pow(x, y)计算x的y次幂
    C << std::pow(xe_temp, 3), std::pow(xe_temp, 4), std::pow(xe_temp, 5),
        3 * std::pow(xe_temp, 2), 4 * std::pow(xe_temp, 3),
        5 * std::pow(xe_temp, 4), 6 * xe_temp, 12 * std::pow(xe_temp, 2),
        20 * std::pow(xe_temp, 3);
    Eigen::Vector3f D;
    D << ye - b0 - b1 * xe_temp - b2 * std::pow(xe_temp, 2),
        vye - b1 - 2 * b2 * xe_temp, aye - 2 * b2;
    // Eigen矩阵colPivHouseholderQr().solve()求解Ax=b
    Eigen::Vector3f b_eigen = C.colPivHouseholderQr().solve(D);

    b3 = b_eigen[0];
    b4 = b_eigen[1];
    b5 = b_eigen[2];
  }

  /*******************************************************************************/

  // x(t)
  float calc_point_x(float t) {
    return a0 + a1 * t + a2 * std::pow(t, 2) + a3 * std::pow(t, 3) +
           a4 * std::pow(t, 4) + a5 * std::pow(t, 5);
  };
  // y(x)_t
  float calc_point_y_x(float x) {
    return b0 + b1 * x + b2 * std::pow(x, 2) + b3 * std::pow(x, 3) +
           b4 * std::pow(x, 4) + b5 * std::pow(x, 5);
  };

  /*******************************************************************************/

  // x(t)_d
  float calc_point_xd(float t) {
    return a1 + 2 * a2 * t + 3 * a3 * std::pow(t, 2) + 4 * a4 * std::pow(t, 3) +
           5 * a5 * std::pow(t, 4);
  };
  // y(x)_d
  float calc_point_y_x_d(float x) {
    return b1 + 2 * b2 * x + 3 * b3 * std::pow(x, 2) + 4 * a4 * std::pow(x, 3) +
           5 * b5 * std::pow(x, 4);
  };
  // y(t)_d
  float calc_point_y_t_d(float y_x_d, float x_d) { return y_x_d * x_d; };

  /*******************************************************************************/

  // x(t)_dd
  float calc_point_xdd(float t) {
    return 2 * a2 + 6 * a3 * t + 12 * a4 * std::pow(t, 2) +
           20 * a5 * std::pow(t, 3);
  };
  // y(x)_dd
  float calc_point_y_x_dd(float x) {
    return 2 * b2 + 6 * b3 * x + 12 * b4 * std::pow(x, 2) +
           20 * b5 * std::pow(x, 3);
  };
  // y(t)_dd
  float calc_point_y_t_dd(float y_x_dd, float xd, float y_x_d, float xdd) {
    return y_x_dd * std::pow(xd, 2) + y_x_d * xdd;
  };

  /*******************************************************************************/

  // thetar
  // float calc_point_thetar(float y_x_t_d) { return atan(y_x_t_d); };
  float calc_point_thetar(float y_x_t_d, float x_d) {
    return atan2(y_x_t_d, x_d);
  };

  // k 曲率
  float calc_point_k(float y_x_dd, float y_x_d) {
    return y_x_dd / std::pow((1 + std::pow(y_x_d, 2)), 1.5);
  }
  // float calc_point_k(float y_x_t_dd, float y_x_t_d, float x_dd, float x_d) {
  //   return (x_d * y_x_t_dd - y_x_t_d * x_dd) /
  //          std::pow((std::pow(x_d, 2) + std::pow(y_x_t_d, 2)), 1.5);
  // }
};
};
#endif
