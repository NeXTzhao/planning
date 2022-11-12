/**
 * @file cubic_spline.h
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2021-09-20
 *
 * @copyright Copyright (c) 2021
 *
 */
#ifndef _CUBIC_SPLINE_H
#define _CUBIC_SPLINE_H

#include <Eigen/Eigen>
#include <array>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "cpprobotics_types.h"

namespace cpprobotics {

Vec_f vec_diff(Vec_f input) {
  Vec_f output;
  for (unsigned int i = 1; i < input.size(); i++) {
    output.push_back(input[i] - input[i - 1]);
  }
  return output;
};

Vec_f cum_sum(Vec_f input) {
  Vec_f output;
  float temp = 0;
  for (unsigned int i = 0; i < input.size(); i++) {
    temp += input[i];
    output.push_back(temp);
  }
  return output;
};

/**
* @brief 三次样条插值 根据插值原理进行插值
*
*       三次样条的原理：
          三次样条的原理和二次样条的原理相同，我们用函数aX^3+bX^2+cX+d这个函数来进行操作，
        这里一共是4个点，分为3个区间，每个区间一个三次样条函数的话，一共是12个方程，只要我们找出这12个方程，这个问题就解决了。

        要求：
          1>内部节点处的函数值应该相等，这里一共是4个方程。

          2>函数的第一个端点和最后一个端点，应该分别在第一个方程和最后一个方程中。这里是2个方程。

          3>两个函数在节点处的一阶导数应该相等。这里是两个方程。

          4>两个函数在节点处的二阶导数应该相等，这里是两个方程。

          5>端点处的二阶导数为零，这里是两个方程。
              a1=0

              b1=0
*/
class Spline {
 public:
  Vec_f x;
  Vec_f y;
  int nx;
  Vec_f h;
  Vec_f a;
  Vec_f b;
  Vec_f c;
  // Eigen::VectorXf c;
  Vec_f d;

  Spline(){};
  // d_i * (x-x_i)^3 + c_i * (x-x_i)^2 + b_i * (x-x_i) + a_i
  Spline(Vec_f x_, Vec_f y_)
      : x(x_), y(y_), nx(x_.size()), h(vec_diff(x_)), a(y_) {
    // Matrix表示矩阵，Vector表示向量，数字表示维度，最后的f和i分别表示单精度和整型数据类型
    // A是一个动态大小的矩阵，目前的大小是0*0，它的元素数组完全没有分配
    // B是动态向量
    Eigen::MatrixXf A = calc_A();
    Eigen::VectorXf B = calc_B();
    // colPivHouseholderQr().solve()函数，求解的就是Ax=b中的x
    Eigen::VectorXf c_eigen = A.colPivHouseholderQr().solve(B);
    // 存储顺序默认使用列存储，可以使用data()函数获得存储数据的首地址
    float *c_pointer = c_eigen.data();
    // Map类用于通过C++中普通的连续指针或者数组 （raw C/C++
    // arrays）来构造Eigen里的Matrix类，
    // 这就好比Eigen里的Matrix类的数据和raw C++array
    // 共享了一片地址，也就是引用。
    // 比如有个API只接受普通的C++数组，但又要对普通数组进行线性代数操作，那么用它构造为Map类，直接操作Map就等于操作了原始普通数组，省时省力。

    // STL中不同容器之间是不能直接赋值的，assign（）可以实现不同容器但相容的类型赋值
    c.assign(c_pointer, c_pointer + c_eigen.rows());

    for (int i = 0; i < nx - 1; i++) {
      d.push_back((c[i + 1] - c[i]) / (3.0 * h[i]));
      b.push_back((a[i + 1] - a[i]) / h[i] -
                  h[i] * (c[i + 1] + 2 * c[i]) / 3.0);
    }
  };

  float calc(float t) {
    if (t < x.front() || t > x.back()) {
      throw std::invalid_argument(
          "received value out of the pre-defined range");
    }
    int seg_id = bisect(t, 0, nx);
    float dx = t - x[seg_id];
    return a[seg_id] + b[seg_id] * dx + c[seg_id] * dx * dx +
           d[seg_id] * dx * dx * dx;
  };

  float calc_d(float t) {
    if (t < x.front() || t > x.back()) {
      throw std::invalid_argument(
          "received value out of the pre-defined range");
    }
    int seg_id = bisect(t, 0, nx - 1);
    float dx = t - x[seg_id];
    return b[seg_id] + 2 * c[seg_id] * dx + 3 * d[seg_id] * dx * dx;
  }

  float calc_dd(float t) {
    if (t < x.front() || t > x.back()) {
      throw std::invalid_argument(
          "received value out of the pre-defined range");
    }
    int seg_id = bisect(t, 0, nx);
    float dx = t - x[seg_id];
    return 2 * c[seg_id] + 6 * d[seg_id] * dx;
  }

 private:
  Eigen::MatrixXf calc_A() {
    Eigen::MatrixXf A = Eigen::MatrixXf::Zero(nx, nx);
    A(0, 0) = 1;
    for (int i = 0; i < nx - 1; i++) {
      if (i != nx - 2) {
        A(i + 1, i + 1) = 2 * (h[i] + h[i + 1]);
      }
      A(i + 1, i) = h[i];
      A(i, i + 1) = h[i];
    }
    A(0, 1) = 0.0;
    A(nx - 1, nx - 2) = 0.0;
    A(nx - 1, nx - 1) = 1.0;
    return A;
  };

  Eigen::VectorXf calc_B() {
    Eigen::VectorXf B = Eigen::VectorXf::Zero(nx);
    for (int i = 0; i < nx - 2; i++) {
      B(i + 1) = 3.0 * (a[i + 2] - a[i + 1]) / h[i + 1] -
                 3.0 * (a[i + 1] - a[i]) / h[i];
    }
    return B;
  };

  int bisect(float t, int start, int end) {
    int mid = (start + end) / 2;
    if (t == x[mid] || end - start <= 1) {
      return mid;
    } else if (t > x[mid]) {
      return bisect(t, mid, end);
    } else {
      return bisect(t, start, mid);
    }
  }
};

class Spline2D {
 public:
  Spline sx;
  Spline sy;
  // s为弧长
  Vec_f s;

  //
  Spline2D(Vec_f x, Vec_f y) {
    s = calc_s(x, y);
    sx = Spline(s, x);
    sy = Spline(s, y);
  };

  Poi_f calc_postion(float s_t) {
    float x = sx.calc(s_t);
    float y = sy.calc(s_t);
    return {{x, y}};
  };

  float calc_curvature(float s_t) {
    float dx = sx.calc_d(s_t);
    float ddx = sx.calc_dd(s_t);
    float dy = sy.calc_d(s_t);
    float ddy = sy.calc_dd(s_t);
    return (ddy * dx - ddx * dy) /
           ((dx * dx + dy * dy) * std::sqrt(dx * dx + dy * dy));
  };

  float calc_yaw(float s_t) {
    float dx = sx.calc_d(s_t);
    float dy = sy.calc_d(s_t);
    return std::atan2(dy, dx);
  };

 private:
  // 计算s值
  Vec_f calc_s(Vec_f x, Vec_f y) {
    Vec_f ds;
    Vec_f out_s{0};
    // 调用两两数值的差值
    Vec_f dx = vec_diff(x);
    Vec_f dy = vec_diff(y);

    for (unsigned int i = 0; i < dx.size(); i++) {
      // 求两两数之间的距离
      ds.push_back(std::sqrt(dx[i] * dx[i] + dy[i] * dy[i]));
    }
    // 调用数组累加
    Vec_f cum_ds = cum_sum(ds);
    // 在out_s头部插入cum_ds.begin()个cum_ds.end()
    out_s.insert(out_s.end(), cum_ds.begin(), cum_ds.end());
    return out_s;
  };
};
}  // namespace cpprobotics
#endif