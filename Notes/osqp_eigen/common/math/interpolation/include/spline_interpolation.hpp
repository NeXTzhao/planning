/*
 * @Author: wangdezhao
 * @Date: 2022-04-07 21:11:19
 * @LastEditTime: 2022-04-08 17:41:13
 * @FilePath: /osqp_eigen/common/math/interpolation/include/interpolation/spline_interpolation.hpp
 * @Copyright:  
 */
#pragma once

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>
#include <vector>

namespace interpolation {
// NOTE: X(s) = a_i (s - s_i)^3 + b_i (s - s_i)^2 + c_i (s - s_i) + d_i : (i =
// 0, 1, ... N-1)
struct MultiSplineCoef {
  explicit MultiSplineCoef(const size_t num_spline) {
    a.resize(num_spline);
    b.resize(num_spline);
    c.resize(num_spline);
    d.resize(num_spline);
  }

  std::vector<double> a;
  std::vector<double> b;
  std::vector<double> c;
  std::vector<double> d;
};

std::vector<double> slerp(const std::vector<double>& base_keys,
                          const std::vector<double>& base_values,
                          const std::vector<double>& query_keys);
}
