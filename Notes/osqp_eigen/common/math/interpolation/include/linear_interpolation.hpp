/*
 * @Author: wangdezhao
 * @Date: 2022-04-07 21:11:19
 * @LastEditTime: 2022-04-08 17:41:35
 * @FilePath: /osqp_eigen/common/math/interpolation/include/interpolation/linear_interpolation.hpp
 * /osqp_eigen/common/math/interpolation/include/interpolation/linear_interpolation.hpp
 * @Copyright:
 */

#pragma once
#include <vector>

// #include "interpolation/include/interpolation_utils.hpp"
#include "interpolation_utils.hpp"

namespace interpolation {
double lerp(const double src_val, const double dst_val, const double ratio);

std::vector<double> lerp(const std::vector<double>& base_keys,
                         const std::vector<double>& base_values,
                         const std::vector<double>& query_keys);
}
