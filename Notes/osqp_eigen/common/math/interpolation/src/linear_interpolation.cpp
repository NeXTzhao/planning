/*
 * @Author: wangdezhao
 * @Date: 2022-04-07 21:11:19
 * @LastEditTime: 2022-04-08 17:22:27
 * @FilePath: /osqp_eigen/common/math/interpolation/src/linear_interpolation.cpp
 * @Copyright:
 */
// Copyright 2021 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// #include "interpolation/linear_interpolation.hpp"
#include "linear_interpolation.hpp"

#include <vector>

namespace interpolation {
double lerp(const double src_val, const double dst_val, const double ratio) {
  return src_val + (dst_val - src_val) * ratio;
}

std::vector<double> lerp(const std::vector<double>& base_keys,
                         const std::vector<double>& base_values,
                         const std::vector<double>& query_keys) {
  // throw exception for invalid arguments
  interpolation_utils::validateInput(base_keys, base_values, query_keys);

  // calculate linear interpolation
  std::vector<double> query_values;
  size_t key_index = 0;
  for (const auto query_key : query_keys) {
    while (base_keys.at(key_index + 1) < query_key) {
      ++key_index;
    }

    const double src_val = base_values.at(key_index);
    const double dst_val = base_values.at(key_index + 1);
    const double ratio =
        (query_key - base_keys.at(key_index)) /
        (base_keys.at(key_index + 1) - base_keys.at(key_index));

    //(Yq - Yi) / (Yi+1 -Yi) = (Xq -Xi) / (Xi+1 -Xi)
    const double interpolated_val = lerp(src_val, dst_val, ratio);
    query_values.push_back(interpolated_val);
  }

  return query_values;
}
}  // namespace interpolation
