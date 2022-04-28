/*
 * @Author: wangdezhao
 * @Date: 2022-04-14 01:10:20
 * @LastEditTime: 2022-04-16 16:13:05
 * @FilePath: /osqp_eigen/src/pathDemo.cpp
 * @Copyright:
 */
#include "PathOptimize.hpp"

void spinle(std::vector<double> &value) {
  const std::vector<double> base_keys{-1.5, 1.0,  5.0,  8.5,  10.0, 13.6, 15.0,
                                      18.2, 20.0, 22.3, 24.5, 28.3, 30.9};
  const std::vector<double> base_values{-1.2, 0.5, 1.0, 1.2, 1.6, 1.0, 2.0,
                                        1.3,  1.5, 1.1, 0.8, 1.2, 2.6};
  const std::vector<double> query_keys{
      -0.9, -0.5, 0.0,  0.5,  1.2,  2.3,  4.6,  5.8,  6.2,  7.9,  8.3,
      10.6, 12.1, 16.5, 18.0, 19.3, 21.8, 23.0, 23.5, 26.7, 29.2, 29.6};
  // const std::vector<double> query_values;
  const auto query_values =
      interpolation::slerp(base_keys, base_values, query_keys);

  std::vector<double> new_keys;
  merge(base_keys.begin(), base_keys.end(), query_keys.begin(),
        query_keys.end(), back_inserter(new_keys));
  std::vector<double> new_values;
  merge(base_values.begin(), base_values.end(), query_values.begin(),
        query_values.end(), back_inserter(new_values));

  value = query_keys;
  // printVector(value);
}

int main() {
  double s_len = 50;
  double delta_s_ = 0.5;
  size_t numPoints = s_len / delta_s_;
  std::array<double, 4> xWeight = {l_weight, dl_weight, ddl_weight,
                                   dddl_weight};

  PathOptimize qp(100);
  std::vector<double> new_values_;
  spinle(new_values_);

  qp.set_bound(numPoints, xWeight, new_values_);
  qp.Optimize();

  // auto temp = qp.x_bounds_;
  // std::cout << "x_bound_" << '\n';
  // for (const auto &item : temp) {
  //   std::cout << item.first <<","<<item.second<< '\n';
  // }
  qp.matplot();

  return 0;
}