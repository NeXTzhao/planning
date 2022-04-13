/*
 * @Author: wangdezhao
 * @Date: 2022-04-08 17:45:37
 * @LastEditTime: 2022-04-08 20:08:05
 * @FilePath: /osqp_eigen/src/splineInterpoltion.cpp
 * @Copyright:
 */
#include <cmath>
#include <iostream>
#include <utility>
#include <vector>

#include "matplotlibcpp.h"
#include "spline_interpolation.hpp"

namespace plt = matplotlibcpp;

int main() {
  const std::vector<double> base_keys{-1.5, 1.0,  5.0,  8.5,  10.0, 13.6, 15.0,
                                      18.2, 20.0, 22.3, 24.5, 28.3, 30.9};
  const std::vector<double> base_values{-1.2, 0.5, 1.0, 1.2, 1.6, 1.0, 2.0,
                                        1.3,  1.5, 1.1, 0.8, 1.2, 2.6};
  const std::vector<double> query_keys{0.0,  3.0,  6.2,  12.1, 16.5, 18.0, 19.3,
                                       21.8, 23.0, 23.5, 26.7, 29.2, 29.6};
  // const std::vector<double> query_values;
  const auto query_values =
      interpolation::slerp(base_keys, base_values, query_keys);
  plt::clf();

  // plt::xlim(0,12);
  plt::xlabel("x");
  // plt::ylim(0,10);
  plt::ylabel("y");
  plt::title("spline interpolation");  //图片标题

  //   plt::named_plot("query_value", query_keys, query_values,
  //                   "ro-");  //（取名，参数，参数，直线连接）
  plt::named_plot("query_value", query_keys, query_values,
                  "r*");  //（取名，参数，参数，直线连接）
  plt::named_plot("x_y_value", base_keys, base_values,
                  "b*");  //(取名，参数，参数，离散点)

  plt::legend();
  plt::pause(0.1);

  const char* filename = "../result_picture/splineInterpolation.png";
  std::cout << "Saving result to " << filename << std::endl;
  plt::save(filename);
  // }
}