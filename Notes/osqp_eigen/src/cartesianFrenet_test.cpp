/**
 * @file cartesianFrenet.cpp
 * @brief
 * @author Wang Dezhao (1282507109@qq.com)
 * @version 1.0
 * @date 2022-05-14 17:37:37
 *
 * @copyright Copyright (c) 2022
 */

#include "cartesianFrenet.hpp"

#include "angle.h"
#include "matplotlibcpp.h"
#include "quinticPolynomial.hpp"

using namespace apollo;
using namespace common;
using namespace math;
using namespace cpprobotics;
namespace plt = matplotlibcpp;

int main() {
  PointState x_start{0.0, 0.0, 0.0};
  PointState x_end{200.0, 0.0, 0.0};
  PointState y_start{0.0, 0.0, 0.0};
  PointState y_end{30.0, 0.0, 0.0};

  double TotalTimes = x_end.xy - x_start.xy;
  FrenetPath refrencelineInfo;
  QuinticPolynomial refrenceline(x_start, x_end, y_start, y_end, TotalTimes);
  refrenceline.getPloyPath(refrencelineInfo, 1.0);

  PointState carx_start{0.0, 0.0, 0.0};
  PointState carx_end{80.0, 0.0, 0.0};
  PointState cary_start{0.0, 0.0, 0.0};
  PointState cary_end{-10.0, 0.0, 0.0};

  // double carTotalTimes = carx_end.xy - carx_start.xy;
  // FrenetPath car_tracj;
  // QuinticPolynomial car_quinticPloy(carx_start, carx_end, cary_start,
  // cary_end,
  //                                   carTotalTimes);
  // car_quinticPloy.getPloyPath(car_tracj, 1.0);
  /*****************************************************************************************************/

  std::vector<double> x_result;
  std::vector<double> y_result;

  size_t loop = 11;
  double delta_y = std::abs(cary_end.xy) * 2 / loop;
  for (size_t i = 0; i < loop; i += 1.0) {
    double times = carx_end.xy;
    FrenetPath car_tracj;
    cary_end.xy += delta_y;
    QuinticPolynomial car_quinticPloy(carx_start, carx_end, cary_start,
                                      cary_end, times);
    car_quinticPloy.getPloyPath(car_tracj, 1);
    int pre_index = 0;
    for (size_t i = 0; i < car_tracj.t.size(); ++i) {
      auto time1 = std::chrono::system_clock::now();
      int cur_index = 0;
      refrenceline.matchPoint(refrencelineInfo, car_tracj.x.at(i),
                              car_tracj.y.at(i), pre_index, cur_index);
      if (cur_index >= car_tracj.t.size()) {
        cur_index = car_tracj.t.size() - 1;
      }
      pre_index = cur_index;
      auto time2 = std::chrono::system_clock::now();
      std::chrono::duration<double> diff = time2 - time1;
      std::cout << "cur_index:" << cur_index << '\n';
      std::cout << "Time for matching Point = " << diff.count() * 1000
                << " msec.\n";

      const auto angle = refrencelineInfo.theta.at(cur_index);
      // std::cout << "angle:" << angle << '\n';

      double x_out = refrencelineInfo.x.at(cur_index) -
                     std::sin(angle) * car_tracj.y.at(cur_index);

      double y_out = refrencelineInfo.y.at(cur_index) +
                     std::cos(angle) * car_tracj.y.at(cur_index);
      // std::cout << "cur_index:" << cur_index << "，x_out:" << x_out
      //           << ",y_out:" << y_out << '\n';
      x_result.emplace_back(x_out);
      y_result.emplace_back(y_out);
      // std::cout << "x_result:" << x_out << "，y_result:" << y_out << '\n';
    }
    // plt::named_plot("slove_xy", x_result, y_result);
    x_result.clear();
    y_result.clear();
  }

  /*****************************************************************************************************/
  // plt::named_plot("ref_line_XY", refrencelineInfo.x, refrencelineInfo.y);
  // plt::legend();
  // plt::axis("equal");
  // plt::legend();
  // plt::axis("equal");
  // plt::legend();
  // plt::show();
  return 0;
}

// std::cout << "cur_index:" << cur_index << '\n';
// double ptr_s = 0;
// double ptr_d = 0;
// CartesianFrenetConverter::cartesian_to_frenet(
//     refrencelineInfo.s.at(cur_index), refrencelineInfo.x.at(cur_index),
//     refrencelineInfo.y.at(cur_index),
//     refrencelineInfo.theta.at(cur_index), car_tracj.x.at(i),
//     car_tracj.y.at(i), &ptr_s, &ptr_d);

// double x_out;
// double y_out;
// double theta_out;
// double kappa_out;
// double v_out;
// double a_out;
// std::array<double, 3> s_conditions{car_tracj.x.at(cur_index),
//                                    car_tracj.x_d.at(cur_index),
//                                    car_tracj.x_dd.at(cur_index)};
// std::array<double, 3> d_conditions{car_tracj.y.at(cur_index),
//                                    car_tracj.y_d.at(cur_index),
//                                    car_tracj.y_dd.at(cur_index)};

// CartesianFrenetConverter::frenet_to_cartesian(
//     refrencelineInfo.s.at(cur_index), refrencelineInfo.x.at(cur_index),
//     refrencelineInfo.y.at(cur_index),
//     refrencelineInfo.theta.at(cur_index),
//     refrencelineInfo.kappa.at(cur_index),
//     refrencelineInfo.dkappa.at(cur_index), s_conditions, d_conditions,
//     &x_out, &y_out, &theta_out, &kappa_out, &v_out, &a_out);