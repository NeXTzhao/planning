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

int main() {
  PointState x_start{0.0, 0.0, 0.0};
  PointState x_end{150.0, 0.0, 0.0};
  PointState y_start{0.0, 0.0, 0.0};
  PointState y_end{0.0, 0.0, 0.0};

  PointState x1_start{0.0, 0.0, 0.0};
  PointState x1_end{150.0, 0.0, 0.0};
  PointState y1_start{1.875, 0.0, 0.0};
  PointState y1_end{1.875, 0.0, 0.0};

  PointState x2_start{0.0, 0.0, 0.0};
  PointState x2_end{150.0, 0.0, 0.0};
  PointState y2_start{3.75, 0.0, 0.0};
  PointState y2_end{3.75, 0.0, 0.0};

  PointState x3_start{0.0, 0.0, 0.0};
  PointState x3_end{150.0, 0.0, 0.0};
  PointState y3_start{5.625, 0.0, 0.0};
  PointState y3_end{5.625, 0.0, 0.0};

  PointState x4_start{0.0, 0.0, 0.0};
  PointState x4_end{150.0, 0.0, 0.0};
  PointState y4_start{7.5, 0.0, 0.0};
  PointState y4_end{7.5, 0.0, 0.0};

  double TotalTimes = x_end.xy - x_start.xy;
  FrenetPath refrencelineInfo;
  QuinticPolynomial refrenceline(x_start, x_end, y_start, y_end, TotalTimes);
  refrenceline.getPloyPath(refrencelineInfo, 1.0);

  FrenetPath refrencelineInfo1;
  QuinticPolynomial refrenceline1(x1_start, x1_end, y1_start, y1_end,
                                  TotalTimes);
  refrenceline1.getPloyPath(refrencelineInfo1, 1.0);

  FrenetPath refrencelineInfo2;
  QuinticPolynomial refrenceline2(x2_start, x2_end, y2_start, y2_end,
                                  TotalTimes);
  refrenceline2.getPloyPath(refrencelineInfo2, 1.0);

  FrenetPath refrencelineInfo3;
  QuinticPolynomial refrenceline3(x3_start, x3_end, y3_start, y3_end,
                                  TotalTimes);
  refrenceline3.getPloyPath(refrencelineInfo3, 1.0);

  FrenetPath refrencelineInfo4;
  QuinticPolynomial refrenceline4(x4_start, x4_end, y4_start, y4_end,
                                  TotalTimes);
  refrenceline4.getPloyPath(refrencelineInfo4, 1.0);

  PointState carx_start{0.0, 0.0, 0.0};
  PointState carx_end{80.0, 0.0, 0.0};
  PointState cary_start{1.875, 0.0, 0.0};
  PointState cary_end{5.625, 0.0, 0.0};

  double carTotalTimes = carx_end.xy - carx_start.xy;
  FrenetPath car_tracj;
  QuinticPolynomial car_quinticPloy(carx_start, carx_end, cary_start, cary_end,
                                    carTotalTimes);
  auto time2 = std::chrono::system_clock::now();
  car_quinticPloy.getPloyPath(car_tracj, 1.0);

  auto time3 = std::chrono::system_clock::now();
  std::chrono::duration<double> diff2 = time3 - time2;
  std::cout << "Time for making kdtree = " << diff2.count() * 1000
            << " msec.\n";
  /*****************************************************************************************************/
  /*
    std::vector<double> x_result;
    std::vector<double> y_result;

    size_t loop = 1;
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
        if (cur_index >= static_cast<int>(car_tracj.t.size()) - 1) {
          cur_index = car_tracj.t.size() - 1;
        }
        pre_index = cur_index;
        std::cout << "cur_index:" << cur_index << '\n';
        auto time2 = std::chrono::system_clock::now();
        std::chrono::duration<double> diff = time2 - time1;
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
      plt::named_plot("slove_xy", x_result, y_result);
      x_result.clear();
      y_result.clear();
    }
  */
  /*****************************************************************************************************/
  plt::named_plot("ref_line_XY", refrencelineInfo.x, refrencelineInfo.y, "g");
  // plt::named_plot("ref_line1_XY", refrencelineInfo1.x, refrencelineInfo1.y,
  //                 "b--");
  plt::named_plot("ref_line2_XY", refrencelineInfo2.x, refrencelineInfo2.y,
                  "g--");
  // plt::named_plot("ref_line3_XY", refrencelineInfo3.x, refrencelineInfo3.y,
  //                 "b--");
  plt::named_plot("ref_line4_XY", refrencelineInfo4.x, refrencelineInfo4.y,
                  "g");
  plt::named_plot("car_tracj_XY", car_tracj.x, car_tracj.y, "r");
  
  plt::legend();
  plt::axis("equal");
  plt::legend();
  plt::axis("equal");
  plt::legend();
  plt::show();
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