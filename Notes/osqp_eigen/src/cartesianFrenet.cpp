/**
 * @file cartesianFrenet.cpp
 * @brief
 * @author Wang Dezhao (1282507109@qq.com)
 * @version 1.0
 * @date 2022-05-02 10:11:53
 *
 * @copyright Copyright (c) 2022
 */

#include "cartesianFrenet.hpp"

#include "matplotlibcpp.h"
#include "quinticPolynomial.hpp"

using namespace apollo;
using namespace common;
using namespace math;
using namespace cpprobotics;

int main() {
  PointState x_start{0.0, 0.0, 0.0};
  PointState x_end{100.0, 0.0, 0.0};
  PointState y_start{0.0, 0.0, 0.0};
  PointState y_end{20.0, 0.0, 0.0};

  double TotalTimes = x_end.xy;
  FrenetPath fp;
  QuinticPolynomial quinticPloy(x_start, x_end, y_start, y_end, TotalTimes);
  quinticPloy.getPloyPath(fp, 1);

  PointState carx_start{0.0, 0.0, 0.0};
  PointState carx_end{50.0, 0.0, 0.0};
  PointState cary_start{0.0, 0.0, 0.0};
  PointState cary_end{-10.0, 0.0, 0.0};

  namespace plt = matplotlibcpp;
  // plt::named_plot("sloveXY", fp.x, fp.y);
  // std::array<double, 3> s_conditions;
  // std::array<double, 3> d_conditions;
  std::vector<double> s;
  std::vector<double> d;
  double loop = 11;
  double delta_y = std::abs(cary_end.xy) * 2 / loop;
  for (size_t i = 0; i < loop; ++i) {
    double times = carx_end.xy;
    FrenetPath car_fp;
    cary_end.xy += delta_y;
    QuinticPolynomial car_quinticPloy(carx_start, carx_end, cary_start,
                                      cary_end, times);
    car_quinticPloy.getPloyPath(car_fp, 1);

    // for (size_t i = 0; i < fp.t.size(); ++i) {
    //   CartesianFrenetConverter::cartesian_to_frenet(
    //       fp.s.at(i), fp.x.at(i), fp.y.at(i), fp.theta.at(i), fp.kappa.at(i),
    //       fp.dkappa.at(i), car_fp.x.at(i), car_fp.y.at(i), car_fp., a, theta,
    //       kappa, &s_conditions, &d_conditions);
    // }

    double ptr_s, ptr_d;
    for (size_t i = 0; i < car_fp.t.size(); ++i) {
      int index = 0;
      car_quinticPloy.matchPoint(fp, car_fp.x.at(i), car_fp.y.at(i), index);
      CartesianFrenetConverter::cartesian_to_frenet(
          fp.x.at(i), fp.x.at(i), fp.y.at(i), fp.theta.at(i),
          car_fp.x.at(i), car_fp.y.at(i), &ptr_s, &ptr_d);
      s.push_back(ptr_s);
      d.push_back(ptr_d);
    }
    // plt::named_plot("car_sloveXY", car_fp.x, car_fp.y);
    // plt::named_plot("frenet_sloveXY", d);
    std::vector<double> temp_s;
    std::vector<double> temp_d;

    int nu = car_fp.t.size();
    for (const auto& item : fp.s) {
      if (nu > 0) {
        temp_s.push_back(item);
        std::cout<<"s:"<<item<<'\n';
      }
      nu--;
    }
    int nua =car_fp.t.size();
    for (const auto& item : fp.y) {
      if (nu > 0) {
        temp_d.push_back(item);
        
      }
      nua--;
    }
    // plt::named_plot("frenet_sloveXY", temp_s, d);
    plt::named_plot("frenet_sloveXY", s, d);

    // plt::named_plot("path_sloveXY", temp_s, temp_d);

    s.clear();
    d.clear();
    temp_s.clear();
    temp_d.clear();

  }

  // {
  //   def cartesian_to_frenet1D(rs, rx, ry, rtheta, x, y):
  //     void cartesian_to_frenet1D(double rs,double rx,double ry,double
  //     rtheta,) s_condition = np.zeros(1) d_condition = np.zeros(1)
  //     std::vector<double> s_condition;
  //     std::vector<double> d_condition;

  //     dx = x - rx
  //     dy = y - ry

  //     cos_theta_r = cos(rtheta)
  //     sin_theta_r = sin(rtheta)

  //     cross_rd_nd = cos_theta_r * dy - sin_theta_r * dx
  //     d_condition[0] = copysign(sqrt(dx * dx + dy * dy), cross_rd_nd)

  //     s_condition[0] = rs

  //     return s_condition, d_condition
  // }

  // plt::named_plot("car_sloveXY", car_fp.x, car_fp.y);

  // plt::named_plot("sloveXY", fp.y, "r-o");
  // plt::named_plot("X_sl", x1, y1, "g-o");
  // plt::named_plot("X_xy", x1_, y1_, "r-o");
  // plt::named_plot("ref_xy", ref_x, ref_y, "r-o");

  // plt::named_plot("Y", y1, "r-o");

  //   plt::named_plot("slovekapp", fp.k,
  //   "r-o");
  plt::legend();
  plt::axis("equal");
  //   plt::figure();
  //   plt::named_plot("slove_kappa", t_coord,
  //   ref_kappa, "r-o");
  //   // plt::named_plot("ref_kappa", t_coord,
  //   constraint_curature, "g-o");
  //   plt::legend();
  plt::show();
  return 0;
}