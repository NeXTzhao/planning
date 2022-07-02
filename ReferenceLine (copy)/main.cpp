#include <chrono>
#include <iostream>

//#include "curve.h"
#include "curve_creator.h"
//#include "matplotlibcpp.h"

#include "cpprobotics_types_double.h"
#include "quinticPolynomial.hpp"

//namespace plt = matplotlibcpp;

using namespace reference_line;
using namespace cpprobotics;

/**
 * quintic cure path
 * @param x
 * @param y
 */
void quintic_cure(std::vector<double> &x, std::vector<double> &y) {
  PointState x_start{0.0, 0.0, 0.0};
  PointState x_end{150.0, 0.0, 0.0};
  PointState y_start{0.0, 0.0, 0.0};
  PointState y_end{50, 0.0, 0.0};

  double TotalTimes = x_end.xy - x_start.xy;
  FrenetPath refrencelineInfo;
  QuinticPolynomial refrenceline(x_start, x_end, y_start, y_end, TotalTimes);
  refrenceline.getPloyPath(refrencelineInfo, 1.0);

  //      x = std::move(refrencelineInfo.x);
  //      y = std::move(refrencelineInfo.y);
  int length = 50;
  x.assign(refrencelineInfo.x.begin() + length,
           refrencelineInfo.x.end() - length);
  y.assign(refrencelineInfo.y.begin() + length,
           refrencelineInfo.y.end() - length);
}

/**
 * @brief 等距弧长离散园
 * 由于圆的特殊性，其圆弧上每一点的曲率均相等，故可以将等弧长与相同的角度相对应
 * xy对应圆心坐标,r为半径,size用于设置划分的间距
 * @param  x
 * @param  y
 * @param  r
 * @param  size
 */
void div_circle(std::vector<double> &x, std::vector<double> &y, double r,
                double size) {
  double angle_step = 0.0; // 一小步的弧度
  angle_step = size / r;
  double x_out, y_out;

  for (int i = M_PI / angle_step; i > 0; --i) {
    x_out = r * std::cos(i * angle_step);
    y_out = r * std::sin(i * angle_step);
    x.push_back(x_out);
    y.push_back(y_out);
  }
}

void test_solve() {

  /*
        std::vector<double> raw_xs{
            4.946317773,   5.017218975,   4.734635316,  4.425064575,  3.960102096,
            3.503172702,   2.989950824,   2.258523535,  1.562447892,
            0.8764776599, 0.09899323097, -0.7132021974, -1.479055426,
            -2.170306775, -3.034455492, -3.621987909,  -3.979289889,
            -4.434628966, -4.818245921, -4.860190444, -5.09947597};
        std::vector<double> raw_ys{
            0.08436953512, 0.7205757236, 1.642930209, 2.365356462, 2.991632152,
            3.44091492,    3.9590821,    4.354377368, 4.46801472,  4.871705856,
            5.0285845841,  5.010851105,  4.680181989, 4.463442715, 4.074651273,
            3.585790302,   3.014232351,  2.367848826, 1.467395733, 0.8444358019,
            -0.01022405467};
  */

  std::vector<double> raw_xs, raw_ys;
  div_circle(raw_xs, raw_ys, 20, 1);
  //  quintic_cure(raw_xs, raw_ys);

  reference_line::CurveCreator creator(raw_xs, raw_ys);

  auto time2 = std::chrono::system_clock::now();
  creator.solve();
  auto time3 = std::chrono::system_clock::now();
  std::chrono::duration<double> diff2 = time3 - time2;
//  std::cout << "Time for solve time = " << diff2.count() * 1000 << " msec.\n";

  auto &curve = creator.curve();
  double length = curve->length();

  //  constexpr int N = 200;
  int N = ((int)raw_xs.size() - 1);

  double stride = length / N;
  printf("length=%f,stride=%f\n", length, stride);

  std::vector<double> final_xs, final_ys, final_theta, final_kappas,
      final_dkappas, final_deltas;

  for (int i = 0; i < N + 1; i++) {
    auto s = i * stride;
    final_xs.emplace_back(curve->x(s));
    final_ys.emplace_back(curve->y(s));
    final_theta.emplace_back(curve->theta(s));
    final_kappas.emplace_back(curve->kappa(s));
    final_dkappas.emplace_back(curve->dkappa(s));
    //    printf("final_x=%f , final_y=%f , final_theta=%f, final_kappa=%f
    //    s=%f\n",
    //           curve->x(s), curve->y(s), curve->theta(s), curve->kappa(s), s);
  }

  printf("final_xs.size = %zu,ref_x.size = %zu\n", final_xs.size(),
         raw_xs.size());
//
//  plt::named_plot("ref_line_XY", raw_xs, raw_ys, "*");
//  //    plt::named_plot("final_XY", final_xs, final_ys, "*");
//  plt::named_plot("final_XY", final_xs, final_ys);
//  plt::axis("equal");
//  plt::legend();
//  plt::show();
}

int main() {
  std::cout << "Hello, World!" << std::endl;
  test_solve();

  return 0;
}
