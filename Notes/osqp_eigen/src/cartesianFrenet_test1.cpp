/**
 * @file cartesianFrenet.cpp
 * @brief
 * @author Wang Dezhao (1282507109@qq.com)
 * @version 1.0
 * @date 2022-05-14 17:37:37
 *
 * @copyright Copyright (c) 2022
 */

#include "angle.h"
#include "cartesianFrenet.hpp"
#include "kd_tree.hpp"
#include "matplotlibcpp.h"
#include "quinticPolynomial.hpp"

using namespace apollo;
using namespace common;
using namespace math;
using namespace cpprobotics;
using Poi_d = std::array<double, 2>;
using point2d = PathKdTreePoint<double, 2>;
using tree2d = PathKdTree<double, 2>;
namespace plt = matplotlibcpp;

PathKdTree<double, 2> pathKdTree;

int main() {
  PointState x_start{0.0, 0.0, 0.0};
  PointState x_end{200.0, 0.0, 0.0};
  PointState y_start{0.0, 0.0, 0.0};
  PointState y_end{30.0, 0.0, 0.0};

  double TotalTimes = x_end.xy - x_start.xy;
  FrenetPath refrencelineInfo;
  QuinticPolynomial refrenceline(x_start, x_end, y_start, y_end, TotalTimes);
  refrenceline.getPloyPath(refrencelineInfo, 1.0);

  std::vector<point2d> pointList;
  auto time2 = std::chrono::system_clock::now();

  pathkdtree::makeKdTree(pointList, refrencelineInfo.x, refrencelineInfo.y);
  tree2d PathKdTree_(std::begin(pointList), std::end(pointList));
  // fun PathKdTree = PathKdTree_;
  pathKdTree = PathKdTree_;

  // PathKdTree.tree_.nearest()
  auto time3 = std::chrono::system_clock::now();
  std::chrono::duration<double> diff2 = time3 - time2;
  std::cout << "Time for making kdtree = " << diff2.count() * 1000
            << " msec.\n";

  PointState carx_start{0.0, 0.0, 0.0};
  PointState carx_end{80.0, 0.0, 0.0};
  PointState cary_start{0.0, 0.0, 0.0};
  PointState cary_end{-10.0, 0.0, 0.0};

  // /*****************************************************************************************************/

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

    for (size_t i = 0; i < car_tracj.t.size(); ++i) {
      auto time1 = std::chrono::system_clock::now();
      PathKdTreePoint<double, 2> findpoint =
          pathKdTree.nearest({car_tracj.x.at(i), car_tracj.y.at(i)});
      auto index = std::find(refrencelineInfo.x.begin(),
                             refrencelineInfo.x.end(), findpoint.get(0));
      if (index == refrencelineInfo.x.end()) {
        std::cout << "not finf point" << std::endl;
        continue;
      }
      int cur_index = index - refrencelineInfo.x.begin();
      auto time2 = std::chrono::system_clock::now();

      std::cout << "cur_index:" << cur_index << '\n';
      std::chrono::duration<double> diff = time2 - time1;
      std::cout << "Time for matching Point = " << diff.count() * 1000
                << " msec.\n";

      if (cur_index >= static_cast<int>(car_tracj.t.size())-1) {
        cur_index = car_tracj.t.size() - 1;
      }
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

  plt::named_plot("ref_line_XY", refrencelineInfo.x, refrencelineInfo.y);
  plt::legend();
  plt::axis("equal");
  plt::legend();
  plt::axis("equal");
  plt::legend();
  plt::show();

  return 0;
}