#include "lane_line_construction.h"
#include "matplotlibcpp.h"
#include <memory>

namespace plt = matplotlibcpp;

int main() {
  std::vector<std::vector<double>> config{
// straight line: length
      {30},
//curve: Angle Radius
      {150, 10},
      {30},
//      {180, 5},
//      {36},
//      {-180, 12},
//      {5}
  };

  auto lane1 = std::make_shared<LaneLine>(config, single_lane);


  /***********************************************************/
  auto center_line = lane1->getCenterLinePoints();
  auto left_bound = lane1->getLeftBoundPoints();
  auto right_bound = lane1->getRightBoundP0ints();

  std::vector<double> cen_x, cen_y, left_x, left_y, right_x, right_y;
  for (int i = 0; i < center_line.size(); ++i) {
    cen_x.push_back(center_line[i].x);
    cen_y.push_back(center_line[i].y);

    left_x.push_back(left_bound[i].x);
    left_y.push_back(left_bound[i].y);

    right_x.push_back(right_bound[i].x);
    right_y.push_back(right_bound[i].y);
  }

  plt::named_plot("center_line", cen_x, cen_y, "b--");
  plt::named_plot("left_bound", left_x, left_y, "k-");
  plt::named_plot("right_bound", right_x, right_y, "r-");
  plt::grid(true);
  plt::axis("equal");
  plt::legend();
  plt::show();
}