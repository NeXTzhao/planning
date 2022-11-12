#include "matplotlibcpp.h"
#include "reeds_shepp_path.h"
#include <iostream>
#include <memory>
#include <vector>

namespace plt = matplotlibcpp;
using namespace apollo::planning;
using namespace apollo::common;

void test_rs() {
  std::vector<double> XYbounds_;
  XYbounds_.push_back(-1000.0);
  XYbounds_.push_back(1000.0);
  XYbounds_.push_back(-1000.0);
  XYbounds_.push_back(1000.0);

  auto reedshepp_test = std::make_unique<ReedShepp>();
  std::shared_ptr<Node3d> start_node = std::make_shared<Node3d>(
    0.0, 0.0, -0.0 * M_PI / 180.0, XYbounds_);
  std::shared_ptr<Node3d> end_node = std::make_shared<Node3d>(
    -7.0, -7.0, -15 * M_PI / 180.0, XYbounds_);
  std::shared_ptr<ReedSheppPath> optimal_path =
    std::make_shared<ReedSheppPath>();
  reedshepp_test->ShortestRSP(start_node, end_node, optimal_path);

  /*******************************************/
  plt::subplot(1,2,1);
  plt::named_plot("XY", optimal_path->x, optimal_path->y,"g");
  plt::legend();
  plt::axis("equal");
  plt::subplot(1,2,2);
  plt::named_plot("heading", optimal_path->phi,"--");
  plt::legend();
  plt::show();

}

int main() {
  std::cout << "Hello World!" << '\n';
  test_rs();


  return 0;
}