#include "bezier.h"
#include "iostream"
#include "matplotlibcpp.h"
#include <memory>
namespace plt = matplotlibcpp;

int main() {
  //  std::vector<CubicBezier::Point2> ctrlPoints = {
  //      {0.0, 0.0},
  //      {1.0, 2.0},
  //      {3.0, 1.0},
  //      {4.0, 3.0}
  //  };
  //
  //  double t = 0.5;
  //  CubicBezier::Point2 result = CubicBezier::q(ctrlPoints, t);
  //
  //  // Output the result
  //  std::cout << "Point at t=0.5: (" << result.x << ", " << result.y << ")" << std::endl;
  auto curve = std::make_unique<Bezier2Poly>();
  return 0;
}