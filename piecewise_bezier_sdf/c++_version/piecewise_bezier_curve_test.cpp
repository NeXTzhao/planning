#include "iostream"
#include "matplotlibcpp.h"
#include "piecewise_bezier_curve.h"
#include "vector"

namespace plt = matplotlibcpp;
int main() {
  std::vector<Point2> points = {
      {0.0, 0.0},
      {0.0, 0.5},
      {1.1, 1.4},
      {2.1, 1.6},
      {3.2, 1.1},
      {4.0, 0.2},
      {4.0, 0.0}};
  double error = 4.0;

  BezierFitting bezierFitting(points, error);
  bezierFitting.DrawBezierCurve(4);

  return 0;
}
