#include "matplotlibcpp.h"
#include "piecewise_bezier_curve.h"
#include <memory>
#include <vector>

namespace plt = matplotlibcpp;
// 函数：生成弧长数据和坐标数据
void generateData(int numPoints, double startAngle, double endAngle,
                  double radius,
                  std::vector<Point> &point) {
  point.resize(numPoints);

  std::vector<double> tdata(numPoints);
  double angleIncrement = (endAngle - startAngle) / (numPoints - 1);
  for (int i = 0; i < numPoints; ++i) {
    double t = startAngle + i * angleIncrement;
    tdata[i] = t;
  }

  for (int i = 0; i < numPoints; ++i) {
    double t = tdata[i];
    double x = radius * std::cos(t);
    double y = radius * std::sin(t);
    point[i] = Point{x, y};
  }
}

int main() {
  std::vector<Point> points;

  // 调用函数生成弧长数据和坐标数据
  int numPoints = 50;
  double startAngle = 0.0;
  double endAngle = M_PI;
  double radius = 100.0;

  generateData(numPoints, startAngle, endAngle, radius, points);

  double error = 1;

  auto pieceBez = std::make_shared<PiecewiseBezierFit>(points, error);

  auto con = pieceBez->getControlPoints();

  auto curve_points = pieceBez->getPiecewiseBezierCurvesPoints();

  /********************** plt *************************************/
  std::vector<double> row_x, row_y;
  for (const auto point : points) {
    row_x.push_back(point.x);
    row_y.push_back(point.y);
  }

  plt::named_plot("row", row_x, row_y, "r.");

  for (const auto &curve : curve_points) {
    std::vector<double> x, y;
    for (const auto &point : curve) {
      x.push_back(point.x);
      y.push_back(point.y);
    }
    plt::named_plot("curve", x, y, "-");

    std::vector<double> con_x, con_y;
    for (const auto &con : curve) {
      con_x.push_back(con.x);
      con_y.push_back(con.y);
    }
    plt::named_plot("con_point", con_x, con_y, "*");
  }
  plt::grid("true");
  plt::legend();
  plt::show();

  return 0;
}
