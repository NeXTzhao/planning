#include "bezier_to_poly.h"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

int main() {
  // 定义贝塞尔曲线的控制点
  std::vector<Point> controlPoints = {
      {0.0, 0.0},
      {1.0, 2.0},
      {3.0, 4.0},
      {5.0, 1.0}};

  // 创建BezierCurveFitting对象并进行拟合
  Bezier2Poly bezierCurveFitting(controlPoints);

  // 打印多项式系数和梯度
//  bezierCurveFitting.printCoefficients();

  // 获取拟合的多项式系数和曲线上的点
  std::vector<double> xCoeffs = bezierCurveFitting.getXCoefficients();
  std::vector<double> yCoeffs = bezierCurveFitting.getYCoefficients();
  std::vector<Point> curvePoints = bezierCurveFitting.getPolyCurvePoints();

  // 获取尺度
  double scale = bezierCurveFitting.getScale();

  // 输出拟合的多项式系数和曲线上的点
  //  std::cout << "XCoefficients: ";
  //  for (double coeff : xCoeffs) {
  //    std::cout << coeff << " ";
  //  }
  //  std::cout << std::endl;
  //
  //  std::cout << "YCoefficients: ";
  //  for (double coeff : yCoeffs) {
  //    std::cout << coeff << " ";
  //  }
  //  std::cout << std::endl;
  //
  //  std::cout << "Scaled Curve Points:" << std::endl;
  //  for (const auto &point : curvePoints) {
  //    std::cout << "x: " << point.x * scale << ", y: " << point.y * scale << std::endl;
  //  }

  /********************************************************/
  // 将 curvePoints 中的 x 和 y 分别存入两个 vector
  std::vector<double> xValues, yValues;

  for (const auto &point : curvePoints) {
    xValues.push_back(point.x);
    yValues.push_back(point.y);
  }

  std::vector<double> controlX, controlY;

  for (const auto &point : controlPoints) {
    controlX.push_back(point.x);
    controlY.push_back(point.y);
  }

  plt::named_plot("poly_curve", xValues, yValues, "r--");
  plt::named_plot("Control Points", controlX, controlY, "bo");
  plt::axis("equal");
  plt::grid(true);// 添加方格
  plt::legend();
  plt::show();
  return 0;
}
