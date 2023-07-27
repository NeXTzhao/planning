#include <implicit_curve.h>

#include <memory>

#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

// 函数：生成弧长数据和坐标数据
void generateData(int numPoints, double startAngle, double endAngle,
                  double radius, std::vector<double> &sdata,
                  std::vector<double> &xdata, std::vector<double> &ydata) {
  sdata.resize(numPoints);
  xdata.resize(numPoints);
  ydata.resize(numPoints);

  // 添加数据点
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
    xdata[i] = x;
    ydata[i] = y;
    if (i == 0) {
      sdata[0] = 0.0;
    } else {
      double dx = x - xdata[i - 1];
      double dy = y - ydata[i - 1];
      double s = sdata[i - 1] + std::hypot(dx, dy);
      sdata[i] = s;
    }
  }
}

void generateMeshGrid(double startX, double endX, int numPointsX, double startY,
                      double endY, int numPointsY,
                      std::vector<std::vector<double>> &X,
                      std::vector<std::vector<double>> &Y) {
  std::vector<double> x;
  std::vector<double> y;

  double stepX = (endX - startX) / (numPointsX - 1);
  double stepY = (endY - startY) / (numPointsY - 1);

  for (int i = 0; i < numPointsX; ++i) {
    double val = startX + i * stepX;
    x.push_back(val);
  }

  for (int i = 0; i < numPointsY; ++i) {
    double val = startY + i * stepY;
    y.push_back(val);
  }

  X.resize(numPointsY, std::vector<double>(numPointsX));
  Y.resize(numPointsY, std::vector<double>(numPointsX));

  for (int i = 0; i < numPointsY; ++i) {
    for (int j = 0; j < numPointsX; ++j) {
      X[i][j] = x[j];
      Y[i][j] = y[i];
    }
  }
}

int main() {
  // 调用函数生成弧长数据和坐标数据
  int numPoints = 100;
  double startAngle = 0.0;
  double endAngle = M_PI;
  double radius = 100.0;

  std::vector<double> sdata, xdata, ydata;

  generateData(numPoints, startAngle, endAngle, radius, sdata, xdata, ydata);

  auto fit = std::make_unique<PolyCurveFit>(sdata, xdata, ydata, 2);
  std::vector<double> x_fit, y_fit;
  fit->getXYFitData(x_fit, y_fit);
  auto px = fit->getXFitcoffs();
  auto py = fit->getYFitcoffs();

  px = std::vector<double>{-2, 3, 3, 1};
  py = std::vector<double>{5.935925e-15, -6.000000e+00, 6.000000e+00, 1.000000e+00};

  std::reverse(px.begin(), px.end());
  std::reverse(py.begin(), py.end());
  for (int i = 0; i < px.size(); ++i) {
    std::cout << "x = " << px[i] << " , "
              << "y = " << py[i] << '\n';
  }
  double start = -100.0;
  double end = 100.0;
  int xnumPoints = 100;

  std::vector<std::vector<double>> X;
  std::vector<std::vector<double>> Y;

  generateMeshGrid(start, end, xnumPoints, start, end, numPoints, X, Y);

  std::vector<std::vector<double>> Z(X.size(),
                                     std::vector<double>(X[0].size(), 0.0));
  auto curve = std::make_shared<Poly_Implicit>(px, py);

  for (int i = 0; i < X.size(); ++i) {
    for (int j = 0; j < X[0].size(); ++j) {
      Z[i][j] = curve->eval(X[i][j], Y[i][j]);
    }
  }

  std::cout << "get dis = " << std::abs(curve->eval(0, 100)) << std::endl;
  /**************************************************/

  plt::figure();

  plt::subplot(1, 2, 1);
  plt::named_plot("raw", xdata, ydata, "r.");
  plt::named_plot("fit_curve", x_fit, y_fit, "b-");
  plt::legend();
  plt::axis("equal");
  plt::title("fit curve");

  plt::subplot(1, 2, 2);
  plt::contour(X, Y, Z);
  plt::axis("equal");
  plt::title("sdf");

  plt::show();

  return 0;
}
