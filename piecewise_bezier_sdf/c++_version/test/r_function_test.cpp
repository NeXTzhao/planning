#include "matplotlibcpp.h"
#include "r_function.h"
#include <iostream>

namespace plt = matplotlibcpp;
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
  std::array<Point, 4> control_points{{1.0, 1.0}, {8.0, 10.0}, {16.0, 10.0}, {32.0, 1.0}};
  auto beziertopoly = std::make_unique<Bezier2Poly>(control_points);
  auto px = beziertopoly->getXCoefficients();
  auto py = beziertopoly->getYCoefficients();
  for (int i = 0; i < px.size(); ++i) {
    std::cout << "x = " << px[i] << " , "
              << "y = " << py[i] << '\n';
  }
  auto points_data = beziertopoly->getPolyCurvePoints();

  auto r_fun = std::make_shared<RFunction>(px, py, control_points);
  auto imp_cur = r_fun->implicit_curve_;
  printf("f (3,5) = %f\n", imp_cur->eval(3, 5));

  //  r_fun->trimmingArea(x, y);
  double start = -130.0;
  double end = 130.0;
  int numPoints = 100;

  std::vector<std::vector<double>> X;
  std::vector<std::vector<double>> Y;

  generateMeshGrid(start, end, numPoints, start, end, numPoints, X, Y);

  std::vector<std::vector<double>> Z(X.size(),
                                     std::vector<double>(X[0].size(), 0.0));

  for (int i = 0; i < X.size(); ++i) {
    for (int j = 0; j < X[0].size(); ++j) {
      Z[i][j] = r_fun->trimmingArea(X[i][j], Y[i][j]);
    }
  }
  std::cout << "get dis = " << std::abs(r_fun->trimmingArea(3, 5)) << std::endl;
  /**************************************************/
  std::vector<double> xdata, ydata;
  for (const auto &point : points_data) {
    xdata.push_back(point.x);
    ydata.push_back(point.y);
  }
  std::vector<double> x_con, y_con;
  for (auto point : control_points) {
    x_con.push_back(point.x);
    y_con.push_back(point.y);
  }
  x_con.push_back(control_points.begin()->x);
  y_con.push_back(control_points.begin()->y);

  /*********************************************************/
  plt::figure();
  plt::subplot(1, 2, 1);
  plt::named_plot("control_points", x_con, y_con, "b--");
  plt::named_plot("raw", xdata, ydata, "r.");
  //  plt::named_plot("fit_curve", x_fit, y_fit, "b-");
  plt::legend();
  //  plt::axis("equal");
  plt::title("fit curve");
  plt::subplot(1, 2, 2);
  plt::contour(X, Y, Z);
  //  plt::axis("equal");
  plt::title("sdf");

  plt::show();

  return 0;
}