//
// Created by vtd on 23-7-24.
//
#include "bezier_to_poly.h"
#include "implicit_curve.h"
#include "matplotlibcpp.h"
#include "piecewise_bezier_curve2.h"

#include <chrono>
#include <iostream>
#include <memory>
namespace plt = matplotlibcpp;

// 函数：生成弧长数据和坐标数据
void generateData(std::vector<Point> &points) {

#if 0
  std::vector<double> x, y;


  int num_points = 30;
  for (int i = 0; i < num_points; ++i) {
    double t = i * M_PI / (num_points - 1);
    x.push_back(100 * std::cos(t));
    y.push_back(100 * std::sin(t));
  }

  // 翻转 x 和 y
  std::reverse(x.begin(), x.end());
  std::reverse(y.begin(), y.end());

  // 将数据保存到 Point 结构体
  for (size_t i = 0; i < x.size(); ++i) {
    Point point = {x[i], y[i]};
    points.push_back(point);
  }
#endif
#if 1
  std::vector<double> x, y;
  std::vector<double> tdata1 = {-60, -50, -40, -30, -20, -10};
  std::vector<double> tdata2 = {-10, -20, -30, -40, -50, -60};

  for (double t1 : tdata1) {
    x.push_back(100);
    y.push_back(t1);
  }

  int num_points = 30;
  for (int i = 0; i < num_points; ++i) {
    double t = i * M_PI / (num_points - 1);
    x.push_back(100 * std::cos(t));
    y.push_back(100 * std::sin(t));
  }

  for (double t2 : tdata2) {
    x.push_back(-100);
    y.push_back(t2);
  }

  // 翻转 x 和 y
  std::reverse(x.begin(), x.end());
  std::reverse(y.begin(), y.end());

  // 将数据保存到 Point 结构体
  for (size_t i = 0; i < x.size(); ++i) {
    Point point = {x[i], y[i]};
    points.push_back(point);
  }
//  for (const auto& point : points) {
//    std::cout << "x: " << point.x << ", y: " << point.y << std::endl;
//  }
#endif
#if 0
    for (int i = 0; i <= 50; i++) {
      points.push_back({static_cast<double>(i), 0});
    }
    double h = 0;
    for (int i = 0; i < 5; i++) {
      h++;
      points.push_back({50, h});
    }
    points.pop_back();
    points.push_back({50, 5});
    for (int i = 51; i < 61; i++) {
      points.push_back({static_cast<double>(i), 5});
    }
    for (int i = 0; i < 5; i++) {
      points.push_back({61, h});
      h--;
    }

    for (int i = 61; i <= 81; i++) {
      points.push_back({static_cast<double>(i), 0});
    }

    //    for (const auto &point : points) {
    //      std::cout << "x = " << point.x << " , "
    //                << " y = " << point.y << '\n';
    //    }
#endif
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
  std::vector<Point> points;

  generateData(points);

  auto start_Construct = std::chrono::high_resolution_clock::now();
  auto pieceBez = std::make_shared<PiecewiseBezierFit2>(points, 2, 6);
  auto end_Construct = std::chrono::high_resolution_clock::now();
  auto Construct_time = std::chrono::duration_cast<std::chrono::microseconds>(
                            end_Construct - start_Construct)
                            .count();
  std::cout << "PiecewiseBezierFit time: " << Construct_time / 1000.0 << "ms\n";

  auto control_point = pieceBez->getControlPoints();

  auto curve_points = pieceBez->getPiecewiseBezierCurvesPoints();

  double start = -300.0;
  double end = 300.0;
  int xnumPoints = 100;

  std::vector<std::vector<double>> X;
  std::vector<std::vector<double>> Y;

  generateMeshGrid(start, end, xnumPoints, start, end, xnumPoints, X, Y);

  std::vector<std::vector<double>> Z(X.size(),
                                     std::vector<double>(X[0].size(), 0.0));
  for (size_t i = 0; i < X.size(); ++i) {
    for (size_t j = 0; j < X[0].size(); ++j) {
      Z[i][j] = pieceBez->getSdfDis(X[i][j], Y[i][j]);
    }
  }
  auto start_cal_dis = std::chrono::high_resolution_clock::now();
  auto dis = std::abs(pieceBez->getSdfDis(55, 10));
  auto end_cal_dis = std::chrono::high_resolution_clock::now();

  // 计算执行时间并输出
  auto cal_dis_time = std::chrono::duration_cast<std::chrono::microseconds>(
                          end_cal_dis - start_cal_dis)
                          .count();
  std::cout << "cal dis time: " << cal_dis_time / 1000.0 << "ms\n";

  std::cout << "dis = " << dis << std::endl;
  //  //  /**************************************************/
#if 1
  std::vector<double> row_x, row_y;
  for (const auto point : points) {
    row_x.push_back(point.x);
    row_y.push_back(point.y);
  }

  plt::named_plot("row", row_x, row_y, "r.");
  int curveCount = 1;// 记录曲线编号
  for (const auto &curve : curve_points) {
    std::vector<double> x, y;
    for (const auto &point : curve) {
      x.push_back(point.x);
      y.push_back(point.y);
    }
    // 使用 std::to_string 将编号转换为字符串
    std::string curveName = "Curve " + std::to_string(curveCount);

    plt::named_plot(curveName, x, y, "-");

    curveCount++;
    std::vector<double> con_x, con_y;
    int color_index = 0;// 颜色索引
    for (const auto &cons : control_point) {
      for (const auto &con : cons) {
        con_x.push_back(con.x);
        con_y.push_back(con.y);
      }
    }
    plt::named_plot("con_pt", con_x, con_y, "bo");
  }
  //  plt::contour(X, Y, Z);
  plt::grid("true");
  plt::axis("equal");
  plt::legend();
  plt::show();
#endif
  return 0;
}
