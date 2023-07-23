#pragma once

#include <iostream>
#include <vector>
#include <cmath>

class Point2 {
 public:
  double x, y;
};

class BezierFitting {
 public:
  // 构造函数
  explicit BezierFitting(double error) : error_(error) {}

  // 拟合贝塞尔曲线的入口函数
  void FitCurve(Point2 *d, int nPts);

 private:
  // 类内部辅助函数声明
  void FitCubic(std::vector<Point2> &d, int first, int last, Point2 tHat1, Point2 tHat2);
  static double *Reparameterize(std::vector<Point2> &d, int first, int last, double *u, std::vector<Point2> &bezCurve);
  static double NewtonRaphsonRootFind(std::vector<Point2> &Q, Point2 P, double u);
  static Point2 BezierII(int degree, std::vector<Point2> &V, double t);
  static double B0(double u);
  static double B1(double u);
  static double B2(double u);
  static double B3(double u);
  std::vector<std::vector<double>> ComputeB(double *u);
  static Point2 ComputeLeftTangent(std::vector<Point2> &d, int end);
  static Point2 ComputeRightTangent(std::vector<Point2> &d, int end);
  static Point2 ComputeCenterTangent(std::vector<Point2> &d, int center);
  static double *ChordLengthParameterize(std::vector<Point2> &d, int first, int last);
  std::vector<Point2> GenerateBezier(std::vector<Point2> &d,
                                     int first,
                                     int last,
                                     double *u,
                                     Point2 tHat1,
                                     Point2 tHat2);
  static double ComputeMaxError(std::vector<Point2> &d,
                                int first,
                                int last,
                                std::vector<Point2> &bezCurve,
                                double *u,
                                int *splitPoint);

  // 绘制贝塞尔曲线的函数，你可以根据需求实现该函数来进行显示或输出
  static void DrawBezierCurve(const std::vector<Point2> &curve);

  double error_;
};


