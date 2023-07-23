//
// Created by vtd on 23-7-24.
//
#include "piecewise_bezier_curve.h"
#include "vector"
#include "iostream"

int main() {
  // 示例用法
  Point2 points[] = {{0.0, 0.0}, {1.0, 1.5}, {2.0, 2.0}, {3.0, 1.0}, {4.0, 0.5}};
  int numPoints = sizeof(points) / sizeof(points[0]);

  double errorThreshold = 0.1; // 设置拟合误差阈值
  BezierFitting bezierFitting(errorThreshold);
  bezierFitting.FitCurve(points, numPoints);

  return 0;
}