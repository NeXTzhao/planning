#pragma once

#include "bezier_to_poly.h"
#include "implicit_curve.h"
#include <cmath>
#include <memory>
#include <numeric>
#include <vector>

template<typename T, typename U>
T Power(T base, U exponent) {
  return std::pow(base, static_cast<T>(exponent));
}

template<typename T>
T Sqrt(T x) {
  return std::sqrt(x);
}

using SdfFunction = std::vector<std::vector<double>>;

class RFunction {
 public:
  RFunction() = default;
  //  RFunction(std::vector<double> &px, std::vector<double> &py, const std::array<Point, 4> &control_points);
  RFunction(std::vector<double> &px, std::vector<double> &py,
            const std::vector<Point> &control_points);

  /**
   * @brief 凸包裁剪等势面
   * @param f 等势面隐函数
   * @param t 凸包多边形等势面
   * @return
   */
  static double trim(double f, double t);
  /**
   * @brief 归一化
   * @param sdf
   * @return
   */
  static void normalizeSdf(SdfFunction &sdf);
  /**
   * @brief 通过R函数拼接多个距离场
   * @param sdf_fields 多个距离场
   * @param p 范数
   * @return
   */
  static double composedSdf(const std::vector<double> &sdf_fields, int p = 2);
  /**
   * @brief 构造线段距离场
   * @param x 变量
   * @param y 变量
   * @param start_point 起点 
   * @param end_point 终点
   * @return
   */
  //  static double sdfLineSegment(double x, double y, const Point &start_point, const Point &end_point);
  /**
   * @brief 拼接两个线段距离场
   * @param line_a
   * @param line_b
   * @return
   */
  //  static SdfFunction composedSdfLine(const SdfFunction &line_a, const SdfFunction &line_b);
  /**
   * @brief 生成四边形凸包距离场
   * @param X 变量
   * @param Y --
   * @param p1 控制点
   * @param p2 --
   * @param p3 --
   * @param p4 --
   * @return
   */
  double trimmingArea(double x, double y) const;
  /**
   * @brief
   * @param start
   * @param end
   * @param numPoints
   * @return
   */
  static std::vector<double> linspace(double start, double end,
                                      size_t numPoints);
  static double generateLineSdf(double x, double y, Point &a, Point &b);

 public:
  std::vector<Point> control_points_;
  std::shared_ptr<Poly_Implicit> implicit_curve_;
  static double composedLineSdf(double sdfA, double sdfB);
  double getPolygonSdf(double x, double y) const;
};
