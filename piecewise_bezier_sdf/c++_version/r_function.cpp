#include "r_function.h"

#include <chrono>
#include <cmath>

RFunction::RFunction(std::vector<double> &px, std::vector<double> &py,
                     const std::vector<Point> &control_points)
    : control_points_(control_points) {
  std::reverse(px.begin(), px.end());
  std::reverse(py.begin(), py.end());
  implicit_curve_ = std::make_shared<Poly_Implicit>(px, py);
}

/**
 * @brief  通过R-fun进行将凸包和曲线进行组合
                R-fun = (f^2 + ((t^2 + f^4)^2 - t)^2 * 0.25)^0.5
 * @param f
 * @param t
 * @return
 */
double RFunction::trim(double f, double t) {
  double fSquared = f * f;
  double tSquared = t * t;
  double fToTheFourth = fSquared * fSquared;
  double innerExpression = std::sqrt(tSquared + fToTheFourth) - t;
  double innerExpressionSquared = innerExpression * innerExpression;
  double result = std::sqrt(fSquared + innerExpressionSquared * 0.25);
  return result;
}

void RFunction::normalizeSdf(SdfFunction &arr) {
  double min_val = *std::min_element(arr[0].begin(), arr[0].end());
  double max_val = *std::max_element(arr[0].begin(), arr[0].end());

  for (const auto &row : arr) {
    double row_min = *std::min_element(row.begin(), row.end());
    double row_max = *std::max_element(row.begin(), row.end());
    min_val = std::min(min_val, row_min);
    max_val = std::max(max_val, row_max);
  }
  if (max_val == min_val) {
    for (auto &row : arr) {
      std::fill(row.begin(), row.end(), 0.0);
    }
    return;
  }
  double range = max_val - min_val;
  for (auto &row : arr) {
    for (double &val : row) {
      val = (val - min_val) / range;
    }
  }
}

/**
 * @brief 使用 R-conjunction 拼接多个经裁剪后的分段贝塞尔等势面
                 R-conjunction = sdf1 + sdf2 - (sdf1^p + sdf2^p)^(1/p)
 * @param sdf_fields
 * @param p
 * @return
 */
double RFunction::composedSdf(const std::vector<double> &sdf_fields, int p) {
  std::vector<double> normalized_sdf_fields = sdf_fields;
  double result = normalized_sdf_fields[0];

  for (size_t i = 1; i < normalized_sdf_fields.size(); ++i) {
    double sdf = normalized_sdf_fields[i];
    double sum_powers = std::pow(result, p) + std::pow(sdf, p);
    double denominator = std::pow(sum_powers, 1.0 / p);
    result = result + sdf - denominator;
  }
  return result;
}

std::vector<double> RFunction::linspace(double start, double end,
                                        size_t numPoints) {
  std::vector<double> result(numPoints);
  double step = (end - start) / (numPoints - 1);
  std::iota(result.begin(), result.end(), 0);
  std::transform(result.begin(), result.end(), result.begin(),
                 [start, step](double index) { return start + index * step; });
  return result;
}

double RFunction::generateLineSdf(double x, double y, Point &a, Point &b) {
  double x1 = a.x;
  double y1 = a.y;
  double x2 = b.x;
  double y2 = b.y;

  return (x2 * (-y + y1) + x1 * (y - y2) + x * (-y1 + y2)) / (Power(x1 - x2, 2) + Power(y1 - y2, 2));
}

double RFunction::composedLineSdf(double sdfA, double sdfB) {
  return sdfA + sdfB - Sqrt(Power(sdfA, 2) + Power(sdfB, 2));
}

//double RFunction::getPolygonSdf(double x, double y) const {
//
//  auto p0 = control_points_[0];
//  auto p1 = control_points_[1];
//  auto p2 = control_points_[2];
//  auto p3 = control_points_[3];
//
//  auto line0 = generateLineSdf(x, y, p0, p1);
//  auto line1 = generateLineSdf(x, y, p1, p2);
//  auto line2 = generateLineSdf(x, y, p2, p3);
//  auto line3 = generateLineSdf(x, y, p3, p0);
//
//  auto r1 = composedLineSdf(line0, line1);
//  auto r2 = composedLineSdf(r1, line2);
//  auto res = composedLineSdf(r2, line3);
//  return res;
//}

// 计算多边形的SDF
double RFunction::getPolygonSdf(double x, double y) const {
  double res = 0.0;
  int num_points = control_points_.size();

  for (int i = 0; i < num_points; i++) {
    Point p0 = control_points_[i];
    Point p1 = control_points_[(i + 1) % num_points];// 获取下一个点，形成闭合多边形

    double line_sdf = generateLineSdf(x, y, p0, p1);
    res = composedLineSdf(res, line_sdf);
  }

  return res;
}

/**
 * @brief 曲线等势面步骤：
            1.得到贝塞尔控制点
            2.将贝塞尔转化为多项式得到多项式系数，得到曲线等势面
            3.输入(x,y)得到计算值
 * @param x
 * @param y
 * @return
 */
double RFunction::trimmingArea(double x, double y) const {

  //  贝塞尔凸包等势面
  auto scale = implicit_curve_->scale + 1e-6;
  auto start_t = std::chrono::high_resolution_clock::now();
  double t = getPolygonSdf(x, y);
  auto end_t = std::chrono::high_resolution_clock::now();

  auto start_f = std::chrono::high_resolution_clock::now();
  double f = implicit_curve_->eval3(x, y);
  auto end_f = std::chrono::high_resolution_clock::now();

  auto start_trim = std::chrono::high_resolution_clock::now();
  auto result = trim(f, t);
  auto end_trim = std::chrono::high_resolution_clock::now();

  // 计算执行时间并输出
  auto duration_t = std::chrono::duration_cast<std::chrono::microseconds>(end_t - start_t).count();
  auto duration_f = std::chrono::duration_cast<std::chrono::microseconds>(end_f - start_f).count();
  auto duration_trim = std::chrono::duration_cast<std::chrono::microseconds>(end_trim - start_trim).count();

  //  std::cout << "t time: " << duration_t / 1000.0 << "ms\n";
  //  std::cout << "f time: " << duration_f / 1000.0 << "ms\n";
  //  std::cout << "trim time: " << duration_trim / 1000.0 << "ms\n";
  //  printf("scale = %f, t = %f, f = %f, result = %f\n",

  return result;
}