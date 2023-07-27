#include "r_function.h"

#include <cmath>

RFunction::RFunction(std::vector<double> &px, std::vector<double> &py,
                     const std::array<Point, 4> &control_points)
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

void fun(const std::vector<std::array<Point, 4>> &control_points) {
  for (const auto &con_pt : control_points) {
  }
}

double RFunction::trimmingArea(double x, double y) const {
  double x1 = control_points_[0].x;
  double y1 = control_points_[0].y;

  double x2 = control_points_[1].x;
  double y2 = control_points_[1].y;

  double x3 = control_points_[2].x;
  double y3 = control_points_[2].y;

  double x4 = control_points_[3].x;
  double y4 = control_points_[3].y;
  //  贝塞尔凸包等势面
  double sdf_map_time = clock();

  double t =
      (-((-x1 + x2) * (y - y1)) + (x - x1) * (-y1 + y2)) /
          (Power(-x1 + x2, 2) + Power(-y1 + y2, 2)) +
      (-((-x2 + x3) * (y - y2)) + (x - x2) * (-y2 + y3)) /
          (Power(-x2 + x3, 2) + Power(-y2 + y3, 2)) -
      Sqrt(Power(-((-x1 + x2) * (y - y1)) + (x - x1) * (-y1 + y2), 2) /
               Power(Power(-x1 + x2, 2) + Power(-y1 + y2, 2), 2) +
           Power(-((-x2 + x3) * (y - y2)) + (x - x2) * (-y2 + y3), 2) /
               Power(Power(-x2 + x3, 2) + Power(-y2 + y3, 2), 2)) +
      (-((x1 - x4) * (y - y4)) + (x - x4) * (y1 - y4)) /
          (Power(x1 - x4, 2) + Power(y1 - y4, 2)) +
      (-((-x3 + x4) * (y - y3)) + (x - x3) * (-y3 + y4)) /
          (Power(-x3 + x4, 2) + Power(-y3 + y4, 2)) -
      Sqrt(
          Power((-((-x1 + x2) * (y - y1)) + (x - x1) * (-y1 + y2)) /
                        (Power(-x1 + x2, 2) + Power(-y1 + y2, 2)) +
                    (-((-x2 + x3) * (y - y2)) + (x - x2) * (-y2 + y3)) /
                        (Power(-x2 + x3, 2) + Power(-y2 + y3, 2)) -
                    Sqrt(Power(-((-x1 + x2) * (y - y1)) + (x - x1) * (-y1 + y2),
                               2) /
                             Power(Power(-x1 + x2, 2) + Power(-y1 + y2, 2), 2) +
                         Power(-((-x2 + x3) * (y - y2)) + (x - x2) * (-y2 + y3),
                               2) /
                             Power(Power(-x2 + x3, 2) + Power(-y2 + y3, 2), 2)),
                2) +
          Power(-((-x3 + x4) * (y - y3)) + (x - x3) * (-y3 + y4), 2) /
              Power(Power(-x3 + x4, 2) + Power(-y3 + y4, 2), 2)) -
      Sqrt(Power(-((x1 - x4) * (y - y4)) + (x - x4) * (y1 - y4), 2) /
               Power(Power(x1 - x4, 2) + Power(y1 - y4, 2), 2) +
           Power(
               (-((-x1 + x2) * (y - y1)) + (x - x1) * (-y1 + y2)) /
                       (Power(-x1 + x2, 2) + Power(-y1 + y2, 2)) +
                   (-((-x2 + x3) * (y - y2)) + (x - x2) * (-y2 + y3)) /
                       (Power(-x2 + x3, 2) + Power(-y2 + y3, 2)) -
                   Sqrt(Power(-((-x1 + x2) * (y - y1)) + (x - x1) * (-y1 + y2),
                              2) /
                            Power(Power(-x1 + x2, 2) + Power(-y1 + y2, 2), 2) +
                        Power(-((-x2 + x3) * (y - y2)) + (x - x2) * (-y2 + y3),
                              2) /
                            Power(Power(-x2 + x3, 2) + Power(-y2 + y3, 2), 2)) +
                   (-((-x3 + x4) * (y - y3)) + (x - x3) * (-y3 + y4)) /
                       (Power(-x3 + x4, 2) + Power(-y3 + y4, 2)) -
                   Sqrt(Power(
                            (-((-x1 + x2) * (y - y1)) + (x - x1) * (-y1 + y2)) /
                                    (Power(-x1 + x2, 2) + Power(-y1 + y2, 2)) +
                                (-((-x2 + x3) * (y - y2)) +
                                 (x - x2) * (-y2 + y3)) /
                                    (Power(-x2 + x3, 2) + Power(-y2 + y3, 2)) -
                                Sqrt(Power(-((-x1 + x2) * (y - y1)) +
                                               (x - x1) * (-y1 + y2),
                                           2) /
                                         Power(Power(-x1 + x2, 2) +
                                                   Power(-y1 + y2, 2),
                                               2) +
                                     Power(-((-x2 + x3) * (y - y2)) +
                                               (x - x2) * (-y2 + y3),
                                           2) /
                                         Power(Power(-x2 + x3, 2) +
                                                   Power(-y2 + y3, 2),
                                               2)),
                            2) +
                        Power(-((-x3 + x4) * (y - y3)) + (x - x3) * (-y3 + y4),
                              2) /
                            Power(Power(-x3 + x4, 2) + Power(-y3 + y4, 2), 2)),
               2));
  // 曲线等势面步骤：
  //  1.得到贝塞尔控制点
  //  2.将贝塞尔转化为多项式得到多项式系数，得到曲线等势面
  //  3.输入(x,y)得到计算值
  //  t = t * implicit_curve_->scale;
  //  double scale = implicit_curve_->scale;
  std::cout << "t time: " << 1000 * (clock() - sdf_map_time) / CLOCKS_PER_SEC
            << "ms \n";
  double f_time = clock();
  double f = implicit_curve_->eval(x, y);
  std::cout << "f time: " << 1000 * (clock() - f_time) / CLOCKS_PER_SEC
            << "ms \n";

  auto result = trim(f, t);
  //  printf("scale = %f, t = %f, f = %f, result = %f\n",
  //  implicit_curve_->scale, t, f, result);

  return result;
}