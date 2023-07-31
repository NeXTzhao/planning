#include "implicit_curve.h"

#include <algorithm>
#include <cmath>
//namespace hiphi {
//namespace planning {
PolyCurveFit::PolyCurveFit(const std::vector<double> &sdata,
                           const std::vector<double> &xdata,
                           const std::vector<double> &ydata, int degree)
    : sdata_(sdata), xdata_(xdata), ydata_(ydata) {
  x_coeffs_ = cal_coffs(sdata, xdata, degree);
  y_coeffs_ = cal_coffs(sdata, ydata, degree);
  reserve_x_coeffs_ = x_coeffs_;
  reserve_y_coeffs_ = y_coeffs_;
  std::reverse(x_coeffs_.begin(), x_coeffs_.end());
  std::reverse(y_coeffs_.begin(), y_coeffs_.end());
}

//CurveFit::CurveFit(const ReferenceLine& ref_line, int degree) {
//  setXYSData(ref_line);
//  x_coeffs_ = cal_coffs(sdata_, xdata_, degree);
//  y_coeffs_ = cal_coffs(sdata_, ydata_, degree);
//  reserve_x_coeffs_ = x_coeffs_;
//  reserve_y_coeffs_ = y_coeffs_;
//  std::reverse(x_coeffs_.begin(), x_coeffs_.end());
//  std::reverse(y_coeffs_.begin(), y_coeffs_.end());
//}

void PolyCurveFit::getXYFitData(std::vector<double> &x,
                                std::vector<double> &y) {
  for (double i : sdata_) {
    x.push_back(cal_fit_xdata(i));
    y.push_back(cal_fit_ydata(i));
  }
}

void PolyCurveFit::getXYFitData() {
  for (double i : sdata_) {
    fit_x_data.push_back(cal_fit_xdata(i));
    fit_y_data.push_back(cal_fit_ydata(i));
  }
}

std::vector<double> PolyCurveFit::cal_coffs(const std::vector<double> &x,
                                            const std::vector<double> &y,
                                            int degree) {
  size_t n = x.size();
  int m = degree + 1;

  std::vector<double> A((m + 1) * (m + 1), 0.0);
  std::vector<double> B(m + 1, 0.0);

  // 构建矩阵A和向量B
  for (size_t i = 0; i < n; ++i) {
    double xi = x[i];
    double yi = y[i];
    double w = 1.0;

    // 根据权重w对矩阵A和向量B进行加权
    for (int j = 0; j <= m; ++j) {
      for (int k = 0; k <= m; ++k) {
        A[j * (m + 1) + k] += w * std::pow(xi, j + k);
      }
      B[j] += w * yi * std::pow(xi, j);
    }
  }

  // 解线性方程组，得到多项式系数
  std::vector<double> coefficients;
  for (int i = 0; i <= m; ++i) {
    for (int j = i + 1; j <= m; ++j) {
      double ratio = A[j * (m + 1) + i] / A[i * (m + 1) + i];
      for (int k = i; k <= m; ++k) {
        A[j * (m + 1) + k] -= ratio * A[i * (m + 1) + k];
      }
      B[j] -= ratio * B[i];
    }
  }

  // 回代求解系数
  for (int i = m; i >= 0; --i) {
    double coef = B[i];
    for (int j = i + 1; j <= m; ++j) {
      coef -= A[i * (m + 1) + j] * coefficients[m - j];
    }
    coef /= A[i * (m + 1) + i];
    coefficients.push_back(coef);
  }

  // 调整系数顺序以匹配 Python 的 polyfit 结果
  // std::reverse(coefficients.begin(), coefficients.end());

  return coefficients;
}

double PolyCurveFit::cal_fit_xdata(double x) const {
  double result = 0.0;
  double power = 1.0;
  for (double x_coeff : x_coeffs_) {
    result += x_coeff * power;
    power *= x;
  }
  return result;
}

double PolyCurveFit::cal_fit_ydata(double y) const {
  double result = 0.0;
  double power = 1.0;
  for (double y_coeff : y_coeffs_) {
    result += y_coeff * power;
    power *= y;
  }
  return result;
}
//void CurveFit::setXYSData(const ReferenceLine& ref_line) {
//  std::vector<double> x, y, s;
//
//  const auto& ref_points = ref_line.reference_points();
//  int refline_point_size = static_cast<int>(ref_points.size());
//
//  x.resize(refline_point_size);
//  y.resize(refline_point_size);
//  s.resize(refline_point_size);
//  for (size_t i = 0; i < refline_point_size; ++i) {
//    double x_val = ref_points[i].x();
//    double y_val = ref_points[i].y();
//
//    x[i] = x_val;
//    y[i] = y_val;
//    if (i == 0) {
//      s[0] = 0.0;
//    } else {
//      double dx = x_val - x[i - 1];
//      double dy = y_val - y[i - 1];
//      double s_val = s[i - 1] + std::hypot(dx, dy);
//      s[i] = s_val;
//    }
//  }
//  xdata_ = x;
//  ydata_ = y;
//  sdata_ = s;
//}

Poly_Implicit::Poly_Implicit(const std::vector<double> &px,
                             const std::vector<double> &py, int degree) {
  this->px = px;
  this->py = py;

  double x0 = evaluatePolynomial(px, 0.0);
  double y0 = evaluatePolynomial(py, 0.0);
  if (degree == 3) {
    double dx = gradx3(x0, y0);
    double dy = grady3(x0, y0);
    scale = std::hypot(dx, dy);
  } else if (degree == 2) {
    double dx = gradx2(x0, y0);
    double dy = grady2(x0, y0);
    scale = std::hypot(dx, dy);
  } else {
    std::cout << "degree set error" << std::endl;
  }
}

double Poly_Implicit::evaluatePolynomial(const std::vector<double> &coeffs,
                                         double x) {
  double result = 0.0;
  int degree = coeffs.size() - 1;
  for (int i = degree; i >= 0; --i) { result += coeffs[i] * std::pow(x, i); }
  return result;
}

double Poly_Implicit::eval2(double x, double y) {
  double a2 = px[1], a1 = px[2], a0 = px[3];
  double b2 = py[1], b1 = py[2], b0 = py[3];

  double out = std::pow(a2, 2) * std::pow(b0, 2) - a1 * a2 * b0 * b1
      + a0 * a2 * std::pow(b1, 2) + std::pow(a1, 2) * b0 * b2
      - 2 * a0 * a2 * b0 * b2 - a0 * a1 * b1 * b2
      + std::pow(a0, 2) * std::pow(b2, 2) - a2 * std::pow(b1, 2) * x
      + 2 * a2 * b0 * b2 * x + a1 * b1 * b2 * x - 2 * a0 * std::pow(b2, 2) * x
      + std::pow(b2, 2) * std::pow(x, 2) - 2 * std::pow(a2, 2) * b0 * y
      + a1 * a2 * b1 * y - std::pow(a1, 2) * b2 * y + 2 * a0 * a2 * b2 * y
      - 2 * a2 * b2 * x * y + std::pow(a2, 2) * std::pow(y, 2);
  return out / scale;
}
double Poly_Implicit::gradx2(double x, double y) {
  double a2 = px[1], a1 = px[2], a0 = px[3];
  double b2 = py[1], b1 = py[2], b0 = py[3];
  double out = -(a2 * std::pow(b1, 2)) + 2 * a2 * b0 * b2 + a1 * b1 * b2
      - 2 * a0 * std::pow(b2, 2) + 2 * std::pow(b2, 2) * x - 2 * a2 * b2 * y;
  return out;
}
double Poly_Implicit::grady2(double x, double y) {
  double a2 = px[1], a1 = px[2], a0 = px[3];
  double b2 = py[1], b1 = py[2], b0 = py[3];
  double out = -2 * std::pow(a2, 2) * b0 + a1 * a2 * b1 - std::pow(a1, 2) * b2
      + 2 * a0 * a2 * b2 - 2 * a2 * b2 * x + 2 * std::pow(a2, 2) * y;
  return out;
}

double Poly_Implicit::eval3(double x, double y) {
  double a3 = px[0], a2 = px[1], a1 = px[2], a0 = px[3];
  double b3 = py[0], b2 = py[1], b1 = py[2], b0 = py[3];

  double out = (b0 - y)
          * (std::pow(a3, 3) * std::pow(b0 - y, 2)
             + b3
                 * (std::pow(a1, 3) * b3
                    + std::pow(a2, 2) * (a1 * b1 + b2 * (a0 - x))
                    - a1 * a2 * (a1 * b2 + 2 * b3 * (a0 - x))
                    + std::pow(a2, 3) * (-b0 + y))
             + std::pow(a3, 2)
                 * (a2 * b1 * (-b0 + y)
                    + a1 * (std::pow(b1, 2) - 2 * b0 * b2 + 2 * b2 * y)
                    + (a0 - x) * (b1 * b2 - 2 * b0 * b3 + 2 * b3 * y))
             + a3
                 * (std::pow(a1, 2) * (std::pow(b2, 2) - 2 * b1 * b3)
                    + a1 * b2 * b3 * (a0 - x)
                    + std::pow(b3, 2) * std::pow(a0 - x, 2)
                    + std::pow(a2, 2) * b2 * (b0 - y)
                    - a2
                        * (a1 * b1 * b2 + std::pow(b2, 2) * (a0 - x)
                           + 3 * a1 * b3 * (-b0 + y))))
      - (a0 - x)
          * (std::pow(a3, 2)
                 * (std::pow(b1, 3) + b3 * std::pow(b0 - y, 2)
                    + 2 * b1 * b2 * (-b0 + y))
             + b3
                 * (a2 * (std::pow(b2, 2) - 2 * b1 * b3) * (a0 - x)
                    + b3
                        * (std::pow(a1, 2) * b1 + b3 * std::pow(a0 - x, 2)
                           + a1 * b2 * (-a0 + x))
                    - a1 * a2 * (b1 * b2 - b0 * b3 + b3 * y)
                    + std::pow(a2, 2) * (std::pow(b1, 2) + b2 * (-b0 + y)))
             + a3
                 * (a1 * b1 * (std::pow(b2, 2) - 2 * b1 * b3)
                    - (a0 - x)
                        * (std::pow(b2, 3) - 3 * b1 * b2 * b3
                           + 2 * std::pow(b3, 2) * (b0 - y))
                    - a2
                        * (std::pow(b1, 2) * b2 + std::pow(b2, 2) * (-b0 + y)
                           + b1 * b3 * (-b0 + y))));
  if (std::abs(scale - 0.0) < 1e-3) { return out; }
  return out / scale;
}

double Poly_Implicit::gradx3(double x, double y) {
  double a3 = px[0], a2 = px[1], a1 = px[2], a0 = px[3];
  double b3 = py[0], b2 = py[1], b1 = py[2], b0 = py[3];
  double out = std::pow(a3, 2)
          * (std::pow(b1, 3) + 3 * b3 * std::pow(b0 - y, 2)
             + 3 * b1 * b2 * (-b0 + y))
      + b3
          * (2 * a2 * (std::pow(b2, 2) - 2 * b1 * b3) * (a0 - x)
             + b3
                 * (std::pow(a1, 2) * b1 + 3 * b3 * std::pow(a0 - x, 2)
                    + 2 * a1 * b2 * (-a0 + x))
             - a1 * a2 * (b1 * b2 - 3 * b0 * b3 + 3 * b3 * y)
             + std::pow(a2, 2) * (std::pow(b1, 2) + 2 * b2 * (-b0 + y)))
      + a3
          * (-2 * (a0 - x)
                 * (std::pow(b2, 3) - 3 * b1 * b2 * b3
                    + 3 * std::pow(b3, 2) * (b0 - y))
             - a2
                 * (std::pow(b1, 2) * b2 + 2 * std::pow(b2, 2) * (-b0 + y)
                    + b1 * b3 * (-b0 + y))
             + a1
                 * (b1 * std::pow(b2, 2) - 2 * std::pow(b1, 2) * b3
                    + b2 * b3 * (-b0 + y)));
  return out;
}

double Poly_Implicit::grady3(double x, double y) {
  double a3 = px[0], a2 = px[1], a1 = px[2], a0 = px[3];
  double b3 = py[0], b2 = py[1], b1 = py[2], b0 = py[3];
  double out = b3
          * (-(std::pow(a1, 3) * b3)
             - std::pow(a2, 2) * (a1 * b1 + 2 * b2 * (a0 - x))
             + a1 * a2 * (a1 * b2 + 3 * b3 * (a0 - x))
             + 2 * std::pow(a2, 3) * (b0 - y))
      - 3 * std::pow(a3, 3) * std::pow(b0 - y, 2)
      + std::pow(a3, 2)
          * (2 * a2 * b1 * (b0 - y)
             - a1 * (std::pow(b1, 2) - 4 * b0 * b2 + 4 * b2 * y)
             - 3 * (a0 - x) * (b1 * b2 - 2 * b0 * b3 + 2 * b3 * y))
      + a3
          * (-(std::pow(a1, 2) * (std::pow(b2, 2) - 2 * b1 * b3))
             + a2 * (2 * std::pow(b2, 2) + b1 * b3) * (a0 - x)
             - 3 * std::pow(b3, 2) * std::pow(a0 - x, 2)
             + a1 * b2 * b3 * (-a0 + x) + 2 * std::pow(a2, 2) * b2 * (-b0 + y)
             + a1 * a2 * (b1 * b2 + 6 * b3 * (-b0 + y)));
  return out;
}
//}  // namespace planning
//}  // namespace hiphi