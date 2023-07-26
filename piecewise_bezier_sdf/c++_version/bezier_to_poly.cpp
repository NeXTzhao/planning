#include "bezier_to_poly.h"
#include <Eigen/Dense>
#include <cmath>
#include <numeric>

Bezier2Poly::Bezier2Poly(const std::vector<Point> &controlPoints)
    : controlPoints_(controlPoints) {
  tValues_ = linspace(0.0, 1.0, 50);
  calculateBasisFunctions();
  calculatePolynomial();
  calculateGradients();
}

std::vector<double> Bezier2Poly::getXCoefficients() const {
  return poly_coeffs_x_;
}

std::vector<double> Bezier2Poly::getYCoefficients() const {
  return poly_coeffs_y_;
}

std::vector<Point> Bezier2Poly::getPolyCurvePoints() const {
  std::vector<Point> curvePoints;
  for (double t : tValues_) {
    Point point{};
    point.x = polyFuncX_(t);
    point.y = polyFuncY_(t);
    printf("t=%f, x = %f, y = %f\n", t, point.x, point.y);

    curvePoints.push_back(point);
  }
  return curvePoints;
}

double Bezier2Poly::getScale() const {
  return scale_;
}

void Bezier2Poly::calculateBasisFunctions() {
  B0_.resize(tValues_.size());
  B1_.resize(tValues_.size());
  B2_.resize(tValues_.size());
  B3_.resize(tValues_.size());

  for (size_t i = 0; i < tValues_.size(); ++i) {
    double t = tValues_[i];
    B0_[i] = std::pow(1 - t, 3);
    B1_[i] = 3 * std::pow(1 - t, 2) * t;
    B2_[i] = 3 * (1 - t) * std::pow(t, 2);
    B3_[i] = std::pow(t, 3);
  }
}

void Bezier2Poly::calculatePolynomial() {
  std::vector<double> polyX, polyY;

  Point P0 = controlPoints_[0];
  Point P1 = controlPoints_[1];
  Point P2 = controlPoints_[2];
  Point P3 = controlPoints_[3];

  for (size_t i = 0; i < tValues_.size(); ++i) {
    polyX.push_back(B0_[i] * P0.x + B1_[i] * P1.x + B2_[i] * P2.x + B3_[i] * P3.x);
    polyY.push_back(B0_[i] * P0.y + B1_[i] * P1.y + B2_[i] * P2.y + B3_[i] * P3.y);
  }

  poly_coeffs_x_ = polynomialFit(tValues_, polyX, 3);
  poly_coeffs_y_ = polynomialFit(tValues_, polyY, 3);

  polyFuncX_ = polynomialFunction(poly_coeffs_x_);
  polyFuncY_ = polynomialFunction(poly_coeffs_y_);
}

void Bezier2Poly::calculateGradients() {
  xGrad_ = gradient(polyFuncX_);
  yGrad_ = gradient(polyFuncY_);
  scale_ = 1.0 / std::hypot(xGrad_, yGrad_);
}

std::vector<double> Bezier2Poly::linspace(double start, double end, size_t numPoints) {
  std::vector<double> result(numPoints);
  double step = (end - start) / (numPoints - 1);
  std::iota(result.begin(), result.end(), 0);
  std::transform(result.begin(), result.end(), result.begin(), [start, step](double index) {
    return start + index * step;
  });
  return result;
}

std::vector<double> Bezier2Poly::polynomialFit(const std::vector<double> &x, const std::vector<double> &y, int order) {
  // 使用Eigen库进行多项式拟合
  Eigen::MatrixXd A(x.size(), order + 1);
  Eigen::VectorXd B(x.size());
  for (size_t i = 0; i < x.size(); ++i) {
    for (int j = 0; j <= order; ++j) {
      A(i, j) = std::pow(x[i], j);
    }
    B(i) = y[i];
  }

  Eigen::VectorXd coeffs = A.colPivHouseholderQr().solve(B);
  std::vector<double> coefficients(order + 1);
  for (int i = 0; i <= order; ++i) {
    coefficients[i] = coeffs(i);
  }
//  std::reverse(coefficients.begin(), coefficients.end());
  return coefficients;
}

std::function<double(double)> Bezier2Poly::polynomialFunction(const std::vector<double> &coeffs) {
  return [&coeffs](double x) {
    double result = 0.0;
    double powX = 1.0;
    for (double coeff : coeffs) {
      result += coeff * powX;
      powX *= x;
    }
    return result;
  };
}

double Bezier2Poly::gradient(const std::function<double(double)> &func) {
  // 使用数值方法计算梯度
  const double delta = 1e-6;
  return (func(1 + delta) - func(1 - delta)) / (2 * delta);
}
