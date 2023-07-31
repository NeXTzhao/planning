//
// Created by vtd on 23-7-25.
//

#pragma once

#include <functional>
#include <vector>
struct Point {
  double x;
  double y;
};
class Bezier2Poly {
 public:
  explicit Bezier2Poly(const std::array<Point, 4> &controlPoints, int degree);
  explicit Bezier2Poly(const std::vector<Point> &controlPoints, int degree);


  std::vector<double> getXCoefficients() const;
  std::vector<double> getYCoefficients() const;
  std::vector<Point> getPolyCurvePoints() const;
  double getScale() const;

 private:
  std::array<Point, 4> controlPoints4_;
  std::vector<Point> controlPoints3_;
  int degree_;

  std::vector<double> tValues_;
  std::vector<double> B0_, B1_, B2_, B3_;
  std::vector<double> C0_, C1_, C2_;

//  std::vector<std::vector<double>> B;
  std::vector<double> poly_coeffs_x_, poly_coeffs_y_;
  std::function<double(double)> polyFuncX_, polyFuncY_;
  double xGrad_{}, yGrad_{}, scale_{};

  void calculateBasisFunctions();
  void calculatePolynomial();
  void calculateGradients();

  std::vector<double> linspace(double start, double end, size_t numPoints);
  static std::vector<double> polynomialFit(const std::vector<double> &x, const std::vector<double> &y, int order);
  static std::function<double(double)> polynomialFunction(const std::vector<double> &coeffs);
  double gradient(const std::function<double(double)> &func);
};