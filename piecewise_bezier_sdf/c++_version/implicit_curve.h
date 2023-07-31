#pragma once

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

//#include "modules/planning/reference_line/reference_line.h"

//namespace hiphi {
//namespace planning {

class PolyCurveFit {
 private:
  std::vector<double> sdata_;
  std::vector<double> xdata_;
  std::vector<double> ydata_;
  std::vector<double> x_coeffs_;
  std::vector<double> y_coeffs_;
  std::vector<double> reserve_x_coeffs_;
  std::vector<double> reserve_y_coeffs_;

 public:
  std::vector<double> fit_x_data;
  std::vector<double> fit_y_data;

 private:
  static std::vector<double> cal_coffs(const std::vector<double> &x,
                                       const std::vector<double> &y, int degree);

  double cal_fit_xdata(double x) const;

  double cal_fit_ydata(double y) const;

  //  void setXYSData(const ReferenceLine &ref_line);

 public:
  PolyCurveFit(const std::vector<double> &sdata, const std::vector<double> &xdata,
               const std::vector<double> &ydata, int degree);
  //  CurveFit(const ReferenceLine &ref_line, int degree);

  std::vector<double> getXFitcoffs() const { return reserve_x_coeffs_; }

  std::vector<double> getYFitcoffs() const { return reserve_y_coeffs_; }

  void getXYFitData(std::vector<double> &x, std::vector<double> &y);
  void getXYFitData();
};

class Poly_Implicit {
 public:
  std::vector<double> px;
  std::vector<double> py;
  double scale{};

  static double evaluatePolynomial(const std::vector<double> &coeffs, double x);

 public:
  Poly_Implicit() = default;
  Poly_Implicit(const std::vector<double> &px, const std::vector<double> &py, int degree);

  double getDistance(double x, double y) { return std::abs(eval3(x, y)); }
  double eval2(double x, double y);
  double gradx2(double x, double y);
  double grady2(double x, double y);

  double eval3(double x, double y);
  double gradx3(double x, double y);
  double grady3(double x, double y);
};
//}  // namespace planning
//}  // namespace hiphi