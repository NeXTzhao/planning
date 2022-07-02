#pragma once

#include <ceres/jet.h>

#include <cmath>
// #include "angle.h"
//#include "sin_cos_table.h"
#include "ceres/ceres.h"

#define Power pow

namespace reference_line {
struct CurveSegment {
  double theta0_;
  double kappa0_;
  double dkappa0_;
  double x0_;
  double y0_;
  double theta1_;
  double kappa1_{};
  double dkappa1_;
  double x1_;
  double y1_;
  double delta_;

  double a_, b_, c_, d_;

  /**
   * ctor of Curve segment
   * @param theta0 theta at start
   * @param kappa0 kappa at start
   * @param x0 x at start
   * @param y0 y at start
   * @param theta1 theta at end
   * @param kappa1 kappa at end
   * @param x1 x at end
   * @param y1 y at end
   * @param delta delta of Curve segment
   */
  CurveSegment(double theta0, double kappa0, double dkappa0, double x0,
               double y0, double theta1, double kappa1, double dkappa1,
               double x1, double y1, double delta);

  /**
   * evaluate theta at s
   * @param s the Curve length
   * @return theta at s
   */
  double theta(double s) const;

  /**
   * evaluate kappa at s
   * @param s the Curve length
   * @return the kappa at s
   */
  double kappa(double s) const;

  /**
   * evaluate dkappa at s
   * @param s the Curve length
   * @return the dkappa at s
   */
  double dkappa(double s) const;

  /**
   * evaluate x at s
   * @param s the Curve length
   * @return the x at s
   */
  double x(double s) const;

  /**
   * evaluate y at s
   * @param s the Curve length
   * @return the y at s
   */
  double y(double s) const;
};
}  // namespace reference_line
