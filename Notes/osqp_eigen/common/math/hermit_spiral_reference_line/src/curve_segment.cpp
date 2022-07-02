#include "curve_segment.h"

#include <cstdio>

namespace reference_line {
CurveSegment::CurveSegment(double theta0, double kappa0,double dkappa0, double x0, double y0,
                           double theta1, double kappa1, double dkappa1,double x1, double y1,
                           double delta)
    : theta0_(theta0),
      kappa0_(kappa0),
      dkappa0_(dkappa0),
      x0_(x0),
      y0_(y0),
      theta1_(theta1),
      dkappa1_(dkappa1),
      x1_(x1),
      y1_(y1),
      delta_(delta) {
  a_ = theta0;
  b_ = kappa0;
  c_ = (-2 * kappa0) / delta - kappa1 / delta - (3 * theta0) / Power(delta, 2) +
       (3 * theta1) / Power(delta, 2);
  d_ = kappa0 / Power(delta, 2) + kappa1 / Power(delta, 2) +
       (2 * theta0) / Power(delta, 3) - (2 * theta1) / Power(delta, 3);
}

double CurveSegment::theta(double s) const {
  return a_ + b_ * s + c_ * s * s + d_ * s * s * s;
}

double CurveSegment::kappa(double s) const {
  return b_ + 2 * c_ * s + 3 * d_ * s * s;
}

double CurveSegment::dkappa(double s) const { return 2 * c_ + 6 * d_ * s; }

double CurveSegment::x(double s) const {
  auto dx =
      s *
      (0.06474248308443546 * Cos(a_ + 0.025446043828620812 * b_ * s +
                                 0.0006475011465280913 * c_ * Power(s, 2) +
                                 0.000016476342553636037 * d_ * Power(s, 3)) +
       0.1909150252525593 * Cos(a_ + 0.29707742431130146 * b_ * s +
                                0.08825499603543704 * c_ * Power(s, 2) +
                                0.02621856690481176 * d_ * Power(s, 3)) +
       0.20897959183673434 * Cos(a_ + 0.5 * b_ * s + 0.25 * c_ * Power(s, 2) +
                                 0.125 * d_ * Power(s, 3)) +
       0.1909150252525593 * Cos(a_ + 0.7029225756886985 * b_ * s +
                                0.49410014741283415 * c_ * Power(s, 2) +
                                0.347314148267595 * d_ * Power(s, 3)) +
       0.13985269574463816 * Cos(a_ + 0.8707655927996972 * b_ * s +
                                 0.7582327176038082 * c_ * Power(s, 2) +
                                 0.6602429618244054 * d_ * Power(s, 3)) +
       0.06474248308443546 * Cos(a_ + 0.9745539561713792 * b_ * s +
                                 0.9497554134892865 * c_ * Power(s, 2) +
                                 0.9255878956111683 * d_ * Power(s, 3)) +
       0.13985269574463816 *
           Cos(a_ +
               s * (0.12923440720030277 * b_ + 0.01670153200441367 * c_ * s +
                    0.002158412587927285 * d_ * Power(s, 2))));
  return x0_ + dx;
}

double CurveSegment::y(double s) const {
  auto dy =
      s *
      (0.06474248308443546 * Sin(a_ + 0.025446043828620812 * b_ * s +
                                 0.0006475011465280913 * c_ * Power(s, 2) +
                                 0.000016476342553636037 * d_ * Power(s, 3)) +
       0.1909150252525593 * Sin(a_ + 0.29707742431130146 * b_ * s +
                                0.08825499603543704 * c_ * Power(s, 2) +
                                0.02621856690481176 * d_ * Power(s, 3)) +
       0.20897959183673434 * Sin(a_ + 0.5 * b_ * s + 0.25 * c_ * Power(s, 2) +
                                 0.125 * d_ * Power(s, 3)) +
       0.1909150252525593 * Sin(a_ + 0.7029225756886985 * b_ * s +
                                0.49410014741283415 * c_ * Power(s, 2) +
                                0.347314148267595 * d_ * Power(s, 3)) +
       0.13985269574463816 * Sin(a_ + 0.8707655927996972 * b_ * s +
                                 0.7582327176038082 * c_ * Power(s, 2) +
                                 0.6602429618244054 * d_ * Power(s, 3)) +
       0.06474248308443546 * Sin(a_ + 0.9745539561713792 * b_ * s +
                                 0.9497554134892865 * c_ * Power(s, 2) +
                                 0.9255878956111683 * d_ * Power(s, 3)) +
       0.13985269574463816 *
           Sin(a_ +
               s * (0.12923440720030277 * b_ + 0.01670153200441367 * c_ * s +
                    0.002158412587927285 * d_ * Power(s, 2))));
  return y0_ + dy;
}
}  // namespace reference_line