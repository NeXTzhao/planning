#include <iostream>
#include <string>

#include "matplotlibcpp.h"
#include "quintic_polynomial_curve1d.h"

using namespace apollo;
using namespace planning;

void quintic_curve() {
  double x0 = 0.0;
  double dx0 = 1.0;
  double ddx0 = 0.8;

  double x1 = 10.0;
  double dx1 = 5.0;
  double ddx1 = 0.0;

  double t = 8.0;

  QuinticPolynomialCurve1d curve(x0, dx0, ddx0, x1, dx1, ddx1, t);
  auto e_t = curve.ParamLength();
  printf("Length=%f\n", e_t);

//   QuinticPolynomialCurve1d quintic_curve;
//   for (double value = 0.0; value < 4.1; value += 0.1) {
//     quintic_curve.Evaluate(1, value);

//     quintic_curve.Evaluate(2, value);

//     quintic_curve.Evaluate(3, value);
//     quintic_curve.Evaluate(4, value);

//   }
}

int main() {
  quintic_curve();
  return 0;
}