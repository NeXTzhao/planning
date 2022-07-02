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
  double kappa1_;
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

class xy_error_functor : public ceres::SizedCostFunction<2, 4, 4, 1> {
 public:
  xy_error_functor() = default;
  ~xy_error_functor() override = default;

  // template <typename T>
  // bool operator()(const T *const node0, const T *const node1,
  //                 const T *const dist, T *residual) const {
  bool Evaluate(double const *const *Node, double *residuals,
                double **jacobians) const override {
    // do not use std::sin std::cos std::pow, use sin cos pow instead

    // Node{thetas[i], kappas[i], xs[i], ys[i]};
    auto theta0 = Node[0][0];
    auto kappa0 = Node[0][1];
    auto x0 = Node[0][2];
    auto y0 = Node[0][3];

    auto theta1 = Node[1][0];
    auto kappa1 = Node[1][1];
    auto x1 = Node[1][2];
    auto y1 = Node[1][3];

    auto delta = Node[2][0];

    // std::cout << "theta0,kappa0,x0,y0,delta:::" << theta0 << "," << kappa0
    //           << "," << x0 << "," << y0 << "," << delta << '\n';
    // std::cout << "theta0,kappa0,x0,y0,delta:::" << theta1 << "," << kappa1
    //           << "," << x1 << "," << y1 << "," << delta << '\n';
    // calculate coefficient
    // theta = a + bs + cs^2 + ds^3
    auto a = theta0;
    auto b = kappa0;
    auto c = (-2.0 * kappa0) / delta - kappa1 / delta -
             (3.0 * theta0) / (delta * delta) +
             (3.0 * theta1) / (delta * delta);
    auto d = kappa0 / (delta * delta) + kappa1 / (delta * delta) +
             (2.0 * theta0) / (delta * delta * delta) -
             (2.0 * theta1) / (delta * delta * delta);
    // calculate dx using Curve function

    auto dx = 0.06474248308443546 * delta *
                  cos(a + 0.025446043828620812 * b * delta +
                      0.0006475011465280913 * c * (delta * delta) +
                      0.000016476342553636037 * d * (delta * delta * delta)) +
              0.13985269574463816 * delta *
                  cos(a + 0.12923440720030277 * b * delta +
                      0.01670153200441367 * c * (delta * delta) +
                      0.002158412587927285 * d * (delta * delta * delta)) +
              0.1909150252525593 * delta *
                  cos(a + 0.29707742431130146 * b * delta +
                      0.08825499603543704 * c * (delta * delta) +
                      0.02621856690481176 * d * (delta * delta * delta)) +
              0.20897959183673434 * delta *
                  cos(a + 0.5 * b * delta + 0.25 * c * (delta * delta) +
                      0.125 * d * (delta * delta * delta)) +
              0.1909150252525593 * delta *
                  cos(a + 0.7029225756886985 * b * delta +
                      0.49410014741283415 * c * (delta * delta) +
                      0.347314148267595 * d * (delta * delta * delta)) +
              0.13985269574463816 * delta *
                  cos(a + 0.8707655927996972 * b * delta +
                      0.7582327176038082 * c * (delta * delta) +
                      0.6602429618244054 * d * (delta * delta * delta)) +
              0.06474248308443546 * delta *
                  cos(a + 0.9745539561713792 * b * delta +
                      0.9497554134892865 * c * (delta * delta) +
                      0.9255878956111683 * d * (delta * delta * delta));

    // calculate dy using Curve function

    auto dy =
        delta *
        (0.06474248308443546 *
             sin(a + 0.025446043828620812 * b * delta +
                 0.0006475011465280913 * c * (delta * delta) +
                 0.000016476342553636037 * d * (delta * delta * delta)) +
         0.1909150252525593 *
             sin(a + 0.29707742431130146 * b * delta +
                 0.08825499603543704 * c * (delta * delta) +
                 0.02621856690481176 * d * (delta * delta * delta)) +
         0.20897959183673434 *
             sin(a + 0.5 * b * delta + 0.25 * c * (delta * delta) +
                 0.125 * d * (delta * delta * delta)) +
         0.1909150252525593 *
             sin(a + 0.7029225756886985 * b * delta +
                 0.49410014741283415 * c * (delta * delta) +
                 0.347314148267595 * d * (delta * delta * delta)) +
         0.13985269574463816 *
             sin(a + 0.8707655927996972 * b * delta +
                 0.7582327176038082 * c * (delta * delta) +
                 0.6602429618244054 * d * (delta * delta * delta)) +
         0.06474248308443546 *
             sin(a + 0.9745539561713792 * b * delta +
                 0.9497554134892865 * c * (delta * delta) +
                 0.9255878956111683 * d * (delta * delta * delta)) +
         0.13985269574463816 *
             sin(a + delta * (0.12923440720030277 * b +
                              0.01670153200441367 * c * delta +
                              0.002158412587927285 * d * (delta * delta))));
    residuals[0] = x1 - x0 - dx;
    residuals[1] = y1 - y0 - dy;

    double *jacobian;
    if (!jacobians) return true;

    jacobian = jacobians[0];
    if (!jacobian) return true;

    jacobians[0][0] =
        0.06461885402701487 * delta *
            sin(0.024167517878118265 * delta * kappa0 -
                0.0006310248039744553 * delta * kappa1 +
                0.998090449245523 * theta0 + 0.0019095507544770018 * theta1) +
        0.13344915256089193 * delta *
            sin(0.09798975577940272 * delta * kappa0 -
                0.014543119416486384 * delta * kappa1 +
                0.9542122291626136 * theta0 + 0.045787770837386436 * theta1) +
        0.15037844758768545 * delta *
            sin(0.14678599914523913 * delta * kappa0 -
                0.062036429130625285 * delta * kappa1 +
                0.7876721457033125 * theta0 + 0.2123278542966876 * theta1) +
        0.10448979591836717 * delta *
            sin(0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 +
                0.5 * theta1) +
        0.040536577664873834 * delta *
            sin(0.06203642913062524 * delta * kappa0 -
                0.14678599914523915 * delta * kappa1 +
                0.21232785429668755 * theta0 + 0.7876721457033125 * theta1) +
        0.0064035431837461835 * delta *
            sin(0.014543119416486339 * delta * kappa0 -
                0.09798975577940272 * delta * kappa1 +
                0.045787770837386166 * theta0 + 0.9542122291626138 * theta1) +
        0.00012362905742060293 * delta *
            sin(0.0006310248039744781 * delta * kappa0 -
                0.024167517878118217 * delta * kappa1 +
                0.001909550754477074 * theta0 + 0.9980904492455229 * theta1);
    jacobians[0][1] =
        0.0015646651174168634 * Power(delta, 2) *
            sin(0.024167517878118265 * delta * kappa0 -
                0.0006310248039744553 * delta * kappa1 +
                0.998090449245523 * theta0 + 0.0019095507544770018 * theta1) +
        0.013704131501108207 * Power(delta, 2) *
            sin(0.09798975577940272 * delta * kappa0 -
                0.014543119416486384 * delta * kappa1 +
                0.9542122291626136 * theta0 + 0.045787770837386436 * theta1) +
        0.028023652733535475 * Power(delta, 2) *
            sin(0.14678599914523913 * delta * kappa0 -
                0.062036429130625285 * delta * kappa1 +
                0.7876721457033125 * theta0 + 0.2123278542966876 * theta1) +
        0.026122448979591793 * Power(delta, 2) *
            sin(0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 +
                0.5 * theta1) +
        0.011843686434051922 * Power(delta, 2) *
            sin(0.06203642913062524 * delta * kappa0 -
                0.14678599914523915 * delta * kappa1 +
                0.21232785429668755 * theta0 + 0.7876721457033125 * theta1) +
        0.0020338944549318037 * Power(delta, 2) *
            sin(0.014543119416486339 * delta * kappa0 -
                0.09798975577940272 * delta * kappa1 +
                0.045787770837386166 * theta0 + 0.9542122291626138 * theta1) +
        0.00004085411269717685 * Power(delta, 2) *
            sin(0.0006310248039744781 * delta * kappa0 -
                0.024167517878118217 * delta * kappa1 +
                0.001909550754477074 * theta0 + 0.9980904492455229 * theta1);
    jacobians[0][2] = -1;
    jacobians[0][3] = 0;

    jacobians[0][4] =
        -0.06461885402701487 * delta *
            cos(0.024167517878118265 * delta * kappa0 -
                0.0006310248039744553 * delta * kappa1 +
                0.998090449245523 * theta0 + 0.0019095507544770018 * theta1) -
        0.13344915256089193 * delta *
            cos(0.09798975577940272 * delta * kappa0 -
                0.014543119416486384 * delta * kappa1 +
                0.9542122291626136 * theta0 + 0.045787770837386436 * theta1) -
        0.15037844758768545 * delta *
            cos(0.14678599914523913 * delta * kappa0 -
                0.062036429130625285 * delta * kappa1 +
                0.7876721457033125 * theta0 + 0.2123278542966876 * theta1) -
        0.10448979591836717 * delta *
            cos(0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 +
                0.5 * theta1) -
        0.040536577664873834 * delta *
            cos(0.06203642913062524 * delta * kappa0 -
                0.14678599914523915 * delta * kappa1 +
                0.21232785429668755 * theta0 + 0.7876721457033125 * theta1) -
        0.0064035431837461835 * delta *
            cos(0.014543119416486339 * delta * kappa0 -
                0.09798975577940272 * delta * kappa1 +
                0.045787770837386166 * theta0 + 0.9542122291626138 * theta1) -
        0.00012362905742060293 * delta *
            cos(0.0006310248039744781 * delta * kappa0 -
                0.024167517878118217 * delta * kappa1 +
                0.001909550754477074 * theta0 + 0.9980904492455229 * theta1);
    jacobians[0][5] =
        -0.0015646651174168634 * Power(delta, 2) *
            cos(0.024167517878118265 * delta * kappa0 -
                0.0006310248039744553 * delta * kappa1 +
                0.998090449245523 * theta0 + 0.0019095507544770018 * theta1) -
        0.013704131501108207 * Power(delta, 2) *
            cos(0.09798975577940272 * delta * kappa0 -
                0.014543119416486384 * delta * kappa1 +
                0.9542122291626136 * theta0 + 0.045787770837386436 * theta1) -
        0.028023652733535475 * Power(delta, 2) *
            cos(0.14678599914523913 * delta * kappa0 -
                0.062036429130625285 * delta * kappa1 +
                0.7876721457033125 * theta0 + 0.2123278542966876 * theta1) -
        0.026122448979591793 * Power(delta, 2) *
            cos(0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 +
                0.5 * theta1) -
        0.011843686434051922 * Power(delta, 2) *
            cos(0.06203642913062524 * delta * kappa0 -
                0.14678599914523915 * delta * kappa1 +
                0.21232785429668755 * theta0 + 0.7876721457033125 * theta1) -
        0.0020338944549318037 * Power(delta, 2) *
            cos(0.014543119416486339 * delta * kappa0 -
                0.09798975577940272 * delta * kappa1 +
                0.045787770837386166 * theta0 + 0.9542122291626138 * theta1) -
        0.00004085411269717685 * Power(delta, 2) *
            cos(0.0006310248039744781 * delta * kappa0 -
                0.024167517878118217 * delta * kappa1 +
                0.001909550754477074 * theta0 + 0.9980904492455229 * theta1);
    jacobians[0][6] = 0;
    jacobians[0][7] = -1;

    jacobian = jacobians[1];
    if (!jacobian) return true;

    jacobians[1][0] =
        0.00012362905742059827 * delta *
            sin(0.024167517878118265 * delta * kappa0 -
                0.0006310248039744553 * delta * kappa1 +
                0.998090449245523 * theta0 + 0.0019095507544770018 * theta1) +
        0.006403543183746221 * delta *
            sin(0.09798975577940272 * delta * kappa0 -
                0.014543119416486384 * delta * kappa1 +
                0.9542122291626136 * theta0 + 0.045787770837386436 * theta1) +
        0.04053657766487384 * delta *
            sin(0.14678599914523913 * delta * kappa0 -
                0.062036429130625285 * delta * kappa1 +
                0.7876721457033125 * theta0 + 0.2123278542966876 * theta1) +
        0.10448979591836717 * delta *
            sin(0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 +
                0.5 * theta1) +
        0.15037844758768545 * delta *
            sin(0.06203642913062524 * delta * kappa0 -
                0.14678599914523915 * delta * kappa1 +
                0.21232785429668755 * theta0 + 0.7876721457033125 * theta1) +
        0.13344915256089196 * delta *
            sin(0.014543119416486339 * delta * kappa0 -
                0.09798975577940272 * delta * kappa1 +
                0.045787770837386166 * theta0 + 0.9542122291626138 * theta1) +
        0.06461885402701487 * delta *
            sin(0.0006310248039744781 * delta * kappa0 -
                0.024167517878118217 * delta * kappa1 +
                0.001909550754477074 * theta0 + 0.9980904492455229 * theta1);
    jacobians[1][1] =
        -0.000040854112697175375 * Power(delta, 2) *
            sin(0.024167517878118265 * delta * kappa0 -
                0.0006310248039744553 * delta * kappa1 +
                0.998090449245523 * theta0 + 0.0019095507544770018 * theta1) -
        0.0020338944549318097 * Power(delta, 2) *
            sin(0.09798975577940272 * delta * kappa0 -
                0.014543119416486384 * delta * kappa1 +
                0.9542122291626136 * theta0 + 0.045787770837386436 * theta1) -
        0.01184368643405193 * Power(delta, 2) *
            sin(0.14678599914523913 * delta * kappa0 -
                0.062036429130625285 * delta * kappa1 +
                0.7876721457033125 * theta0 + 0.2123278542966876 * theta1) -
        0.026122448979591793 * Power(delta, 2) *
            sin(0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 +
                0.5 * theta1) -
        0.028023652733535482 * Power(delta, 2) *
            sin(0.06203642913062524 * delta * kappa0 -
                0.14678599914523915 * delta * kappa1 +
                0.21232785429668755 * theta0 + 0.7876721457033125 * theta1) -
        0.013704131501108207 * Power(delta, 2) *
            sin(0.014543119416486339 * delta * kappa0 -
                0.09798975577940272 * delta * kappa1 +
                0.045787770837386166 * theta0 + 0.9542122291626138 * theta1) -
        0.0015646651174168603 * Power(delta, 2) *
            sin(0.0006310248039744781 * delta * kappa0 -
                0.024167517878118217 * delta * kappa1 +
                0.001909550754477074 * theta0 + 0.9980904492455229 * theta1);

    jacobians[1][2] = 1;
    jacobians[1][3] = 0;
    jacobians[1][4] =
        -0.00012362905742059827 * delta *
            cos(0.024167517878118265 * delta * kappa0 -
                0.0006310248039744553 * delta * kappa1 +
                0.998090449245523 * theta0 + 0.0019095507544770018 * theta1) -
        0.006403543183746221 * delta *
            cos(0.09798975577940272 * delta * kappa0 -
                0.014543119416486384 * delta * kappa1 +
                0.9542122291626136 * theta0 + 0.045787770837386436 * theta1) -
        0.04053657766487384 * delta *
            cos(0.14678599914523913 * delta * kappa0 -
                0.062036429130625285 * delta * kappa1 +
                0.7876721457033125 * theta0 + 0.2123278542966876 * theta1) -
        0.10448979591836717 * delta *
            cos(0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 +
                0.5 * theta1) -
        0.15037844758768545 * delta *
            cos(0.06203642913062524 * delta * kappa0 -
                0.14678599914523915 * delta * kappa1 +
                0.21232785429668755 * theta0 + 0.7876721457033125 * theta1) -
        0.13344915256089196 * delta *
            cos(0.014543119416486339 * delta * kappa0 -
                0.09798975577940272 * delta * kappa1 +
                0.045787770837386166 * theta0 + 0.9542122291626138 * theta1) -
        0.06461885402701487 * delta *
            cos(0.0006310248039744781 * delta * kappa0 -
                0.024167517878118217 * delta * kappa1 +
                0.001909550754477074 * theta0 + 0.9980904492455229 * theta1);
    jacobians[1][5] =
        0.000040854112697175375 * Power(delta, 2) *
            cos(0.024167517878118265 * delta * kappa0 -
                0.0006310248039744553 * delta * kappa1 +
                0.998090449245523 * theta0 + 0.0019095507544770018 * theta1) +
        0.0020338944549318097 * Power(delta, 2) *
            cos(0.09798975577940272 * delta * kappa0 -
                0.014543119416486384 * delta * kappa1 +
                0.9542122291626136 * theta0 + 0.045787770837386436 * theta1) +
        0.01184368643405193 * Power(delta, 2) *
            cos(0.14678599914523913 * delta * kappa0 -
                0.062036429130625285 * delta * kappa1 +
                0.7876721457033125 * theta0 + 0.2123278542966876 * theta1) +
        0.026122448979591793 * Power(delta, 2) *
            cos(0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 +
                0.5 * theta1) +
        0.028023652733535482 * Power(delta, 2) *
            cos(0.06203642913062524 * delta * kappa0 -
                0.14678599914523915 * delta * kappa1 +
                0.21232785429668755 * theta0 + 0.7876721457033125 * theta1) +
        0.013704131501108207 * Power(delta, 2) *
            cos(0.014543119416486339 * delta * kappa0 -
                0.09798975577940272 * delta * kappa1 +
                0.045787770837386166 * theta0 + 0.9542122291626138 * theta1) +
        0.0015646651174168603 * Power(delta, 2) *
            cos(0.0006310248039744781 * delta * kappa0 -
                0.024167517878118217 * delta * kappa1 +
                0.001909550754477074 * theta0 + 0.9980904492455229 * theta1);
    jacobians[1][6] = 0;
    jacobians[1][7] = 1;

    jacobian = jacobians[2];
    if (!jacobian) return true;

    jacobians[2][0] =
        -0.06474248308443546 *
            cos(0.024167517878118265 * delta * kappa0 -
                0.0006310248039744553 * delta * kappa1 +
                0.998090449245523 * theta0 + 0.0019095507544770018 * theta1) -
        0.13985269574463816 *
            cos(0.09798975577940272 * delta * kappa0 -
                0.014543119416486384 * delta * kappa1 +
                0.9542122291626136 * theta0 + 0.045787770837386436 * theta1) -
        0.1909150252525593 *
            cos(0.14678599914523913 * delta * kappa0 -
                0.062036429130625285 * delta * kappa1 +
                0.7876721457033125 * theta0 + 0.2123278542966876 * theta1) -
        0.20897959183673434 *
            cos(0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 +
                0.5 * theta1) -
        0.1909150252525593 *
            cos(0.06203642913062524 * delta * kappa0 -
                0.14678599914523915 * delta * kappa1 +
                0.21232785429668755 * theta0 + 0.7876721457033125 * theta1) -
        0.13985269574463816 *
            cos(0.014543119416486339 * delta * kappa0 -
                0.09798975577940272 * delta * kappa1 +
                0.045787770837386166 * theta0 + 0.9542122291626138 * theta1) -
        0.06474248308443546 *
            cos(0.0006310248039744781 * delta * kappa0 -
                0.024167517878118217 * delta * kappa1 +
                0.001909550754477074 * theta0 + 0.9980904492455229 * theta1) +
        0.06474248308443546 * delta *
            (0.024167517878118265 * kappa0 - 0.0006310248039744553 * kappa1) *
            sin(0.024167517878118265 * delta * kappa0 -
                0.0006310248039744553 * delta * kappa1 +
                0.998090449245523 * theta0 + 0.0019095507544770018 * theta1) +
        0.13985269574463816 * delta *
            (0.09798975577940272 * kappa0 - 0.014543119416486384 * kappa1) *
            sin(0.09798975577940272 * delta * kappa0 -
                0.014543119416486384 * delta * kappa1 +
                0.9542122291626136 * theta0 + 0.045787770837386436 * theta1) +
        0.1909150252525593 * delta *
            (0.14678599914523913 * kappa0 - 0.062036429130625285 * kappa1) *
            sin(0.14678599914523913 * delta * kappa0 -
                0.062036429130625285 * delta * kappa1 +
                0.7876721457033125 * theta0 + 0.2123278542966876 * theta1) +
        0.20897959183673434 * delta * (0.125 * kappa0 - 0.125 * kappa1) *
            sin(0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 +
                0.5 * theta1) +
        0.1909150252525593 * delta *
            (0.06203642913062524 * kappa0 - 0.14678599914523915 * kappa1) *
            sin(0.06203642913062524 * delta * kappa0 -
                0.14678599914523915 * delta * kappa1 +
                0.21232785429668755 * theta0 + 0.7876721457033125 * theta1) +
        0.13985269574463816 * delta *
            (0.014543119416486339 * kappa0 - 0.09798975577940272 * kappa1) *
            sin(0.014543119416486339 * delta * kappa0 -
                0.09798975577940272 * delta * kappa1 +
                0.045787770837386166 * theta0 + 0.9542122291626138 * theta1) +
        0.06474248308443546 * delta *
            (0.0006310248039744781 * kappa0 - 0.024167517878118217 * kappa1) *
            sin(0.0006310248039744781 * delta * kappa0 -
                0.024167517878118217 * delta * kappa1 +
                0.001909550754477074 * theta0 + 0.9980904492455229 * theta1);

    jacobians[2][1] =
        -0.06474248308443546 * delta *
            (0.024167517878118265 * kappa0 - 0.0006310248039744553 * kappa1) *
            cos(0.024167517878118265 * delta * kappa0 -
                0.0006310248039744553 * delta * kappa1 +
                0.998090449245523 * theta0 + 0.0019095507544770018 * theta1) -
        0.13985269574463816 * delta *
            (0.09798975577940272 * kappa0 - 0.014543119416486384 * kappa1) *
            cos(0.09798975577940272 * delta * kappa0 -
                0.014543119416486384 * delta * kappa1 +
                0.9542122291626136 * theta0 + 0.045787770837386436 * theta1) -
        0.1909150252525593 * delta *
            (0.14678599914523913 * kappa0 - 0.062036429130625285 * kappa1) *
            cos(0.14678599914523913 * delta * kappa0 -
                0.062036429130625285 * delta * kappa1 +
                0.7876721457033125 * theta0 + 0.2123278542966876 * theta1) -
        0.20897959183673434 * delta * (0.125 * kappa0 - 0.125 * kappa1) *
            cos(0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 +
                0.5 * theta1) -
        0.1909150252525593 * delta *
            (0.06203642913062524 * kappa0 - 0.14678599914523915 * kappa1) *
            cos(0.06203642913062524 * delta * kappa0 -
                0.14678599914523915 * delta * kappa1 +
                0.21232785429668755 * theta0 + 0.7876721457033125 * theta1) -
        0.13985269574463816 * delta *
            (0.014543119416486339 * kappa0 - 0.09798975577940272 * kappa1) *
            cos(0.014543119416486339 * delta * kappa0 -
                0.09798975577940272 * delta * kappa1 +
                0.045787770837386166 * theta0 + 0.9542122291626138 * theta1) -
        0.06474248308443546 * delta *
            (0.0006310248039744781 * kappa0 - 0.024167517878118217 * kappa1) *
            cos(0.0006310248039744781 * delta * kappa0 -
                0.024167517878118217 * delta * kappa1 +
                0.001909550754477074 * theta0 + 0.9980904492455229 * theta1) -
        0.06474248308443546 *
            sin(0.024167517878118265 * delta * kappa0 -
                0.0006310248039744553 * delta * kappa1 +
                0.998090449245523 * theta0 + 0.0019095507544770018 * theta1) -
        0.13985269574463816 *
            sin(0.09798975577940272 * delta * kappa0 -
                0.014543119416486384 * delta * kappa1 +
                0.9542122291626136 * theta0 + 0.045787770837386436 * theta1) -
        0.1909150252525593 *
            sin(0.14678599914523913 * delta * kappa0 -
                0.062036429130625285 * delta * kappa1 +
                0.7876721457033125 * theta0 + 0.2123278542966876 * theta1) -
        0.20897959183673434 *
            sin(0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 +
                0.5 * theta1) -
        0.1909150252525593 *
            sin(0.06203642913062524 * delta * kappa0 -
                0.14678599914523915 * delta * kappa1 +
                0.21232785429668755 * theta0 + 0.7876721457033125 * theta1) -
        0.13985269574463816 *
            sin(0.014543119416486339 * delta * kappa0 -
                0.09798975577940272 * delta * kappa1 +
                0.045787770837386166 * theta0 + 0.9542122291626138 * theta1) -
        0.06474248308443546 *
            sin(0.0006310248039744781 * delta * kappa0 -
                0.024167517878118217 * delta * kappa1 +
                0.001909550754477074 * theta0 + 0.9980904492455229 * theta1);

    return true;
  }
};  // namespace reference_line
/*
struct objective_functor {
  objective_functor() = default;

  template <typename T>
  bool operator()(const T *const node0, const T *const node1,
                  const T *const dist, T *residual) const {
    T obj_value;
    // do not use std::sin std::cos std::pow, use sin cos pow instead
    T theta0 = node0[0];
    T kappa0 = node0[1];
    T theta1 = node1[0];
    T kappa1 = node1[1];
    T delta = dist[0];
    obj_value += delta * 1.0;
    //  theta = a + bs + cs^2 + ds^3
    //  kappa = b + 2cs + 3ds^2
    //  dkappa = 2c + 6ds
    T b = kappa0;
    T c = (-2.0 * kappa0) / delta - kappa1 / delta -
          (3.0 * theta0) / (delta * delta) + (3.0 * theta1) / (delta * delta);
    T d = kappa0 / (delta * delta) + kappa1 / (delta * delta) +
          (2.0 * theta0) / (delta * delta * delta) -
          (2.0 * theta1) / (delta * delta * delta);
/*
    int num_of_internal_points_ = 7;
    for (int j = 0; j < num_of_internal_points_; ++j) {
      double ratio =
          static_cast<double>(j) / static_cast<double>(num_of_internal_points_);
      T delta_s = ratio * delta;

      T kappa = b + 2.0 * c * delta_s + 3.0 * d * delta_s * delta_s;
      obj_value += kappa * kappa * 1.0;

      T dkappa = 2.0 * c + 6.0 * d * delta_s;
      obj_value += dkappa * dkappa * 100.0;
    }

    // T kappa = b + 2.0 * c * delta + 3.0 * d * delta * delta;
    // obj_value += kappa * kappa * 1.0;

    // T dkappa = 2.0 * c + 6.0 * d * delta;
    // obj_value += dkappa * dkappa * 1.0;

    residual[0] = obj_value;

    return true;
  }
};

struct objective_functor {
  objective_functor() = default;

  template <typename T>
  bool operator()(const T *const dist, T *residual) const {
    // do not use std::sin std::cos std::pow, use sin cos pow instead
    T delta = dist[0];

    residual[0] = delta * 10.0;

    return true;
  }
};

struct connection_point_functor {
  connection_point_functor() = default;

  template <typename T>
  bool operator()(const T *const node0, const T *const node1,
                  const T *const node2, const T *const dist,
                  T *residual) const {
    // do not use std::sin std::cos std::pow, use sin cos pow instead
    T theta0 = node0[0];
    T kappa0 = node0[1];
    T dkappa0 = node0[2];

    T theta1 = node1[0];
    T kappa1 = node1[1];
    T dkappa1 = node1[2];

    T delta = dist[0];

    T theta2 = node2[0];
    T kappa2 = node2[1];
    T delta1 = dist[1];

    // calculate coefficient
    // theta = a + bs + cs^2 + ds^3
    // kappa = b + 2cs + 3ds^2
    // dkappa = 2c + 6ds
    T a = theta0;
    T b = kappa0;
    T c = (-2.0 * kappa0) / delta - kappa1 / delta -
          (3.0 * theta0) / (delta * delta) + (3.0 * theta1) / (delta * delta);
    T d = kappa0 / (delta * delta) + kappa1 / (delta * delta) +
          (2.0 * theta0) / (delta * delta * delta) -
          (2.0 * theta1) / (delta * delta * delta);

    T a1 = theta1;
    T b1 = kappa1;
    T c1 = (-2.0 * kappa1) / delta1 - kappa2 / delta1 -
           (3.0 * theta1) / (delta1 * delta1) +
           (3.0 * theta2) / (delta1 * delta1);
    T d1 = kappa1 / (delta1 * delta1) + kappa2 / (delta1 * delta1) +
           (2.0 * theta1) / (delta1 * delta1 * delta1) -
           (2.0 * theta2) / (delta1 * delta1 * delta1);

    T cost_theta0 =
        a + b * delta + c * delta * delta + d * delta * delta * delta;

    T cost_theta1 =
        a1 + b1 * delta1 + c1 * delta1 * delta1 + d1 * delta1 * delta1 * delta1;
    // kappa = b + 2cs + 3ds^2
    T cost_kappa0 = b + 2.0 * c * delta + 3.0 * d * delta * delta;
    // dkappa = 2c + 6ds
    T cost_dkappa0 = 2.0 * c + 6.0 * d * delta;

    residual[0] = 1.0 * (theta1 - cost_theta0);
    residual[1] = 10.0 * (kappa1 - cost_kappa0);
    residual[2] = 100.0 * (dkappa1 - cost_dkappa0);
    return true;
  }
};*/

}  // namespace reference_line
