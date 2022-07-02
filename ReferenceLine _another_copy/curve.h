#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "curve_segment.h"

namespace reference_line {
class Curve {
 public:
  std::vector<double> xs_;
  std::vector<double> ys_;
  std::vector<double> thetas_;
  std::vector<double> kappas_;
  std::vector<double> dkappas_;
  std::vector<double> deltas_;
  std::vector<double> acc_s_;
  std::vector<CurveSegment> segments_;

  inline void get_index(double s, int &index, double &ds) const;

 public:
  Curve() = default;

  Curve(const std::vector<double> &xs, const std::vector<double> &ys);

  Curve(const std::vector<double> &thetas, const std::vector<double> &kappas,
        const std::vector<double> &dkappas, const std::vector<double> &xs,
        const std::vector<double> &ys, const std::vector<double> &deltas);

  const std::vector<double> &xs() const { return xs_; }

  const std::vector<double> &ys() const { return ys_; }

  const std::vector<double> &thetas() const { return thetas_; }

  const std::vector<double> &kappas() const { return kappas_; }

  const std::vector<double> &dkappas() const { return dkappas_; }

  const std::vector<double> &deltas() const { return deltas_; }

  const std::vector<CurveSegment> &segments() const { return segments_; }

  double length() const { return acc_s_.back(); }

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
/*
struct objective_functor {
  std::shared_ptr<Curve> curve_;

  explicit objective_functor(std::shared_ptr<Curve> curve)
      : curve_(std::move(curve)){};

  template <typename T>
  bool operator()(const T *const dist, T *residual) const {
    T obj_value;
    for (size_t i = 0; i < curve_->xs().size() - 1; ++i) {
      const auto &spiral_curve = curve_->segments().at(i);
      auto theta0 = spiral_curve.theta0_;
      auto kappa0 = spiral_curve.kappa0_;
      auto theta1 = spiral_curve.theta1_;
      auto kappa1 = spiral_curve.kappa1_;
      // std::cout << "theat0,theat1:" << theta0 << ',' << theta1 << '\n';

      auto delta = dist[0];

      // auto delta = spiral_curve.delta_;
      // std::cout << "curve_->xs().size(),,,curve_->segments(),,,dist:\n"
      //           << curve_->xs().size() << ',' << curve_->segments().size()
      //           << '\n';
      obj_value += delta * 1.0;

      // calculate coefficient
      // theta = a + bs + cs^2 + ds^3
      // kappa = b + 2cs + 3ds^2
      // dkappa = 2c + 6ds
      // T a = theta0;
      auto b = kappa0;
      auto c = (-2.0 * kappa0) / delta - kappa1 / delta -
               (3.0 * theta0) / (delta * delta) +
               (3.0 * theta1) / (delta * delta);
      auto d = kappa0 / (delta * delta) + kappa1 / (delta * delta) +
               (2.0 * theta0) / (delta * delta * delta) -
               (2.0 * theta1) / (delta * delta * delta);

      auto kappa = b + 2.0 * c * delta + 3.0 * d * delta * delta;
      auto dkappa = 2.0 * c + 6.0 * d * delta;
      obj_value += kappa * kappa * 1.0;
      obj_value += dkappa * dkappa * 1.0;
    }
    residual[0] = obj_value;
    return true;
  }
};
*/
}  // namespace reference_line