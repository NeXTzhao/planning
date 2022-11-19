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

  inline void get_index(const double s, int &index, double &ds) const;

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
}  // namespace reference_line