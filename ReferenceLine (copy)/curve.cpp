#include "curve.h"

#include <algorithm>
#include <cassert>
#include <cmath>

namespace reference_line {
Curve::Curve(const std::vector<double> &xs, const std::vector<double> &ys) {
  assert(xs.size() == ys.size());
  assert(xs.size() >= 2);
  xs_ = xs;
  ys_ = ys;
  thetas_.reserve(xs.size());
  kappas_.reserve(xs.size());
  dkappas_.reserve(xs.size());
  deltas_.reserve(xs.size() - 1);
  acc_s_.reserve(xs.size());

  std::vector<double> dxs;
  std::vector<double> dys;
  std::vector<double> y_over_s_first_derivatives;
  std::vector<double> x_over_s_first_derivatives;
  std::vector<double> y_over_s_second_derivatives;
  std::vector<double> x_over_s_second_derivatives;
  size_t points_size = xs.size();
  for (std::size_t i = 0; i < points_size; ++i) {
    double x_delta = 0.0;
    double y_delta = 0.0;
    if (i == 0) {
      x_delta = (xs[i + 1] - xs[i]);
      y_delta = (ys[i + 1] - ys[i]);
    } else if (i == points_size - 1) {
      x_delta = (xs[i] - xs[i - 1]);
      y_delta = (ys[i] - ys[i - 1]);
    } else {
      x_delta = 0.5 * (xs[i + 1] - xs[i - 1]);
      y_delta = 0.5 * (ys[i + 1] - ys[i - 1]);
    }
    dxs.push_back(x_delta);
    dys.push_back(y_delta);
  }
  // theta calculation
  for (std::size_t i = 0; i < points_size; ++i) {
    thetas_.emplace_back(std::atan2(dys[i], dxs[i]));
  }

  // thetas_.emplace_back(thetas_.back());
  double distance = 0.0;
  acc_s_.emplace_back(distance);
  double fx = xs[0];
  double fy = ys[0];
  double nx = 0.0;
  double ny = 0.0;

  for (size_t i = 1; i < points_size; ++i) {
    nx = xs[i];
    ny = ys[i];
    double end_segment_s =
        std::sqrt((fx - nx) * (fx - nx) + (fy - ny) * (fy - ny));
    deltas_.emplace_back(end_segment_s);
    acc_s_.emplace_back(end_segment_s + distance);
    distance += end_segment_s;
    fx = nx;
    fy = ny;
  }

  /**kappa calculate*/
  for (size_t i = 0; i < points_size; ++i) {
    double xds = 0.0;
    double yds = 0.0;
    if (i == 0) {
      xds = (xs[i + 1] - xs[i]) / (acc_s_.at(i + 1) - acc_s_.at(i));
      yds = (ys[i + 1] - ys[i]) / (acc_s_.at(i + 1) - acc_s_.at(i));
    } else if (i == xs.size() - 1) {
      xds = (xs[i] - xs[i - 1]) / (acc_s_.at(i) - acc_s_.at(i - 1));
      yds = (ys[i] - ys[i - 1]) / (acc_s_.at(i) - acc_s_.at(i - 1));
    } else {
      xds = (xs[i + 1] - xs[i - 1]) / (acc_s_.at(i + 1) - acc_s_.at(i - 1));
      yds = (ys[i + 1] - ys[i - 1]) / (acc_s_.at(i + 1) - acc_s_.at(i - 1));
    }
    x_over_s_first_derivatives.push_back(xds);
    y_over_s_first_derivatives.push_back(yds);
  }

  for (std::size_t i = 0; i < points_size; ++i) {
    double xdds = 0.0;
    double ydds = 0.0;
    if (i == 0) {
      xdds =
          (x_over_s_first_derivatives[i + 1] - x_over_s_first_derivatives[i]) /
          (acc_s_.at(i + 1) - acc_s_.at(i));
      ydds =
          (y_over_s_first_derivatives[i + 1] - y_over_s_first_derivatives[i]) /
          (acc_s_.at(i + 1) - acc_s_.at(i));
    } else if (i == xs.size() - 1) {
      xdds =
          (x_over_s_first_derivatives[i] - x_over_s_first_derivatives[i - 1]) /
          (acc_s_.at(i) - acc_s_.at(i - 1));
      ydds =
          (y_over_s_first_derivatives[i] - y_over_s_first_derivatives[i - 1]) /
          (acc_s_.at(i) - acc_s_.at(i - 1));
    } else {
      xdds = (x_over_s_first_derivatives[i + 1] -
              x_over_s_first_derivatives[i - 1]) /
             (acc_s_.at(i + 1) - acc_s_.at(i - 1));
      ydds = (y_over_s_first_derivatives[i + 1] -
              y_over_s_first_derivatives[i - 1]) /
             (acc_s_.at(i + 1) - acc_s_.at(i - 1));
    }
    x_over_s_second_derivatives.push_back(xdds);
    y_over_s_second_derivatives.push_back(ydds);
  }

  /*kappa calculation*/
  for (size_t i = 0; i < xs.size(); ++i) {
    double xds = x_over_s_first_derivatives[i];
    double yds = y_over_s_first_derivatives[i];
    double xdds = x_over_s_second_derivatives[i];
    double ydds = y_over_s_second_derivatives[i];
    double kappa =
        (xds * ydds - yds * xdds) /
        (std::sqrt(xds * xds + yds * yds) * (xds * xds + yds * yds) + 1e-6);
    kappas_.emplace_back(kappa);
  }

  /*dkappa calculation*/
  for (size_t i = 0; i < points_size; ++i) {
    double dkappa = 0.0;
    if (i == 0) {
      dkappa = (kappas_.at(i + 1) - kappas_.at(i)) /
               (acc_s_.at(i + 1) - acc_s_.at(i));
    } else if (i == xs.size() - 1) {
      dkappa = (kappas_.at(i) - kappas_.at(i - 1)) /
               (acc_s_.at(i) - acc_s_.at(i - 1));
    } else {
      dkappa = (kappas_.at(i + 1) - kappas_.at(i - 1)) /
               (acc_s_.at(i + 1) - acc_s_.at(i - 1));
    }
    dkappas_.emplace_back(dkappa);
  }

  for (size_t i = 0; i < deltas_.size(); i++) {
    segments_.emplace_back(CurveSegment(
        thetas_[i], kappas_[i], dkappas_[i], xs[i], ys[i], thetas_[i + 1],
        kappas_[i + 1], dkappas_[i + 1], xs[i + 1], ys[i + 1], deltas_[i]));
  }
}

Curve::Curve(const std::vector<double> &thetas,
             const std::vector<double> &kappas,
             const std::vector<double> &dkappas, const std::vector<double> &xs,
             const std::vector<double> &ys, const std::vector<double> &deltas) {
  assert(thetas.size() == kappas.size());
  assert(kappas.size() == xs.size());
  assert(dkappas.size() == xs.size());
  assert(xs.size() == ys.size());
  assert(ys.size() == deltas.size() + 1);
  assert(thetas.size() >= 2);
  thetas_ = thetas;
  kappas_ = kappas;
  dkappas_ = dkappas;
  xs_ = xs;
  ys_ = ys;
  deltas_ = deltas;
  segments_.reserve(deltas.size());
  acc_s_.reserve(xs.size());
  acc_s_.emplace_back(0);
  for (size_t i = 0; i < deltas.size(); i++) {
    segments_.emplace_back(CurveSegment(
        thetas[i], kappas[i], dkappas[i], xs[i], ys[i], thetas[i + 1],
        kappas[i + 1], dkappas[i + 1], xs[i + 1], ys[i + 1], deltas[i]));
    acc_s_.emplace_back(acc_s_.back() + deltas[i]);
  }
}

void Curve::get_index(const double s, int &index, double &ds) const {
  auto it = std::upper_bound(acc_s_.begin(), acc_s_.end(), s);
  // std::prve 获取前一个元素的迭代器
  if (it == acc_s_.end()) {
    it = std::prev(it, 2);
  } else if (it != acc_s_.begin()) {
    it = std::prev(it);
  }
  index = std::distance(acc_s_.begin(), it);
  ds = s - *it;
}

double Curve::theta(double s) const {
  int index;
  double ds;
  get_index(s, index, ds);
  return segments_[index].theta(ds);
}

double Curve::kappa(double s) const {
  int index;
  double ds;
  get_index(s, index, ds);
  return segments_[index].kappa(ds);
}

double Curve::dkappa(double s) const {
  int index;
  double ds;
  get_index(s, index, ds);
  return segments_[index].dkappa(ds);
}

double Curve::x(double s) const {
  int index;
  double ds;
  get_index(s, index, ds);
  return segments_[index].x(ds);
}

double Curve::y(double s) const {
  int index;
  double ds;
  get_index(s, index, ds);
  return segments_[index].y(ds);
}
} // namespace reference_line
