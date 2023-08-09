#pragma once
#include "bezier.h"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

class BezierTurn : public BezierCurve {
 private:
  double D = 3.75;
  Point Pa{40.0, 40.0}, Pb{0.0, 0.0}, Pi{40.0, 0.0};
  Point cal_ua() { return (Pb - Pi).normalized(); }
  Point cal_ub() { return (Pa - Pi).normalized(); }

 public:
  BezierTurn() { cal_control_points(); };

  void cal_control_points() {
    auto ua = cal_ua();
    auto ub = cal_ub();
    Point P0 = 4 * D * ub + Pi;
    Point P1 = 2 * D * ub + Pi;
    Point P2 = D * ub + Pi;
    Point P3 = D * ua + Pi;
    Point P4 = 2 * D * ua + Pi;
    Point P5 = 4 * D * ua + Pi;

    //    std::vector<Point> con_pts{P0, P1, P2, P3, P4, P5};
    controlPoints_ = {P0, P1, P2, P3, P4, P5};
    degree_ = controlPoints_.size() - 1;
    //    return con_pts;
  }

  //  void vis_curvature() {
  //    std::vector<double> x, y, kappa, dkappa;
  //    auto points = this->getCurvePoints();
  //    auto k = this->getCurveKappa();
  //    auto dk = this->getCurveDkappa();
  //
  //    for (int i = 0; i < points.size(); ++i) {
  //      x.push_back(points.at(i).x);
  //      y.push_back(points.at(i).y);
  //
  //      kappa.push_back(k[i]);
  //      dkappa.push_back(dk[i]);
  //    }
  //    plt::named_plot("local trajectory", x, y, "r");
  //    plt::figure();
  //    plt::named_plot("local trajectory kappa", kappa, "b-");
  //    plt::named_plot("local trajectory dkappa", dkappa, "r-");
  //    plt::grid(true);
  //    plt::legend();
  //  }
};

class BezierLaneChange : public BezierCurve {
 private:
  double D = 3.75;
  double w = 3.75;
  Point Pa{43.0, 110.0}, Pi{43.0, 111.0}, PLC{40, 60};
  Point cal_ub() { return (Pa - Pi).normalized(); }
  Point cal_ua() { return cal_ub() * -1; }

 public:
  BezierLaneChange() { cal_control_points(); };
  Point getPLCC(bool isRightTurn) {
    auto ua = cal_ua();
    double theta = std::atan2(ua.y, ua.x);
    // 如果是向右变换，则为-号，如果是向左变换，则为+号
    if (isRightTurn) {
      theta -= M_PI * 0.5;
    } else {
      theta += M_PI * 0.5;
    }
    Point plc{};
    plc.x = PLC.x + w * std::cos(theta);
    plc.y = PLC.y + w * std::sin(theta);

    return plc;
  }

  void cal_control_points() {
    auto ua = cal_ua();
    auto ub = cal_ub();
    auto PLCC = getPLCC(true);

    Point P0 = ub * D * 2.5 + PLC;
    Point P1 = ub * D * 1.5 + PLC;
    Point P2 = ub * D * 0.5 + PLC;
    Point P3 = ua * D * 0.5 + PLCC;
    Point P4 = ua * D * 1.5 + PLCC;
    Point P5 = ua * D * 2.5 + PLCC;

    controlPoints_ = {P0, P1, P2, P3, P4, P5};
    degree_ = controlPoints_.size() - 1;
  }
};

class BezierRoundabouts : public BezierCurve {};
