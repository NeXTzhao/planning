#pragma once
#include "bezier.h"

class BezierTurn : public BezierCurve {
 private:
  int D{};
  Point Pa{}, Pb{}, Pi{};
  Point cal_ua() { return (Pb - Pi).normalized(); }
  Point cal_ub() { return (Pa - Pi).normalized(); }
 public:

  std::vector<Point> cal_control_points() {
    auto ua = cal_ua();
    auto ub = cal_ub();

    Point P0 = 4 * D * ua + Pi;
    Point P1 = 2 * D * ub + Pi;
    Point P2 = D * ub + Pi;
    Point P3 = D * ua + Pi;
    Point P4 = 2 * D * ua + Pi;
    Point P5 = 4 * D * ua + Pi;

    std::vector<Point> con_pts{P0,P1,P2,P3,P4,P5};
    return con_pts;
  }

  

};

class BezierLaneChange : public BezierCurve {};

class BezierRoundabouts : public BezierCurve {};
