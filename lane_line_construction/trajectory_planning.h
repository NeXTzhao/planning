#pragma once
#include "bezier.h"

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

    controlPoints_ = {P0, P1, P2, P3, P4, P5};
    degree_ = (int) controlPoints_.size() - 1;
  }
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
  Point cal_PLCC(bool isRightTurn) {
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
    auto PLCC = cal_PLCC(true);

    Point P0 = ub * D * 2.5 + PLC;
    Point P1 = ub * D * 1.5 + PLC;
    Point P2 = ub * D * 0.5 + PLC;
    Point P3 = ua * D * 0.5 + PLCC;
    Point P4 = ua * D * 1.5 + PLCC;
    Point P5 = ua * D * 2.5 + PLCC;

    controlPoints_ = {P0, P1, P2, P3, P4, P5};
    degree_ = (int) controlPoints_.size() - 1;
  }
};

class BezierRoundabouts : public BezierCurve {
 private:
  Point Pr{20, 120}, Pa{14, 79}, Pii{20.0, 71.0}, Pb{43.0, 87.0}, Pi{-14, 140};
  double ai = 20 * M_PI / 180, a0 = 20 * M_PI / 180, D = 3.75, R = 20 + 3.75;

 public:
  BezierRoundabouts() { cal_RE(); };

  Point cal_ub() { return (Pb - Pi).normalized(); }
  Point cal_ua() { return (Pa - Pi).normalized(); }

  Point cal_Pe() {
    auto ub = cal_ub();
    double theta_e = std::atan2(ub.y, ub.x);
    //   环岛分为顺时针和逆时针，国内应该都是逆时针？
    theta_e += ai;

    Point Pe{};
    Pe.x = Pr.x + R * std::cos(theta_e);
    Pe.y = Pr.y + R * std::sin(theta_e);

    return Pe;
  }

  Point cal_Pex() {
    auto ua = cal_ua();
    double theta_ex = std::atan2(ua.y, ua.x);
    theta_ex -= a0;
    Point Pex{};
    Pex.x = Pr.x + R * std::cos(theta_ex);
    Pex.y = Pr.y + R * std::sin(theta_ex);

    return Pex;
  }

  void cal_RE() {
    auto Pe = cal_Pe();
    double theta_e = std::atan2(Pe.y - Pr.y, Pe.x - Pr.x);
    Point u_theta_e{std::cos(theta_e + D / R), std::sin(theta_e + D / R)};
    Point ue = (Pe - Pb).normalized() * -1;

    Point P0 = D * ue * 1.5 + Pe;
    Point P1 = D * ue * 0.5 + Pe;
    Point P2 = Pe;
    Point P4 = Pr + R * u_theta_e.normalized();
    Point arc{38, 126};
    Point P3 = D * (P4 - arc).normalized() + P4;

    controlPoints_ = {P0, P1, P2, P3, P4};

    degree_ = (int) controlPoints_.size() - 1;
  }

  void cal_REX() {}
};
