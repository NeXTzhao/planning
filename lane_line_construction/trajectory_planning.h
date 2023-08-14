#pragma once
#include "bezier.h"

enum RoadType { Straight = 0, Turn = 1, Roundabouts = 2 };

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
  //  Point Pb{43.0, 110.0}, Pi{43.0, 111.0}, PLC{40, 60};
  Point Pb;
  Point Pi;
  Point PLC;

 private:
  Point cal_ub() { return (Pb - Pi).normalized(); }
  Point cal_ua() { return cal_ub() * -1; }
  double D = 3.75;
  double w = 3.75;

 public:
  BezierLaneChange(const Point& pb, const Point& pi, const Point& plc) : Pb(pb), Pi(pi), PLC(plc) {
    cal_control_points();
  };

  Point cal_PLCC(bool isRightTurn) {
    auto ua = cal_ua();
    double theta = std::atan2(ua.y, ua.x);
    // 如果是向右变换，则为-号，如果是向左变换，则为+号
    if (isRightTurn) {
      theta -= M_PI * 0.5;
    } else {
      theta += M_PI * 0.5;
    }
    Point plcc{};
    plcc.x = PLC.x + w * std::cos(theta);
    plcc.y = PLC.y + w * std::sin(theta);

    return plcc;
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
  Point Pr{51, 90}, Pa{14, 79}, Pii{20.0, 71.0}, Pb{40.0, 71.0}, Pi{40, 90};
  double ai = 30 * M_PI / 180, a0 = 30 * M_PI / 180, D = 3.75, R = 10;

 private:
  double k1{};
  double k2{};
  Point center{};
  double radius{};

 public:
  BezierRoundabouts() { cal_RE(); };
  Point getCenter() const { return center; }
  double getRadius() const { return radius; }

  Point cal_ub() { return (Pb - Pi).normalized(); }
  Point cal_ua() { return (Pa - Pi).normalized(); }

  Point cal_Pe() {
    auto ub = cal_ub();
    //    double theta_e = std::atan2(ub.y, ub.x);
    double theta_e = std::atan2(Pr.y, Pr.x);

    //   环岛分为顺时针和逆时针,
    theta_e += ai;

    Point Pe{};
    Pe.x = Pr.x + R * std::cos(theta_e);
    Pe.y = Pr.y + R * std::sin(theta_e);

    return Pe;
  }

  void calculateCircle(const Point& p1, const Point& p2, const Point& p3) {
    // Calculate midpoints
    Point m1 = {(p1.x + p2.x) / 2, (p1.y + p2.y) / 2};
    Point m2 = {(p2.x + p3.x) / 2, (p2.y + p3.y) / 2};

    // Calculate slopes
    double dx1 = p2.x - p1.x;
    double dy1 = p2.y - p1.y;
    double dx2 = p3.x - p2.x;
    double dy2 = p3.y - p2.y;

    // Check for parallel lines
    if (dy1 == 0) {
      k1 = std::numeric_limits<double>::infinity();
    } else {
      k1 = -dx1 / dy1;
    }

    if (dy2 == 0) {
      k2 = std::numeric_limits<double>::infinity();
    } else {
      k2 = -dx2 / dy2;
    }

    // Calculate center
    center.x = (m2.y - m1.y - k2 * m2.x + k1 * m1.x) / (k1 - k2);
    center.y = m1.y + k1 * (center.x - m1.x);

    // Calculate radius
    radius = std::sqrt(std::pow(p1.x - center.x, 2) + std::pow(p1.y - center.y, 2));
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
    //        Point Pe{40, 89};
    double theta_e = std::atan2(Pe.y - Pr.y, Pe.x - Pr.x);
    Point u_theta_e{std::cos(theta_e + D / R), std::sin(theta_e + D / R)};
    std::cout << "pe = " << Pe.x << " , " << Pe.y << std::endl;
    Point ue = (Pb - Pe).normalized();

    //    Point P0 = D * ue * 1.5 + Pe;
    Point P0 = Pb;

    Point P1 = D * ue * 0.5 + Pe;
    Point P2 = Pe;
    Point P4 = Pr + R * u_theta_e;
    //    Point P4{43, 96};
    Point arc1{46, 98};
    Point arc{44, 97};

    //    Point P4 = arc;
    //    Point P3 = D * P4 .normalized() + P4;
    Point P3 = D * (arc1 - P4).normalized() + P4;
    controlPoints_ = {P0, P1, P2, P3, P4};

    degree_ = (int) controlPoints_.size() - 1;
  }

  void cal_REX() {}
};
