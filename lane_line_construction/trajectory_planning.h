#pragma once
#include "bezier.h"

class BezierTurn : public BezierCurve {
 private:
  int D{};
  Point Pa{}, Pb{}, Pint{};

 public:
  Point ua() { return (Pb - Pint).normalized(); }
  Point ub() { return (Pa - Pint).normalized(); }


};

class BezierLaneChange : public BezierCurve {};

class BezierRoundabouts : public BezierCurve {};
