//
// Created by vtd on 23-7-24.
//

#pragma once

#include <cmath>
#include <vector>

class CubicBezier {
 public:
  struct Point2 {
    double x;
    double y;
  };

  // Evaluate cubic Bezier curve at parameter t
  static Point2 q(const std::vector<Point2>& ctrlPoly, double t) {
    double oneMinusT = 1.0 - t;
    double oneMinusTSquared = oneMinusT * oneMinusT;
    double tSquared = t * t;

    Point2 result{};
    result.x = oneMinusTSquared * oneMinusT * ctrlPoly[0].x + 3.0 * oneMinusTSquared * t * ctrlPoly[1].x +
        3.0 * oneMinusT * tSquared * ctrlPoly[2].x + tSquared * t * ctrlPoly[3].x;
    result.y = oneMinusTSquared * oneMinusT * ctrlPoly[0].y + 3.0 * oneMinusTSquared * t * ctrlPoly[1].y +
        3.0 * oneMinusT * tSquared * ctrlPoly[2].y + tSquared * t * ctrlPoly[3].y;

    return result;
  }

  // Evaluate first derivative of cubic Bezier curve at parameter t
  static Point2 qprime(const std::vector<Point2>& ctrlPoly, double t) {
    double oneMinusT = 1.0 - t;
    double oneMinusTSquared = oneMinusT * oneMinusT;
    double tSquared = t * t;

    Point2 result;
    result.x = 3.0 * oneMinusTSquared * (ctrlPoly[1].x - ctrlPoly[0].x) +
        6.0 * oneMinusT * t * (ctrlPoly[2].x - ctrlPoly[1].x) +
        3.0 * tSquared * (ctrlPoly[3].x - ctrlPoly[2].x);
    result.y = 3.0 * oneMinusTSquared * (ctrlPoly[1].y - ctrlPoly[0].y) +
        6.0 * oneMinusT * t * (ctrlPoly[2].y - ctrlPoly[1].y) +
        3.0 * tSquared * (ctrlPoly[3].y - ctrlPoly[2].y);

    return result;
  }

  // Evaluate second derivative of cubic Bezier curve at parameter t
  static Point2 qprimeprime(const std::vector<Point2>& ctrlPoly, double t) {
    double oneMinusT = 1.0 - t;
    double tSquared = t * t;

    Point2 result{};
    result.x = 6.0 * oneMinusT * (ctrlPoly[2].x - 2.0 * ctrlPoly[1].x + ctrlPoly[0].x) +
        6.0 * t * (ctrlPoly[3].x - 2.0 * ctrlPoly[2].x + ctrlPoly[1].x);
    result.y = 6.0 * oneMinusT * (ctrlPoly[2].y - 2.0 * ctrlPoly[1].y + ctrlPoly[0].y) +
        6.0 * t * (ctrlPoly[3].y - 2.0 * ctrlPoly[2].y + ctrlPoly[1].y);

    return result;
  }
};


