#pragma once

#include <cmath>
#include <vector>
#define MAXPOINTS 1000
struct Point2 {
  double x, y;
};
typedef Point2 Vector2;
typedef Point2 *BezierCurve;

class BezierFitting {
 public:
  BezierFitting(const std::vector<Point2> &points, double error);
  void DrawBezierCurve(int n);

  // private:
  void FitCurve(std::vector<Point2> d, int nPts, double error);
  void FitCubic(Point2 *d, int first, int last, Vector2 tHat1, Vector2 tHat2, double error);
  std::vector<double> ChordLengthParameterize(const std::vector<Point2> &d, int first, int last);
  static double *Reparameterize(const std::vector<Point2> &d,
                                int first,
                                int last,
                                double *u,
                                const std::vector<Point2> &bezCurve);
  static double NewtonRaphsonRootFind(const std::vector<Point2> &Q, Point2 P, double u);
  static Point2 BezierII(int degree, Point2 *V, double t);
  static double B0(double u);
  static double B1(double u);
  static double B2(double u);
  static double B3(double u);
  static BezierCurve GenerateBezier(Point2 *d, int first, int last, double *uPrime, Vector2 tHat1, Vector2 tHat2);
  Vector2 ComputeLeftTangent(const std::vector<Point2> &d, int end);
  Vector2 ComputeRightTangent(const std::vector<Point2> &d, int end);
  Vector2 ComputeCenterTangent(const std::vector<Point2> &d, int center);
  double ComputeMaxError(Point2 *d,
                         int first, int last,
                         BezierCurve bezCurve,
                         double *u,
                         int *splitPoint);
  static double V2DistanceBetween2Points(const Point2 *a, const Point2 *b) {
    double dx = a->x - b->x;
    double dy = a->y - b->y;
    return std::hypot(dx, dy);
  }
  static Vector2 V2ScaleIII(Vector2 v, double s) {
    Vector2 result;
    result.x = v.x * s;
    result.y = v.y * s;
    return result;
  }
  static double V2SquaredLength(Vector2 *a) {
    return ((a->x * a->x) + (a->y * a->y));
  }
  Vector2 *V2Normalize(Vector2 *v) {
    double len = V2Length(v);
    if (len != 0.0) {
      v->x /= len;
      v->y /= len;
    }
    return (v);
  }
  static Vector2 *V2Negate(Vector2 *v) {
    v->x = -v->x;
    v->y = -v->y;
    return (v);
  }
  static double V2Length(Vector2 *a) {
    return (std::sqrt(V2SquaredLength(a)));
  }
  static Vector2 *V2Scale(Vector2 *v,
                          double newlen) {
    double len = V2Length(v);
    if (len != 0.0) {
      v->x *= newlen / len;
      v->y *= newlen / len;
    }
    return (v);
  }
  static Vector2 *V2Add(Vector2 *a, Vector2 *b, Vector2 *c) {
    c->x = a->x + b->x;
    c->y = a->y + b->y;
    return (c);
  }
  static double V2Dot(Vector2 *a, Vector2 *b) {
    return ((a->x * b->x) + (a->y * b->y));
  }
  static Vector2 V2AddII(Vector2 a, Vector2 b) {
    Vector2 c;
    c.x = a.x + b.x;
    c.y = a.y + b.y;
    return (c);
  }

  static Vector2 V2SubII(Vector2 a, Vector2 b) {
    Vector2 c;
    c.x = a.x - b.x;
    c.y = a.y - b.y;
    return (c);
  }
  std::vector<Point2> controlPoints;
  double tolerance;
};
