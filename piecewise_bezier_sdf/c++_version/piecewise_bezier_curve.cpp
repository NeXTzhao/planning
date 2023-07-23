#include <math.h>
#include <stdlib.h>

#include "piecewise_bezier_curve.h"

void BezierFitting::FitCurve(Point2 *d, int nPts) {
  std::vector<Point2> points(d, d + nPts);
  Point2 tHat1 = ComputeLeftTangent(points, 0);
  Point2 tHat2 = ComputeRightTangent(points, nPts - 1);
  FitCubic(points, 0, nPts - 1, tHat1, tHat2);
}

void BezierFitting::FitCubic(std::vector<Point2> &d, int first, int last, Point2 tHat1, Point2 tHat2) {
  std::vector<Point2> bezCurve;
  double *u = ChordLengthParameterize(d, first, last);
  bezCurve = GenerateBezier(d, first, last, u, tHat1, tHat2);

  int splitPoint;
  double maxError = ComputeMaxError(d, first, last, bezCurve, u, &splitPoint);
  free(u);

  if (maxError < error_) {
    DrawBezierCurve(bezCurve);
    return;
  }

  double iterationError = error_ * 4.0;
  int maxIterations = 4;
  for (int i = 0; i < maxIterations; i++) {
    double *uPrime = Reparameterize(d, first, last, u, bezCurve);
    free(u);
    u = uPrime;
    bezCurve = GenerateBezier(d, first, last, u, tHat1, tHat2);

    maxError = ComputeMaxError(d, first, last, bezCurve, u, &splitPoint);
    if (maxError < error_) {
      DrawBezierCurve(bezCurve);
      free(u);
      return;
    }
  }

  free(u);
  Point2 tHatCenter = ComputeCenterTangent(d, splitPoint);
  FitCubic(d, first, splitPoint, tHat1, tHatCenter);
  tHatCenter.x = -tHatCenter.x;
  tHatCenter.y = -tHatCenter.y;
  FitCubic(d, splitPoint, last, tHatCenter, tHat2);
}

double *BezierFitting::Reparameterize(std::vector<Point2> &d,
                                      int first,
                                      int last,
                                      double *u,
                                      std::vector<Point2> &bezCurve) {
  int nPts = last - first + 1;
  double *uPrime = (double *) malloc(nPts * sizeof(double));
  for (int i = first; i <= last; i++) {
    uPrime[i - first] = NewtonRaphsonRootFind(bezCurve, d[i], u[i - first]);
  }
  return uPrime;
}

double BezierFitting::NewtonRaphsonRootFind(std::vector<Point2> &Q, Point2 P, double u) {
  double numerator, denominator;
  std::vector<Point2> Q1(3), Q2(2);

  Point2 Q_u = BezierII(3, Q, u);

  for (int i = 0; i <= 2; i++) {
    Q1[i].x = (Q[i + 1].x - Q[i].x) * 3.0;
    Q1[i].y = (Q[i + 1].y - Q[i].y) * 3.0;
  }

  for (int i = 0; i <= 1; i++) {
    Q2[i].x = (Q1[i + 1].x - Q1[i].x) * 2.0;
    Q2[i].y = (Q1[i + 1].y - Q1[i].y) * 2.0;
  }

  Point2 Q1_u = BezierII(2, Q1, u);
  Point2 Q2_u = BezierII(1, Q2, u);

  numerator = (Q_u.x - P.x) * (Q1_u.x) + (Q_u.y - P.y) * (Q1_u.y);
  denominator = (Q1_u.x) * (Q1_u.x) + (Q1_u.y) * (Q1_u.y) + (Q_u.x - P.x) * (Q2_u.x) + (Q_u.y - P.y) * (Q2_u.y);

  if (denominator == 0.0) return u;

  return u - (numerator / denominator);
}

Point2 BezierFitting::BezierII(int degree, std::vector<Point2> &V, double t) {
  std::vector<Point2> Vtemp(V);
  for (int i = 1; i <= degree; i++) {
    for (int j = 0; j <= degree - i; j++) {
      Vtemp[j].x = (1.0 - t) * Vtemp[j].x + t * Vtemp[j + 1].x;
      Vtemp[j].y = (1.0 - t) * Vtemp[j].y + t * Vtemp[j + 1].y;
    }
  }
  return Vtemp[0];
}

double BezierFitting::B0(double u) {
  double tmp = 1.0 - u;
  return tmp * tmp * tmp;
}

double BezierFitting::B1(double u) {
  double tmp = 1.0 - u;
  return 3 * u * (tmp * tmp);
}

double BezierFitting::B2(double u) {
  double tmp = 1.0 - u;
  return 3 * u * u * tmp;
}

double BezierFitting::B3(double u) {
  return u * u * u;
}

Point2 BezierFitting::ComputeLeftTangent(std::vector<Point2> &d, int end) {
  Point2 tHat1;
  tHat1.x = d[end + 1].x - d[end].x;
  tHat1.y = d[end + 1].y - d[end].y;
  tHat1.x /= std::hypot(tHat1.x, tHat1.y);
  tHat1.y /= std::hypot(tHat1.x, tHat1.y);
  return tHat1;
}

Point2 BezierFitting::ComputeRightTangent(std::vector<Point2> &d, int end) {
  Point2 tHat2;
  tHat2.x = d[end].x - d[end - 1].x;
  tHat2.y = d[end].y - d[end - 1].y;
  tHat2.x /= std::hypot(tHat2.x, tHat2.y);
  tHat2.y /= std::hypot(tHat2.x, tHat2.y);
  return tHat2;
}

Point2 BezierFitting::ComputeCenterTangent(std::vector<Point2> &d, int center) {
  Point2 V1, V2, tHatCenter;
  V1.x = d[center - 1].x - d[center].x;
  V1.y = d[center - 1].y - d[center].y;
  V2.x = d[center].x - d[center + 1].x;
  V2.y = d[center].y - d[center + 1].y;

  tHatCenter.x = (V1.x + V2.x) * 0.5;
  tHatCenter.y = (V1.y + V2.y) * 0.5;
  tHatCenter.x /= std::hypot(tHatCenter.x, tHatCenter.y);
  tHatCenter.y /= std::hypot(tHatCenter.x, tHatCenter.y);
  return tHatCenter;
}

double *BezierFitting::ChordLengthParameterize(std::vector<Point2> &d, int first, int last) {
  int i;
  double *u = (double *) malloc((last - first + 1) * sizeof(double));
  u[0] = 0.0;
  for (i = first + 1; i <= last; i++) {
    u[i - first] = u[i - first - 1] + std::hypot(d[i].x - d[i - 1].x, d[i].y - d[i - 1].y);
  }
  for (i = first + 1; i <= last; i++) {
    u[i - first] = u[i - first] / u[last - first];
  }
  return u;
}

std::vector<std::vector<double>> BezierFitting::ComputeB(double *u) {
  std::vector<std::vector<double>> B(4, std::vector<double>(4, 0.0));

  B[0][0] = B0(u[0]);
  B[0][1] = B1(u[0]);
  B[0][2] = B2(u[0]);
  B[0][3] = B3(u[0]);

  B[1][0] = B0(u[1]);
  B[1][1] = B1(u[1]);
  B[1][2] = B2(u[1]);
  B[1][3] = B3(u[1]);

  B[2][0] = B0(u[2]);
  B[2][1] = B1(u[2]);
  B[2][2] = B2(u[2]);
  B[2][3] = B3(u[2]);

  B[3][0] = B0(u[3]);
  B[3][1] = B1(u[3]);
  B[3][2] = B2(u[3]);
  B[3][3] = B3(u[3]);

  return B;
}

std::vector<Point2> BezierFitting::GenerateBezier(std::vector<Point2> &d,
                                                  int first,
                                                  int last,
                                                  double *u,
                                                  Point2 tHat1,
                                                  Point2 tHat2) {
  int i;
  std::vector<Point2> bezCurve(4);
  std::vector<std::vector<double>> B = ComputeB(u);

  bezCurve[0] = d[first];
  bezCurve[3] = d[last];

  tHat1.x *= std::hypot(d[first].x - d[first + 1].x, d[first].y - d[first + 1].y);
  tHat1.y *= std::hypot(d[first].x - d[first + 1].x, d[first].y - d[first + 1].y);

  tHat2.x *= std::hypot(d[last - 1].x - d[last].x, d[last - 1].y - d[last].y);
  tHat2.y *= std::hypot(d[last - 1].x - d[last].x, d[last - 1].y - d[last].y);

  for (i = 0; i <= 3; i++) {
    bezCurve[1].x += B[0][i] * d[first].x + B[1][i] * (d[first].x + tHat1.x);
    bezCurve[1].y += B[0][i] * d[first].y + B[1][i] * (d[first].y + tHat1.y);
    bezCurve[2].x += B[2][i] * d[last].x + B[3][i] * (d[last].x + tHat2.x);
    bezCurve[2].y += B[2][i] * d[last].y + B[3][i] * (d[last].y + tHat2.y);
  }

  return bezCurve;
}

double BezierFitting::ComputeMaxError(std::vector<Point2> &d,
                                      int first,
                                      int last,
                                      std::vector<Point2> &bezCurve,
                                      double *u,
                                      int *splitPoint) {
  int i;
  double maxDist;
  int split;
  Point2 P;
  Point2 v;
  double dist;

  split = (last - first + 1) / 2;
  maxDist = 0.0;
  for (i = first + 1; i < last; i++) {
    P = BezierII(3, bezCurve, u[i - first]);
    v.x = P.x - d[i].x;
    v.y = P.y - d[i].y;
    dist = v.x * v.x + v.y * v.y;
    if (dist >= maxDist) {
      maxDist = dist;
      *splitPoint = i;
    }
  }

  return maxDist;
}

// 绘制贝塞尔曲线的函数，你可以根据需求实现该函数来进行显示或输出
void BezierFitting::DrawBezierCurve(const std::vector<Point2> &curve) {
  // 这里可以实现贝塞尔曲线的绘制
  // 你可以根据自己的需求选择使用图形库或其他绘制方法
  // 例如使用OpenGL、OpenCV等进行绘制
}
