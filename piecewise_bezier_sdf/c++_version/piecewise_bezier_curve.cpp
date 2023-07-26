#include "piecewise_bezier_curve.h"
#include <cmath>
#include <iostream>

BezierFitting::BezierFitting(const std::vector<Point2> &points, double error) : tolerance(error) {
  FitCurve(points, points.size(), error);
}

BezierCurve BezierFitting::GenerateBezier(Point2 *d,
                                          int first, int last,
                                          double *uPrime,
                                          Vector2 tHat1, Vector2 tHat2) {
  int i;

  Vector2 A[MAXPOINTS][2]; /* Precomputed rhs for eqn	*/
  int nPts;                /* Number of pts in sub-curve */
  double C[2][2];          /* Matrix C		*/
  double X[2];             /* Matrix X			*/
  double det_C0_C1,        /* Determinants of matrices	*/
      det_C0_X, det_X_C1;
  double alpha_l, /* Alpha values, left and right	*/
      alpha_r;
  Vector2 tmp;          /* Utility variable		*/
  BezierCurve bezCurve; /* RETURN bezier curve ctl pts	*/
  double segLength;
  double epsilon;

  bezCurve = (Point2 *) malloc(4 * sizeof(Point2));
  nPts = last - first + 1;

  /* Compute the A's	*/
  for (i = 0; i < nPts; i++) {
    Vector2 v1, v2;
    v1 = tHat1;
    v2 = tHat2;
    V2Scale(&v1, B1(uPrime[i]));
    V2Scale(&v2, B2(uPrime[i]));
    A[i][0] = v1;
    A[i][1] = v2;
  }

  /* Create the C and X matrices	*/
  C[0][0] = 0.0;
  C[0][1] = 0.0;
  C[1][0] = 0.0;
  C[1][1] = 0.0;
  X[0] = 0.0;
  X[1] = 0.0;

  for (i = 0; i < nPts; i++) {
    C[0][0] += V2Dot(&A[i][0], &A[i][0]);
    C[0][1] += V2Dot(&A[i][0], &A[i][1]);
    /*					C[1][0] += V2Dot(&A[i][0], &A[i][1]);*/
    C[1][0] = C[0][1];
    C[1][1] += V2Dot(&A[i][1], &A[i][1]);

    tmp =
        V2SubII(d[first + i],
                V2AddII(V2ScaleIII(d[first], B0(uPrime[i])),
                        V2AddII(V2ScaleIII(d[first], B1(uPrime[i])),
                                V2AddII(V2ScaleIII(d[last], B2(uPrime[i])),
                                        V2ScaleIII(d[last], B3(uPrime[i]))))));

    X[0] += V2Dot(&A[i][0], &tmp);
    X[1] += V2Dot(&A[i][1], &tmp);
  }

  /* Compute the determinants of C and X	*/
  det_C0_C1 = C[0][0] * C[1][1] - C[1][0] * C[0][1];
  det_C0_X = C[0][0] * X[1] - C[1][0] * X[0];
  det_X_C1 = X[0] * C[1][1] - X[1] * C[0][1];

  /* Finally, derive alpha values	*/
  alpha_l = (det_C0_C1 == 0) ? 0.0 : det_X_C1 / det_C0_C1;
  alpha_r = (det_C0_C1 == 0) ? 0.0 : det_C0_X / det_C0_C1;

  /* If alpha negative, use the Wu/Barsky heuristic (see text) */
  /* (if alpha is 0, you get coincident control points that lead to
 * divide by zero in any subsequent NewtonRaphsonRootFind() call. */
  segLength = V2DistanceBetween2Points(&d[last], &d[first]);
  epsilon = 1.0e-6 * segLength;
  if (alpha_l < epsilon || alpha_r < epsilon) {
    /* fall back on standard (probably inaccurate) formula, and subdivide
 * further if needed. */
    double dist = segLength / 3.0;
    bezCurve[0] = d[first];
    bezCurve[3] = d[last];
    V2Add(&bezCurve[0], V2Scale(&tHat1, dist), &bezCurve[1]);
    V2Add(&bezCurve[3], V2Scale(&tHat2, dist), &bezCurve[2]);
    return (bezCurve);
  }

  /*  First and last control points of the Bezier curve are */
  /*  positioned exactly at the first and last data points */
  /*  Control points 1 and 2 are positioned an alpha distance out */
  /*  on the tangent vectors, left and right, respectively */
  bezCurve[0] = d[first];
  bezCurve[3] = d[last];
  V2Add(&bezCurve[0], V2Scale(&tHat1, alpha_l), &bezCurve[1]);
  V2Add(&bezCurve[3], V2Scale(&tHat2, alpha_r), &bezCurve[2]);
  return (bezCurve);
}

void BezierFitting::DrawBezierCurve(int n) {
  // Implementation of this function is not provided here.
  // You'll need to write this yourself based on your requirements.
  // It could be drawing the Bezier curve using a graphics library or
  // simply printing the points of the curve, etc.
  // For testing purposes, you can just output the points to the console.
  for (int i = 0; i < n; ++i) {
    std::cout << "x: " << controlPoints[i].x << ", y: " << controlPoints[i].y << std::endl;
  }
}

void BezierFitting::FitCurve(const std::vector<Point2> &d, int nPts, double error) {
  Vector2 tHat1, tHat2;

  tHat1 = ComputeLeftTangent(d, 0);
  tHat2 = ComputeRightTangent(d, nPts - 1);
  FitCubic(d, 0, nPts - 1, tHat1, tHat2, error);
}

void BezierFitting::FitCubic(const std::vector<Point2> &d,
                             int first,
                             int last,
                             Vector2 tHat1,
                             Vector2 tHat2,
                             double error) {
  BezierCurve bezCurve;  /* Control points of fitted Bezier curve */
  std::vector<double> u; /* Parameter values for point  */
  double *uPrime;        /* Improved parameter values */
  double maxError;       /* Maximum fitting error	 */
  int splitPoint;        /* Point to split point set at	 */
  int nPts;              /* Number of points in subset  */
  double iterationError; /* Error below which you try iterating  */
  int maxIterations = 4; /* Max times to try iterating  */
  Vector2 tHatCenter;    /* Unit tangent vector at splitPoint */
  int i;

  iterationError = error * 4.0; /* fixed issue 23 */
  nPts = last - first + 1;

  /* Use heuristic if the region only has two points in it */
  if (nPts == 2) {
    double dist = V2DistanceBetween2Points(&d[last], &d[first]) / 3.0;

    bezCurve = new Point2[4];
    bezCurve[0] = d[first];
    bezCurve[3] = d[last];
    V2Add(&bezCurve[0], V2Scale(&tHat1, dist), &bezCurve[1]);
    V2Add(&bezCurve[3], V2Scale(&tHat2, dist), &bezCurve[2]);
    DrawBezierCurve(3);
    return;
  }

  /* Parameterize points and attempt to fit the curve */
  u = ChordLengthParameterize(d, first, last);
  bezCurve = GenerateBezier(d, first, last, u, tHat1, tHat2);

  /* Find the max deviation of points to the fitted curve */
  maxError = ComputeMaxError(d, first, last, bezCurve, u, splitPoint);
  if (maxError < error) {
    //    DrawBezierCurve(3);
    //    delete[] u;
    return;
  }

  /* If the error is not too large, try some reparameterization and iteration */
  if (maxError < iterationError) {
    for (i = 0; i < maxIterations; i++) {
      uPrime = Reparameterize(d, first, last, u, bezCurve);
      delete[] bezCurve;
      bezCurve = GenerateBezier(d, first, last, uPrime, tHat1, tHat2);
      maxError = ComputeMaxError(d, first, last, bezCurve, uPrime, splitPoint);
      if (maxError < error) {
        DrawBezierCurve(3);
        delete[] u;
        delete[] uPrime;
        return;
      }
      delete[] u;
      u = uPrime;
    }
  }

  /* Fitting failed -- split at the max error point and fit recursively */
  delete[] u;
  delete[] bezCurve;
  tHatCenter = ComputeCenterTangent(d, splitPoint);
  FitCubic(d, first, splitPoint, tHat1, tHatCenter, error);
  V2Negate(&tHatCenter);
  FitCubic(d, splitPoint, last, tHatCenter, tHat2, error);
}

std::vector<double> BezierFitting::ChordLengthParameterize(const std::vector<Point2> &d, int first, int last) {
  int i;
  std::vector<double> u(last - first + 1);

  u[0] = 0.0;
  for (i = first + 1; i <= last; i++) {
    u[i - first] = u[i - first - 1] + V2DistanceBetween2Points(&d[i], &d[i - 1]);
  }

  for (i = first + 1; i <= last; i++) {
    u[i - first] = u[i - first] / u[last - first];
  }

  return u;
}

double *BezierFitting::Reparameterize(const std::vector<Point2> &d,
                                      int first,
                                      int last,
                                      double *u,
                                      const std::vector<Point2> &bezCurve) {
  int nPts = last - first + 1;
  int i;
  auto *uPrime = new double[nPts];

  for (i = first; i <= last; i++) {
    uPrime[i - first] = NewtonRaphsonRootFind(bezCurve, d[i], u[i - first]);
  }
  return uPrime;
}

double BezierFitting::NewtonRaphsonRootFind(const std::vector<Point2> &Q, Point2 P, double u) {
  double numerator, denominator;
  std::vector<Point2> Q1(3), Q2(2); /* Q' and Q'' */
  Point2 Q_u, Q1_u{}, Q2_u{};       /* u evaluated at Q, Q', & Q'' */
  double uPrime;                    /* Improved u */
  int i;

  /* Compute Q(u) */
  Q_u = BezierII(3, Q, u);

  /* Generate control vertices for Q' */
  for (i = 0; i <= 2; i++) {
    Q1[i].x = (Q[i + 1].x - Q[i].x) * 3.0;
    Q1[i].y = (Q[i + 1].y - Q[i].y) * 3.0;
  }

  /* Generate control vertices for Q'' */
  for (i = 0; i <= 1; i++) {
    Q2[i].x = (Q1[i + 1].x - Q1[i].x) * 2.0;
    Q2[i].y = (Q1[i + 1].y - Q1[i].y) * 2.0;
  }

  /* Compute Q'(u) and Q''(u) */
  Q1_u = BezierII(2, Q1, u);
  Q2_u = BezierII(1, Q2, u);

  /* Compute f(u)/f'(u) */
  numerator = (Q_u.x - P.x) * (Q1_u.x) + (Q_u.y - P.y) * (Q1_u.y);
  denominator = (Q1_u.x) * (Q1_u.x) + (Q1_u.y) * (Q1_u.y) + (Q_u.x - P.x) * (Q2_u.x) + (Q_u.y - P.y) * (Q2_u.y);

  /* u = u - f(u)/f'(u) */
  uPrime = u - (numerator / denominator);
  return uPrime;
}

Point2 BezierFitting::BezierII(int degree, const std::vector<Point2> &V, double t) {
  int i, j;
  Point2 Q{}; /* Point on curve at parameter t */
  int n = degree;
  std::vector<Point2> Vtemp(degree + 1); /* Local copy of control points  */

  for (i = 0; i <= degree; i++) {
    Vtemp[i] = V[i];
  }

  /* Triangle computation */
  for (i = 1; i <= n; i++) {
    for (j = 0; j <= n - i; j++) {
      Vtemp[j].x = (1.0 - t) * Vtemp[j].x + t * Vtemp[j + 1].x;
      Vtemp[j].y = (1.0 - t) * Vtemp[j].y + t * Vtemp[j + 1].y;
    }
  }

  Q = Vtemp[0];
  return Q;
}

double BezierFitting::B0(double u) {
  double tmp = 1.0 - u;
  return (tmp * tmp * tmp);
}

double BezierFitting::B1(double u) {
  double tmp = 1.0 - u;
  return (3 * u * (tmp * tmp));
}

double BezierFitting::B2(double u) {
  double tmp = 1.0 - u;
  return (3 * u * u * tmp);
}

double BezierFitting::B3(double u) {
  return (u * u * u);
}

Vector2 BezierFitting::ComputeLeftTangent(const std::vector<Point2> &d, int end) {
  Vector2 tHat1;

  tHat1.x = d[end + 1].x - d[end].x;
  tHat1.y = d[end + 1].y - d[end].y;
  V2Normalize(&tHat1);
  return tHat1;
}

Vector2 BezierFitting::ComputeRightTangent(const std::vector<Point2> &d, int end) {
  Vector2 tHat2;

  tHat2.x = d[end - 1].x - d[end].x;
  tHat2.y = d[end - 1].y - d[end].y;
  V2Normalize(&tHat2);
  return tHat2;
}

Vector2 BezierFitting::ComputeCenterTangent(const std::vector<Point2> &d, int center) {
  Vector2 V1, V2, tHatCenter;

  V1.x = d[center - 1].x - d[center].x;
  V1.y = d[center - 1].y - d[center].y;
  V2.x = d[center].x - d[center + 1].x;
  V2.y = d[center].y - d[center + 1].y;
  tHatCenter.x = (V1.x + V2.x) / 2.0;
  tHatCenter.y = (V1.y + V2.y) / 2.0;
  V2Normalize(&tHatCenter);
  return tHatCenter;
}

double BezierFitting::ComputeMaxError(Point2 *d,
                                      int first, int last,
                                      BezierCurve bezCurve,
                                      double *u,
                                      int *splitPoint) {
  int i;
  double maxDist; /* Maximum error */
  double dist;    /* Current error */
  Point2 P{};     /* Point on curve */
  Vector2 v;      /* Vector from point to curve */

  *splitPoint = (last - first + 1) / 2;
  maxDist = 0.0;
  for (i = first + 1; i < last; i++) {
    P = BezierII(3, bezCurve, u[i - first]);
    v.x = P.x - d[i].x;
    v.y = P.y - d[i].y;
    dist = V2SquaredLength(&v);
    if (dist >= maxDist) {
      maxDist = dist;
      splitPoint = i;
    }
  }
  return maxDist;
}

/* Helper functions for vector operations */

double V2Length(Vector2 *a) {
  return sqrt(a->x * a->x + a->y * a->y);
}

double V2DistanceBetween2Points(Point2 *a, Point2 *b) {
  double dx = a->x - b->x;
  double dy = a->y - b->y;
  return sqrt(dx * dx + dy * dy);
}

void V2Normalize(Vector2 *a) {
  double len = V2Length(a);
  if (len != 0.0) {
    a->x /= len;
    a->y /= len;
  }
}

void V2Add(Vector2 *a, Vector2 *b, Vector2 *result) {
  result->x = a->x + b->x;
  result->y = a->y + b->y;
}

void V2Negate(Vector2 *a) {
  a->x = -a->x;
  a->y = -a->y;
}

Vector2 V2Scale(Vector2 *a, double s) {
  Vector2 result;
  result.x = a->x * s;
  result.y = a->y * s;
  return result;
}
