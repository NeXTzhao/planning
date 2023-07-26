
#include <tuple>
#include <vector>

class Bezier2Poly {
 public:
  struct Point {
    double x;
    double y;

    Point operator-(const Point &other) const;
  };

  std::vector<Bezier2Poly::Point> fitCurve(const std::vector<Point> &points, double maxError, std::vector<std::vector<Point>> &controlPoints, std::vector<std::vector<Point>> &curves);

 private:
  Point q(const std::vector<Point> &ctrlPoly, double t) const;
  Point qprime(const std::vector<Point> &ctrlPoly, double t) const;
  Point qprimeprime(const std::vector<Point> &ctrlPoly, double t) const;
  std::vector<std::vector<Point>> fitCubic(const std::vector<Point> &points, Point leftTangent,
                                           Point rightTangent, double error);
  static Point normalize(const Point &v);
  std::vector<Point> generateBezier(const std::vector<Point> &points, const std::vector<double> &parameters,
                                    Point leftTangent, Point rightTangent);
  std::vector<double> reparameterize(const std::vector<Point> &bezier, const std::vector<Point> &points,
                                     const std::vector<double> &parameters);
  double newtonRaphsonRootFind(const std::vector<Point> &bez, const Point &point, double u);
  std::tuple<double, int> computeMaxError(const std::vector<Point> &points,
                                          const std::vector<Point> &bez, const std::vector<double> &parameters);
  std::vector<std::vector<Point>> getFitCurves(const std::vector<std::vector<Point>> &controlPoints, const std::vector<Point> &points);
  std::vector<double> chordLengthParameterize(const std::vector<Point> &points);
};
