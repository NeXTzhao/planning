/*
 * @Author: wangdezhao
 * @Date: 2022-04-15 16:10:20
 * @LastEditTime: 2022-04-15 21:03:46
 * @FilePath: /osqp_eigen/include/circle.hpp
 * @Copyright:
 */
#include <cmath>
#include <vector>

class Circle {
 public:
  Circle(double r, double x, double y)
      : radius(r), centerCircleX_(x), centerCircleY_(y) {}

  int numPoints = 0;
  std::vector<double> getYvalue(double x) {
    std::vector<double> y;
    for (double i = x; i < x + radius; i += 0.5) {
      y.push_back(getYvalueLower(i));
      ++numPoints;
    }
    // for (double i = x; i < x + radius; i += 0.5) {
    //   y.push_back(getYvalueUpper(i));
    //   ++numPoints;
    // }
    return y;
  }

  std::vector<double> getYvalue(double x, double s) {
    std::vector<double> y;
    // for (double i = x; i < x + radius; i += s) {
    //   y.push_back(getYvalueLower(i));
    //   ++numPoints;
    // }
    for (double i = x; i < x + radius; i += s) {
      y.push_back(getYvalueUpper(i));
      ++numPoints;
    }
    return y;
  }

  std::vector<double> getYvalue1(double x) {
    std::vector<double> y;
    for (double j = x + radius; j > x; j -= 0.5) {
      y.push_back(getYvalueUpper(j));
      ++numPoints;
    }
    return y;
  }

  std::vector<double> getYvalue1(double x, double s) {
    std::vector<double> y;
    for (double j = x + radius; j > x; j -= s) {
      x_.push_back(j);
      y.push_back(getYvalueUpper(j));
      ++numPoints;
    }
    return y;
  }

 private:
  double getYvalueUpper(double x) {
    return centerCircleY_ +
           sqrt(radius * radius - (x - centerCircleX_) * (x - centerCircleX_));
  }

  double getYvalueLower(double x) {
    return centerCircleY_ -
           sqrt(radius * radius - (x - centerCircleX_) * (x - centerCircleX_));
  }

 private:
  double radius;
  double centerCircleX_;
  double centerCircleY_;
public:
  std::vector<double> x_;
};