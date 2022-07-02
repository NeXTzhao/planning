#include "circle.hpp"

#include <vector>

#include "matplotlibcpp.h"

/**
 * @brief 由于圆的特殊性，其圆弧上每一点的曲率均相等，故可以将等弧长与相同的角度相对应
 * @param  x                
 * @param  y                
 * @param  r                
 * @param  size             
 */
void div_circle(std::vector<double>& x, std::vector<double>& y, double r,
                double size)  // xy对应圆心坐标,r为半径,size用于设置划分的间距
{
  double angle_step = 0;  //一小步的弧度
  angle_step = size / r;
  double x_out, y_out;

  for (int i = 0; i < M_PI / angle_step; i++) {
    x_out = r * cos(i * angle_step);
    y_out = r * sin(i * angle_step);
    x.push_back(x_out);
    y.push_back(y_out);
  }
}

namespace plt = matplotlibcpp;
int main() {
  //   Circle test(14, 0, 0);
  //   std::vector<double> x, y;
  //   y = std::move(test.getYvalue(-14, 0.5));
  //   x = std::move(test.x_);

  std::vector<double> xs, ys;
  div_circle(xs, ys, 14, 1);
  std::cout << "point.size: " << xs.size() << '\n';
  plt::named_plot("x,y", xs, ys, "*");
  plt::legend();
  plt::axis("equal");
  plt::show();

  return 0;
}