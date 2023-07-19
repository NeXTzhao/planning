#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>

#include "discrete_points_reference_line_smoother.h"
#include "matplotlibcpp.h"

using namespace apollo;
using namespace planning;
namespace plt = matplotlibcpp;

void read_csv(std::vector<Eigen::Vector2d> &raw_points_) {
  std::ifstream inFile(
      "/home/next/planning/Notes/osqp_eigen/data/path_points.csv",
      std::ios::in);
  std::string lineStr;
  char delim = ',';
  if (!inFile) {
    std::cout << "打开文件失败！" << std::endl;
  }
  std::vector<double> raw_xs, raw_ys;
  getline(inFile, lineStr);
  while (getline(inFile, lineStr)) {
    std::stringstream ss(lineStr);
    std::string str;
    std::vector<double> points;
    while (getline(ss, str, delim)) {
      points.emplace_back(std::stod(str));
    }
    raw_xs.emplace_back(points.at(0));
    raw_ys.emplace_back(points.at(1));
  }

  raw_points_.resize(raw_xs.size());

  for (size_t i = 0; i < raw_xs.size(); ++i) {
    raw_points_[i] = {raw_xs[i], raw_ys[i]};
  }
}

void write_to_csv(std::vector<double> &r_x_, std::vector<double> &r_y_) {
  std::ofstream outFile;
  outFile.open("solve_path_points.csv", std::ios::out);
  outFile << "x_value" << ',' << "y_value" << '\n';
  for (size_t i = 0; i < r_x_.size(); ++i) {
    // outFile << r_x_.at(i) << ',' << r_y_.at(i) << '\n';
    outFile << "{" << r_x_.at(i) << ',' << r_y_.at(i) << "}," << '\n';
  }
  outFile.close();
}

int main() {
  std::vector<Eigen::Vector2d> raw_points_;
  //    SetPoints(raw_points_);
  read_csv(raw_points_);
  printf("raw_size=%zu\n", raw_points_.size());
  std::vector<double> solve_X, solve_Y;
  auto smoother = std::make_shared<DiscretePointsReferenceLineSmoother>();
  auto time2 = std::chrono::system_clock::now();

  smoother->Smooth(raw_points_, solve_X, solve_Y);
  auto time3 = std::chrono::system_clock::now();
  std::chrono::duration<double> diff2 = time3 - time2;
  std::cout << "Time for solving = " << diff2.count() * 1000 << " ms.\n";

  std::vector<double> ref_x, ref_y;
  for (const auto &item : raw_points_) {
    ref_x.push_back(item.x());
    ref_y.push_back(item.y());
  }

  write_to_csv(solve_X, solve_Y);
  plt::named_plot("refrenceline_XY", ref_x, ref_y, "*");
  plt::named_plot("discrete_points_xy", solve_X, solve_Y);
  plt::legend();
  plt::axis("equal");
  plt::show();

  return 0;
}