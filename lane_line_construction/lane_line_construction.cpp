#include "lane_line_construction.h"

#include <cmath>
//#include <fstream>
#include <iostream>
//#include <nlohmann/json.hpp>

LaneLine::LaneLine(int id, const std::vector<double> &config, const LaneStatus &lane_status)
    : id_(id),
      config_(config),
      init_status_(lane_status) {
  center_line_.clear();
  left_bound_.clear();
  right_bound_.clear();
  generateCenterLineAndBounds();

  //  std::string filename =
  //  "/home/vtd/Documents/planning/lane_line_construction/JSON/track.json";
  //  writeJsonToFile(filename);
}

void LaneLine::generateCenterLineAndBounds() {
  double x = init_status_.start_x_;
  double y = init_status_.start_y_;
  double yaw = init_status_.start_yaw_;
  double s = init_status_.start_s;
  double resolution = init_status_.resolution_;
  double left_bound_offset = init_status_.left_bound_offset_;
  double right_bound_offset = init_status_.right_bound_offset_;

  if (config_.size() == 1) {
    int num_steps = static_cast<int>(std::floor(config_[0] / resolution));
    for (int i = 0; i < num_steps; ++i) {
      if (i == 0) {
        center_line_.emplace_back(LinePoint{s, x, y, yaw, 0.0});
        double normal_yaw = yaw + M_PI / 2.0;
        double left_bound_x = x + left_bound_offset * std::cos(normal_yaw);
        double left_bound_y = y + left_bound_offset * std::sin(normal_yaw);
        double right_bound_x = x + right_bound_offset * std::cos(normal_yaw);
        double right_bound_y = y + right_bound_offset * std::sin(normal_yaw);

        left_bound_.emplace_back(LinePoint{s, left_bound_x, left_bound_y, yaw, 0.0});
        right_bound_.emplace_back(LinePoint{s, right_bound_x, right_bound_y, yaw, 0.0});
        continue;
      }
      x += resolution * std::cos(yaw);
      y += resolution * std::sin(yaw);
      s += resolution;
      center_line_.emplace_back(LinePoint{s, x, y, yaw, 0.0});

      double normal_yaw = yaw + M_PI / 2.0;
      double left_bound_x = x + left_bound_offset * std::cos(normal_yaw);
      double left_bound_y = y + left_bound_offset * std::sin(normal_yaw);
      double right_bound_x = x + right_bound_offset * std::cos(normal_yaw);
      double right_bound_y = y + right_bound_offset * std::sin(normal_yaw);

      left_bound_.emplace_back(LinePoint{s, left_bound_x, left_bound_y, yaw, 0.0});
      right_bound_.emplace_back(LinePoint{s, right_bound_x, right_bound_y, yaw, 0.0});
    }
  } else if (config_.size() == 2) {
    double degree = config_[0];
    double radius = config_[1];
    double angle = degree * M_PI / 180.0;
    int arc_direction = (angle < 0) ? -1 : 1;
    double arc_length = angle * radius;
    double kappa = (arc_length != 0) ? (1.0 / radius * arc_direction) : 0.0;
    // 为了让起始角度与x轴正方向平行，因为计算机在计算图形的时候是以y正方向为基准，这么做相当于把弧线旋转到以x轴正方向为基准
    double start_angle = yaw - M_PI / 2.0 * arc_direction;
    //    double end_angle = start_angle + angle;
    // 这里是将当前圆弧的切线方向旋转90°得到圆心角的度数，也就是圆弧法线方向的角度
    double center_yaw = yaw + M_PI / 2.0 * arc_direction;
    double xc = x + radius * std::cos(center_yaw);
    double yc = y + radius * std::sin(center_yaw);

    int point_count = std::abs(static_cast<int>(std::floor(arc_length / resolution)));
    double angle_increment = angle / point_count;

    for (int i = 0; i < point_count; ++i) {
      if (i == 0) {
        center_line_.emplace_back(LinePoint{s, x, y, yaw, 0.0});

        double normal_yaw = yaw + M_PI / 2.0;
        double left_bound_x = x + left_bound_offset * std::cos(normal_yaw);
        double left_bound_y = y + left_bound_offset * std::sin(normal_yaw);
        double right_bound_x = x + right_bound_offset * std::cos(normal_yaw);
        double right_bound_y = y + right_bound_offset * std::sin(normal_yaw);

        left_bound_.emplace_back(LinePoint{s, left_bound_x, left_bound_y, yaw, 0.0});
        right_bound_.emplace_back(LinePoint{s, right_bound_x, right_bound_y, yaw, 0.0});
        continue;
      }
      double current_angle = start_angle + i * angle_increment;
      x = xc + radius * std::cos(current_angle);
      y = yc + radius * std::sin(current_angle);
      s += resolution;
      yaw += angle_increment;
      center_line_.emplace_back(LinePoint{s, x, y, yaw, kappa});
      // 同理center_yaw原理一样
      double normal_yaw = yaw + M_PI / 2.0;
      double left_bound_x = x + left_bound_offset * std::cos(normal_yaw);
      double left_bound_y = y + left_bound_offset * std::sin(normal_yaw);
      double right_bound_x = x + right_bound_offset * std::cos(normal_yaw);
      double right_bound_y = y + right_bound_offset * std::sin(normal_yaw);

      left_bound_.emplace_back(LinePoint{s, left_bound_x, left_bound_y, yaw, 0.0});
      right_bound_.emplace_back(LinePoint{s, right_bound_x, right_bound_y, yaw, 0.0});
    }
  } else {
    std::cout << "lane defined error" << std::endl;
  }

  if (center_line_.size() > 1) {
    center_line_[0].kappa = center_line_[1].kappa;
    left_bound_[0].kappa = left_bound_[1].kappa;
    right_bound_[0].kappa = right_bound_[1].kappa;
  }
}

#if 0
void LaneLine::writeJsonToFile(const std::string &filename) {
  std::vector<double> center_x, center_y, center_kappa;
  for (const auto &point : center_line_points_) {
    center_x.push_back(point.x);
    center_y.push_back(point.y);
    center_kappa.push_back(point.kappa);
  }

  std::vector<double> left_bound_x, left_bound_y;
  for (const auto &point : left_bound_points_) {
    left_bound_x.push_back(point.x);
    left_bound_y.push_back(point.y);
  }

  std::vector<double> right_bound_x, right_bound_y;
  for (const auto &point : right_bound_points_) {
    right_bound_x.push_back(point.x);
    right_bound_y.push_back(point.y);
  }

  nlohmann::json j;
  j["X"] = center_x;
  j["Y"] = center_y;
  j["X_i"] = left_bound_x;
  j["Y_i"] = left_bound_y;
  j["X_o"] = right_bound_x;
  j["Y_o"] = right_bound_y;

  std::ofstream file(filename);
  if (file.is_open()) {
    file << j.dump(4);
    file.close();
    std::cout << "JSON data written to file: " << filename << std::endl;
  } else {
    std::cerr << "Failed to open file: " << filename << std::endl;
  }
}
#endif

