#pragma once

#include <string>
#include <vector>
struct LinePoint {
  double s;
  double x;
  double y;
  double theta;
  double kappa;
};

struct LaneStatus {
  int id{};
  double start_x_ = 0.0;
  double start_y_ = 0.0;
  double start_yaw_ = 0.0;
  double resolution_ = 0.1;
  double left_bound_offset_ = 3.5;
  double right_bound_offset_ = -3.5;
};

enum LaneNum {
  single_lane = 1,
  dual_lanes = 2,
  three_lanes = 3,
  four_lanes = 4
};
class LaneLine {
 public:
  explicit LaneLine(const std::vector<std::vector<double>> &config, const LaneStatus &lane_status);

  std::vector<LinePoint> getCenterLinePoints() const { return center_line_points_; }
  std::vector<LinePoint> getLeftBoundPoints() const { return left_bound_points_; }
  std::vector<LinePoint> getRightBoundP0ints() const { return right_bound_points_; }

 private:
  void generateCenterLineAndBounds();

  void writeJsonToFile(const std::string &filename);

 private:
  std::vector<std::vector<double>> config_;
  LaneStatus init_status_;
  std::vector<LinePoint> center_line_points_;
  std::vector<LinePoint> left_bound_points_;
  std::vector<LinePoint> right_bound_points_;

};
