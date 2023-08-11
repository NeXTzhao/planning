#pragma once

#include <memory>
#include <string>
#include <vector>

#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

constexpr double lane_width = 3.75;

struct LinePoint {
  double s;
  double x;
  double y;
  double theta;
  double kappa;
};

struct LaneStatus {
  double start_x_ = 0.0;
  double start_y_ = 0.0;
  double start_yaw_ = 0.0;
  double resolution_ = 0.1;
  double left_bound_offset_ = lane_width * 0.5;
  double right_bound_offset_ = -lane_width * 0.5;
};

enum LaneNum {
  single_lane = 1,
  dual_lanes = 2,
  three_lanes = 3,
  four_lanes = 4
};

class LaneLine {
 private:
  int id_{};
  std::vector<double> config_;
  LaneStatus init_status_;

  std::vector<LinePoint> center_line_;
  std::vector<LinePoint> left_bound_;
  std::vector<LinePoint> right_bound_;

 public:
  int GetId() const { return id_; }
  const std::vector<LinePoint> &GetCenterLine() const { return center_line_; }
  const std::vector<LinePoint> &GetLeftBound() const { return left_bound_; }
  const std::vector<LinePoint> &GetRightBound() const { return right_bound_; }

 public:
  LaneLine() = default;
  explicit LaneLine(int id, const std::vector<double> &config,
                    const LaneStatus &lane_status);

 private:
  void generateCenterLineAndBounds();
  //  void writeJsonToFile(const std::string &filename);
};

struct Road {
  using laneConfig = std::vector<double>;
  using RoadConfig = std::vector<laneConfig>;

 private:
  int id;
  LaneLine centerLane;
  std::vector<LaneLine> lanes;
  std::shared_ptr<Road> pre_road;
  std::shared_ptr<Road> next_road;

 public:
  int GetId() const { return id; }
  const LaneLine &GetCenterLane() const { return centerLane; }
  const std::vector<LaneLine> &GetLanes() const { return lanes; }
  const std::shared_ptr<Road> &GetPreRoad() const { return pre_road; }
  const std::shared_ptr<Road> &GetNextRoad() const { return next_road; }

 public:
  explicit Road(int road_id, int lane_num,
                const std::vector<double> &road_config, LaneStatus &lane_status)
      : id(road_id) {
    generateLanes(lane_num, road_config, lane_status);
  }

 private:
  void generateLanes(int lane_num, const std::vector<double> &road_config,
                     LaneStatus &lane_status) {
    auto laneConfig = generateLaneConfigs(lane_num, road_config);
    auto laneStart = generateLaneStartStatus(lane_num, lane_status);
    for (int i = 0; i < lane_num; ++i) {
      generateLaneFromConfig(i, laneConfig[i], laneStart[i]);
    }
  }

  static std::vector<LaneStatus> generateLaneStartStatus(
      int lane_num, LaneStatus &lane_status) {
    std::vector<LaneStatus> laneStatus{};
    LaneStatus initStatus = lane_status;

    for (int i = 0; i < lane_num; ++i) {
      if (i == 0) { laneStatus.emplace_back(initStatus); }
      const auto &prevStatus = laneStatus.back();
      LaneStatus status{};
      status.start_x_ = prevStatus.start_x_
          + lane_width * std::cos(prevStatus.start_yaw_ - M_PI / 2.0);
      status.start_y_ = prevStatus.start_y_
          + lane_width * std::sin(prevStatus.start_yaw_ - M_PI / 2.0);
      status.start_yaw_ = prevStatus.start_yaw_;
      status.left_bound_offset_ = lane_width * 0.5;
      status.right_bound_offset_ = -lane_width * 0.5;

      laneStatus.emplace_back(status);
    }
    return laneStatus;
  }

  static RoadConfig generateLaneConfigs(
      int lane_num, const std::vector<double> &lane_config) {
    RoadConfig road_configs;

    for (int i = 0; i < lane_num; ++i) {
      if (i == 0) {
        road_configs.push_back(lane_config);
      } else {
        auto pre_config = road_configs.back();
        if (pre_config.size() == 1) {
          road_configs.push_back(pre_config);
        } else if (pre_config.size() == 2) {
          double angle = pre_config[0];
          double radius = angle < 0.0 ? pre_config[1] - lane_width
                                      : pre_config[1] + lane_width;
          laneConfig config{angle, radius};
          road_configs.push_back(config);
        }
      }
    }

    return road_configs;
  }

  void generateLaneFromConfig(int lane_id,
                              const std::vector<double> &lane_config,
                              const LaneStatus &lane_status) {
    LaneLine laneLine(lane_id, lane_config, lane_status);
    if (lane_id == 0) { centerLane = laneLine; }
    lanes.push_back(laneLine);
  }
};

struct Map {
 private:
  std::vector<Road> roads_;
  std::vector<LinePoint> reference_line_;
  LaneStatus initStatus{};

 public:
  const std::vector<Road> &GetRoads() const { return roads_; }
  const std::vector<LinePoint> &GetReferenceLine() const {
    return reference_line_;
  }

 public:
  Map(const int lane_num,
      const std::vector<std::vector<double>> &road_configs) {
    generateRoadsFromConfig(lane_num, road_configs, initStatus);
    generateReferenceLine();
  }

 private:
  void generateRoadsFromConfig(
      int lane_num, const std::vector<std::vector<double>> &road_configs,
      const LaneStatus &lane_status) {
    int id = 0;
    for (const auto &config : road_configs) {
      auto status = lane_status;
      if (!roads_.empty()) {
        auto next_start_point =
            roads_.back().GetCenterLane().GetCenterLine().back();
        status.start_x_ = next_start_point.x;
        status.start_y_ = next_start_point.y;
        status.start_yaw_ = next_start_point.theta;
      }
      Road road{id, lane_num, config, status};
      roads_.push_back(road);
      id++;
    }
  }

  void generateReferenceLine() {
    for (const auto &road : roads_) {
      auto ref = road.GetCenterLane().GetCenterLine();
      reference_line_.insert(reference_line_.end(), ref.begin(), ref.end());
    }
#if 0
    std::vector<double> x, y;
    for (const auto &ref : reference_line_) {
      std::cout << "ref = (" << ref.x << " , " << ref.y << ")" << std::endl;
      x.emplace_back(ref.x);
      y.emplace_back(ref.y);
    }
    plt::named_plot("ref", x, y, "r.");
    plt::axis("equal");

#endif
  }
};
