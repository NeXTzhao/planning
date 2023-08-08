#pragma once

#include <iostream>
#include <vector>

typedef std::vector<std::vector<double>> Config;

constexpr double lane_width = 3.75;

extern std::vector<Config> configs{};
void initLaneConfig(double lane_num) {
  Config initialConfig = {
      // straight line: length
      {150},
      // curve: Angle Radius
      {150, 50},
      {50},
      {-150, 50},
      {150}
      // ...
  };

  configs.clear();
  for (int i = 0; i < lane_num; ++i) {

    Config config = (i == 0) ? initialConfig : Config{};
    if (i != 0) {
      for (const auto& item : configs.back()) {
        if (item.size() == 1) {
          config.push_back(item);
        } else if (item.size() == 2) {
          double angle = item[0];
          double radius =
              angle < 0.0 ? item[1] - lane_width : item[1] + lane_width;
          config.push_back({angle, radius});
        } else {
          std::cout << "The size of config is wrong" << std::endl;
        }
      }
    }
    configs.emplace_back(std::move(config));
  }
}

extern std::vector<LaneStatus> laneStatus{};
void initLaneStartStatus(double lane_num) {
  laneStatus.clear();
  LaneStatus initStatus{};
  initStatus.left_bound_offset_ = lane_width * 0.5;
  initStatus.right_bound_offset_ = -lane_width * 0.5;
  for (int i = 0; i < lane_num; ++i) {
    const auto& prevStatus = (i == 0) ? initStatus : laneStatus.back();
    LaneStatus lane_status{i};
    lane_status.left_bound_offset_ = lane_width * 0.5;
    lane_status.right_bound_offset_ = -lane_width * 0.5;

    if (i != 0) {
      double offset = std::abs(prevStatus.left_bound_offset_) * 2;
      lane_status.start_y_ = prevStatus.start_y_ - offset;
    }
    laneStatus.emplace_back(lane_status);
  }
}

void initLane(double lane_num) {
  initLaneConfig(lane_num);
  initLaneStartStatus(lane_num);
}