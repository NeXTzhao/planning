//
// Created by next on 23-8-5.
//

#pragma once

#include <iostream>
#include <vector>

typedef std::vector<std::vector<double>> Config;

constexpr double lane_width = 3.75;

extern std::vector<Config> configs{};
extern std::vector<LaneStatus> laneStatus{};

void initLaneConfig(double lane_num) {
  Config initialConfig = {
      // straight line: length
      {30},
      // curve: Angle Radius
      {-150, 40},
      {30},
      //      {150, 50},
      //      {150}
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
              angle < 0.0 ? item[1] + lane_width : item[1] - lane_width;
          config.push_back({angle, radius});
        } else {
          std::cout << "The size of config is wrong" << std::endl;
        }
      }
    }
    configs.emplace_back(std::move(config));
  }
}

void initLaneStartStatus(double lane_num) {
  laneStatus.clear();
  for (int i = 0; i < lane_num; ++i) {
    const auto& prevStatus = (i == 0) ? LaneStatus{} : laneStatus.back();
    LaneStatus lane_status{i};
    lane_status.left_bound_offset_ = -lane_width * 0.5;
    lane_status.right_bound_offset_ = lane_width * 0.5;

    if (i != 0) {
      double offset = std::abs(prevStatus.left_bound_offset_) * 2;
      double direction = configs[i][1][0];
      lane_status.start_y_ = direction < 0 ? offset : -offset;
      lane_status.start_y_ += prevStatus.start_y_;
    }

    laneStatus.emplace_back(lane_status);
  }
}

void initLane(double lane_num) {
  initLaneConfig(lane_num);
  initLaneStartStatus(lane_num);
}
