// Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy; see <https://opensource.org/licenses/MIT>.

#pragma once

#include "carla/NonCopyable.h"
#include "carla/road/RoadTypes.h"
#include <string>
#include <vector>

namespace carla {
namespace road {

struct LaneValidity {
public:
  LaneValidity(LaneId from_lane, LaneId to_lane)
      : _from_lane(from_lane), _to_lane(to_lane) {}

  road::LaneId _from_lane;

  road::LaneId _to_lane;
};

} // namespace road
} // namespace carla
