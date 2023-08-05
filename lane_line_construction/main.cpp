#include "Lane.h"
#include "LaneConfig.h"

int main() {
  int lane_num = four_lanes;
  initLane(lane_num);
  std::vector<Lane> lanes;
  for (int i = 0; i < lane_num; ++i) {
    Lane lane(configs[i], laneStatus[i]);
    lanes.emplace_back(lane);
    lane.visualize();
  }
  plt::show();

  return 0;
}
