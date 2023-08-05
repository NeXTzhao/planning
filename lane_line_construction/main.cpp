#include "Lane.h"
#include "LaneConfig.h"

int main() {
  int lane_num = four_lanes;
  initLane(lane_num);

  std::vector<std::shared_ptr<Lane>> lanes;
  for (const auto& status : laneStatus) {
    lanes.push_back(std::make_shared<Lane>(configs[status.id], status));
  }
  auto vis = std::make_shared<Visualization>(lanes);
  //  vis->vis_lane();
  vis->vis_dynamic();
  plt::show();

  return 0;
}
