#include "Lane.h"
#include "lane_config.h"

int main() {
  int lane_num = single_lane;
  initLane(lane_num);

  std::vector<std::shared_ptr<Lane>> lanes;
  lanes.reserve(laneStatus.size());
  for (const auto& status : laneStatus) {
    lanes.push_back(std::make_shared<Lane>(configs[status.id], status));
  }
  auto vis = std::make_shared<Visualization>(lanes);
//    vis->vis_lane();
  vis->vis_dynamic();
  plt::show();

  return 0;
}
