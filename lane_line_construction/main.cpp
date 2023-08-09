#include "lane_config.h"
#include "trajectory_planning.h"
#include "vis.h"

int main() {
  int lane_num = dual_lanes;
  initLane(lane_num);

  std::vector<std::shared_ptr<VisLane>> lanes;
  lanes.reserve(laneStatus.size());
  for (const auto& status : laneStatus) {
    lanes.push_back(std::make_shared<VisLane>(configs[status.id], status));
  }

  auto vis = std::make_shared<VisLaneAndCar>(lanes);
  vis->vis_lane();
//  vis->vis_dynamic();
  auto bez_turn = std::make_shared<BezierTurn>();
  bez_turn->getBezierCurve();
  bez_turn->vis_curve();

  auto bez_LC = std::make_shared<BezierLaneChange>();

  bez_LC->getBezierCurve();
  bez_LC->vis_curve();

  auto bez_R = std::make_shared<BezierRoundabouts>();
  bez_R->getBezierCurve();
  bez_R->vis_curve();
  bez_R->printDebug();

  plt::show();
  return 0;
}
