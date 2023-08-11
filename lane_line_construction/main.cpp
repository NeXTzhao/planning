//#include "lane_config.h"
#include "trajectory_planning.h"
#include "vis.h"

int main() {
  //  auto vis = std::make_shared<VisLaneAndCar>(lanes);
  //  vis->vis_lane();
  ////  vis->vis_dynamic();
  //  auto bez_turn = std::make_shared<BezierTurn>();
  //  bez_turn->getBezierCurve();
  //  bez_turn->vis_curve();
  //
  //  auto bez_LC = std::make_shared<BezierLaneChange>();
  //
  //  bez_LC->getBezierCurve();
  //  bez_LC->vis_curve();
  //
  //  auto bez_R = std::make_shared<BezierRoundabouts>();
  //  bez_R->getBezierCurve();
  //  bez_R->vis_curve();
  //  bez_R->print_debug();
  std::vector<std::vector<double>> initialConfig{// straight line: length
                                                 {30},
                                                 // curve: Angle Radius
                                                 {90, 10},
                                                 {50},
                                                 {220, 20},
                                                 {30}};

  map map_use = map(dual_lanes, initialConfig);
  Visualizer visualizer(map_use);
  visualizer.vis_dynamic();


  plt::show();
  return 0;
}
