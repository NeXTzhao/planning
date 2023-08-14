//#include "lane_config.h"
#include "collision.h"
//#include "trajectory_planning.h"
#include "simulation.h"

int main() {
  //  auto vis = std::make_shared<VisLaneAndCar>(lanes);
  //  vis->vis_lane();
  ////  vis->vis_dynamic();
//    auto bez_turn = std::make_shared<BezierTurn>();
//    bez_turn->GetTrajectory();
//    bez_turn->vis_curve();
  //
  //  auto bez_LC = std::make_shared<BezierLaneChange>();
  //
  //  bez_LC->getBezierCurve();
  //  bez_LC->vis_curve();
  //
    auto bez_R = std::make_shared<BezierRoundabouts>();
    bez_R->GetTrajectory();
    bez_R->vis_curve();
    bez_R->print_debug();
  std::vector<std::vector<double>> initialConfig{// straight line: length
                                                 {30},
                                                 // curve: Angle Radius
                                                 {90, 10},
                                                 {80},
                                                 {-180, 10},
                                                 {30}};
//
  Map map = Map(dual_lanes, initialConfig);
  EgoVehicle ego_car;        // 自动驾驶车辆
  ObsVehicle obs_car(map, 3);// 前车
  Simulation simulation(map, ego_car, obs_car);
  simulation.vis_map();
//  simulation.vis_dynamic();

  plt::show();
  return 0;
}
