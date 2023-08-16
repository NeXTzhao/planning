#pragma once
#include <chrono>
#include <memory>
#include <random>
#include <utility>

#include "collision.h"
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

static void drawCar(double x, double y, double yaw, const std::string& color) {
  double car_length_ = 3.0;
  double car_width_ = 1.5;
  std::array<double, 5> car_x_{};
  std::array<double, 5> car_y_{};

  car_x_ = {x - car_length_ / 2, x + car_length_ / 2, x + car_length_ / 2, x - car_length_ / 2,
            x - car_length_ / 2};
  car_y_ = {y - car_width_ / 2, y - car_width_ / 2, y + car_width_ / 2, y + car_width_ / 2,
            y - car_width_ / 2};

  for (int i = 0; i < 5; ++i) {
    double temp_x = car_x_[i];
    car_x_[i] = (temp_x - x) * cos(yaw) - (car_y_[i] - y) * sin(yaw) + x;
    car_y_[i] = (temp_x - x) * sin(yaw) + (car_y_[i] - y) * cos(yaw) + y;
  }

  plt::plot({car_x_[0], car_x_[1], car_x_[2], car_x_[3], car_x_[4]},
            {car_y_[0], car_y_[1], car_y_[2], car_y_[3], car_y_[4]}, color);
  plt::axis("equal");
}

void drawObs(const std::vector<ObstaclePose>& obs_pose) {
  for (const auto& pose : obs_pose) { drawCar(pose.x, pose.y, pose.yaw, "b-"); }
}

class Simulation {
 private:
  Map map_;
  EgoVehicle ego_vehicle_;
  ObsVehicle obs_vehicle_;
  std::vector<LinePoint> trajectory_;

 public:
  explicit Simulation(Map map, EgoVehicle ego, ObsVehicle obs)
      : map_(std::move(map)),
        ego_vehicle_(ego),
        obs_vehicle_(std::move(obs)) {}

  static void vis_lane_curvature(const LaneLine& laneLine_) {
    auto lane = laneLine_.GetCenterLine();
    std::vector<double> kappa;
    for (const auto& point : lane) { kappa.push_back(point.kappa); }
    plt::figure();
    plt::named_plot("reference line kappa", kappa);
    plt::axis("equal");
    plt::grid(true);
    plt::show();
  }

  static void vis_lane(const LaneLine& lane) {
    auto center_line = lane.GetCenterLine();
    auto left_bound = lane.GetLeftBound();
    auto right_bound = lane.GetRightBound();
    int size = (int) center_line.size();

    std::vector<double> cen_x, cen_y, cen_kappa, left_x, left_y, right_x, right_y;

    for (int i = 0; i < size; ++i) {
      cen_x.push_back(center_line[i].x);
      cen_y.push_back(center_line[i].y);

      left_x.push_back(left_bound[i].x);
      left_y.push_back(left_bound[i].y);

      right_x.push_back(right_bound[i].x);
      right_y.push_back(right_bound[i].y);
    }
    //  std::string lineName = "Lane " + std::to_string(lane.GetId());
    //  plt::named_plot(lineName + " Center Line", cen_x, cen_y, "b--");
    //  plt::named_plot(lineName + " Left Bound", left_x, left_y, "g-");
    //  plt::named_plot(lineName + " Right Bound", right_x, right_y, "g-");

    plt::plot(cen_x, cen_y, "b--");
    plt::plot(left_x, left_y, "g-");
    plt::plot(right_x, right_y, "g-");

    plt::xlabel("X Coordinate");
    plt::ylabel("Y Coordinate");
    plt::title("Lane Visualization");

    plt::grid(true);
    plt::axis("equal");
  }

  static void vis_road(const Road& road) {
    auto lanes = road.GetLanes();
    for (const auto& lane : lanes) { vis_lane(lane); }
  }
  void vis_map() {
    auto roads = map_.GetRoads();
    for (const auto& road : roads) { vis_road(road); }
  }

  void vis_dynamic() {
    double ego_x;
    double ego_y;
    double ego_yaw;
    int start_index = 0;
    int end_index = 200;

    int temp_index = 0;
    addTrajectoryPoint(start_index, end_index);
    auto reference_line = map_.GetReferenceLine();
    auto obs_pose = obs_vehicle_.GetObsPose();
    for (; temp_index < (int) trajectory_.size(); temp_index += 5) {
      if (temp_index > (int) trajectory_.size() - 150) {
        start_index += temp_index;
        end_index = start_index + 200;
        addTrajectoryPoint(start_index, end_index);
        temp_index = 0;
      }
      plt::clf();
      vis_map();
      auto point = trajectory_[temp_index];
      ego_yaw = point.theta;
      ego_x = point.x + std::cos(ego_yaw);
      ego_y = point.y + std::sin(ego_yaw);

      ego_vehicle_.SetX(ego_x);
      ego_vehicle_.SetY(ego_y);
      ego_vehicle_.SetYaw(ego_yaw);

      drawCar(ego_x, ego_y, ego_yaw, "k-");
      //      drawObs(obs_vehicle_.GetObsPose());

      for (const auto& obs : obs_pose) {
        drawCar(obs.x, obs.y, obs.yaw, "k-");
        double t_min = 10.5;// 最小时间间隔
        bool isCollision = ego_vehicle_.checkCollision(obs, t_min);
//        std::cout << "ego :( " << ego_vehicle_.GetX() << " , " << ego_vehicle_.GetY()
//                  << " ) ,  obs :(" << obs.x << " , " << obs.y << " )" << std::endl;

        if (isCollision) {
          auto nearest_index = map_.cal_nearest_index({ego_vehicle_.GetX(), ego_vehicle_.GetY()});
//          std::cout << "nearest_index = " << nearest_index << std::endl;
          Point pb = {reference_line[nearest_index].x, reference_line[nearest_index].y};
          Point pi = {reference_line[nearest_index + 1].x, reference_line[nearest_index + 1].y};
          Point plc = {obs.x, obs.y};
          auto lane_change = std::make_shared<BezierLaneChange>(pb, pi, plc);
          auto trajectory = lane_change->GetTrajectory();
          //          trajectory_.clear();
//                    trajectory_ = trajectory;
          lane_change->vis_curve();
          lane_change.reset();
        }
      }

      double x_min = ego_x - 20.0;
      double x_max = ego_x + 50.0;
      double y_min = ego_y - 20.0;
      double y_max = ego_y + 50.0;

      std::vector<double> traj_x, traj_y;
      for (int j = temp_index + 5; j < (int) trajectory_.size(); ++j) {
        traj_x.push_back(trajectory_[j].x);
        traj_y.push_back(trajectory_[j].y);
      }
      plt::plot(traj_x, traj_y, "r-");

      plt::xlim(x_min, x_max);
      plt::ylim(y_min, y_max);
      plt::grid(true);
      plt::pause(0.0001);
    }
  }

  void addTrajectoryPoint(int start_index, int end_index) {
    auto ref = map_.GetReferenceLine();
    if (start_index < 0) start_index = 0;
    if (start_index >= (int) ref.size()) start_index = (int) ref.size() - 1;
    if (end_index <= 0) end_index = 1;
    if (end_index >= (int) ref.size()) end_index = (int) ref.size() - 1;
    trajectory_.clear();
    trajectory_.insert(trajectory_.begin(), ref.begin() + start_index, ref.begin() + end_index);
  }

  void addTrajectoryPoint(const std::vector<LinePoint>& trajectory) {
    trajectory_.clear();
    trajectory_ = trajectory;
  }
};
