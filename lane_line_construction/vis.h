#pragma once
#include <memory>
#include <utility>

#include "lane_line_construction.h"
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

//class VisLane {
// public:
//  VisLane(const std::vector<std::vector<double>>& config,
//          const LaneStatus& lane_status)
//      : config_(config),
//        laneStatus_(lane_status) {
//    laneLine_ = std::make_shared<LaneLine>(config_, laneStatus_);
//  }
//
//  void vis_lane() {
//    auto center_line = laneLine_->GetCenterLine();
//    auto left_bound = laneLine_->GetLeftBound();
//    auto right_bound = laneLine_->GetRightBound();
//
//    std::vector<double> cen_x, cen_y, cen_kappa, left_x, left_y, right_x,
//        right_y;
//
//    for (int i = 0; i < (int) center_line.size(); ++i) {
//      cen_x.push_back(center_line[i].x);
//      cen_y.push_back(center_line[i].y);
//      cen_kappa.push_back(center_line[i].kappa);
//      left_x.push_back(left_bound[i].x);
//      left_y.push_back(left_bound[i].y);
//
//      right_x.push_back(right_bound[i].x);
//      right_y.push_back(right_bound[i].y);
//    }
//    std::string lineName = "Lane " + std::to_string(laneStatus_.id);
//
//    //    plt::named_plot(lineName + " Center Line", cen_x, cen_y, "b--");
//    //    plt::named_plot(lineName + " Left Bound", left_x, left_y, "g-");
//    //    plt::named_plot(lineName + " Right Bound", right_x, right_y, "g-");
//
//    plt::plot(cen_x, cen_y, "b--");
//    plt::plot(left_x, left_y, "g-");
//    plt::plot(right_x, right_y, "g-");
//
//    plt::xlabel("X Coordinate");
//    plt::ylabel("Y Coordinate");
//    plt::title("Lane Visualization");
//
//    plt::grid(true);
//    plt::axis("equal");
//  }
//
//  void vis_curvature() {
//    auto lane = laneLine_->getCenterLinePoints();
//    std::vector<double> kappa;
//    for (const auto& point : lane) { kappa.push_back(point.kappa); }
//    plt::figure();
//    plt::named_plot("reference line kappa", kappa);
//    plt::axis("equal");
//    plt::grid(true);
//    plt::show();
//  }
//
//  std::shared_ptr<LaneLine> getLaneLine() const { return laneLine_; };
//
// private:
//  std::vector<std::vector<double>> config_;
//  LaneStatus laneStatus_;
//  std::shared_ptr<LaneLine> laneLine_;
//};
//
class Visualizer {
 private:
  map map_;

 public:
  explicit Visualizer(map map) : map_(std::move(map)) {}

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

    std::vector<double> cen_x, cen_y, cen_kappa, left_x, left_y, right_x,
        right_y;

    for (int i = 0; i < size; ++i) {
      cen_x.push_back(center_line[i].x);
      cen_y.push_back(center_line[i].y);

      left_x.push_back(left_bound[i].x);
      left_y.push_back(left_bound[i].y);

      right_x.push_back(right_bound[i].x);
      right_y.push_back(right_bound[i].y);
    }
    std::string lineName = "Lane " + std::to_string(lane.GetId());

    //    plt::named_plot(lineName + " Center Line", cen_x, cen_y, "b--");
    //    plt::named_plot(lineName + " Left Bound", left_x, left_y, "g-");
    //    plt::named_plot(lineName + " Right Bound", right_x, right_y, "g-");

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
    addTrajectoryPoint(0, 50);
    for (const auto& i : trajectory_) {
      plt::clf();
      //      vis_lane();
      vis_map();
      double car_x = i.x;
      double car_y = i.y;
      double car_yaw = i.theta;
      drawCar(car_x, car_y, car_yaw);

      double x_min = car_x - 30.0;
      double x_max = car_x + 60.0;
      double y_min = car_y - 10.0;
      double y_max = car_y + 10.0;

      std::vector<double> traj_x, traj_y;
      for (const auto& point : trajectory_) {
        traj_x.push_back(point.x);
        traj_y.push_back(point.y);
      }
      plt::plot(traj_x, traj_y, "r-");

      plt::xlim(x_min, x_max);// 设置 x 轴范围
      plt::ylim(y_min, y_max);// 设置 y 轴范围
      plt::grid(true);
      plt::pause(0.01);
    }
  }

 private:
  void drawCar(double x, double y, double yaw) const {
    double car_x[5] = {x - car_length_ / 2, x + car_length_ / 2,
                       x + car_length_ / 2, x - car_length_ / 2,
                       x - car_length_ / 2};
    double car_y[5] = {y - car_width_ / 2, y - car_width_ / 2,
                       y + car_width_ / 2, y + car_width_ / 2,
                       y - car_width_ / 2};

    for (int i = 0; i < 5; ++i) {
      double temp_x = car_x[i];
      car_x[i] = (temp_x - x) * cos(yaw) - (car_y[i] - y) * sin(yaw) + x;
      car_y[i] = (temp_x - x) * sin(yaw) + (car_y[i] - y) * cos(yaw) + y;
    }

    plt::plot({car_x[0], car_x[1], car_x[2], car_x[3], car_x[4]},
              {car_y[0], car_y[1], car_y[2], car_y[3], car_y[4]}, "k-");
    plt::axis("equal");
  }

  void addTrajectoryPoint(int start_index, int num_points) {
    auto points = map_.GetRoads().back().GetCenterLane().GetCenterLine();
    for (int i = start_index; i < num_points; ++i) {
      auto point = points[i];
      trajectory_.emplace_back(point);
    }
  }

 private:
  double car_length_ = 3.0;
  double car_width_ = 1.5;

  std::vector<LinePoint> trajectory_;
};
