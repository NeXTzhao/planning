//
// Created by next on 23-8-5.
//

#pragma once
#include <memory>

#include "LaneLineConstruction.h"
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

class Lane {
 public:
  Lane(const std::vector<std::vector<double>>& config,
       const LaneStatus& laneStatus)
      : config_(config),
        laneStatus_(laneStatus) {
    laneLine_ = std::make_shared<LaneLine>(config_, laneStatus_);
  }

  void vis_lane() {
    auto center_line = laneLine_->getCenterLinePoints();
    auto left_bound = laneLine_->getLeftBoundPoints();
    auto right_bound = laneLine_->getRightBoundP0ints();

    std::vector<double> cen_x, cen_y, cen_kappa, left_x, left_y, right_x,
        right_y;

    for (int i = 0; i < center_line.size(); ++i) {
      cen_x.push_back(center_line[i].x);
      cen_y.push_back(center_line[i].y);
      cen_kappa.push_back(center_line[i].kappa);
      left_x.push_back(left_bound[i].x);
      left_y.push_back(left_bound[i].y);

      right_x.push_back(right_bound[i].x);
      right_y.push_back(right_bound[i].y);
    }
    std::string lineName = "Lane " + std::to_string(laneStatus_.id);

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

  std::shared_ptr<LaneLine> getLaneLine() const { return laneLine_; };

 private:
  std::vector<std::vector<double>> config_;
  LaneStatus laneStatus_;
  std::shared_ptr<LaneLine> laneLine_;
};

class Visualization {
 public:
  explicit Visualization(const std::vector<std::shared_ptr<Lane>>& lanes)
      : lanes_(lanes){};

  void vis_lane() const {
    for (const auto& lane : lanes_) { lane->vis_lane(); }
  }

  void vis_dynamic() const {
    const auto& center_line =
        lanes_.front()->getLaneLine()->getCenterLinePoints();
    for (int i = 0; i < center_line.size(); ++i) {
      plt::clf();
      vis_lane();
      double car_x = center_line[i].x;
      double car_y = center_line[i].y;
      double car_yaw = center_line[i].theta;
      drawCar(car_x, car_y, car_yaw);
      plt::grid(true);
      plt::axis("equal");
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
  }

 private:
  double car_length_ = 3.0;
  double car_width_ = 1.5;
  std::vector<std::shared_ptr<Lane>> lanes_;
};
