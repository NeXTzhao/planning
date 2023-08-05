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
       const LaneStatus& laneStatus) : config_(config),
                                       laneStatus_(laneStatus) {
    laneLine_ = std::make_shared<LaneLine>(config_, laneStatus_);
  }

  void visualize() {
    auto center_line = laneLine_->getCenterLinePoints();
    auto left_bound = laneLine_->getLeftBoundPoints();
    auto right_bound = laneLine_->getRightBoundP0ints();

    std::vector<double> cen_x, cen_y, cen_kappa, left_x, left_y, right_x, right_y;

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
    //
    plt::plot(cen_x, cen_y, "b--");
    plt::plot(left_x, left_y, "g-");
    plt::plot(right_x, right_y, "g-");

    plt::xlabel("X Coordinate");
    plt::ylabel("Y Coordinate");
    plt::title("Lane Visualization");

    plt::grid(true);
    plt::axis("equal");
    ;
//    plt::legend();
  }

 private:
  std::vector<std::vector<double>> config_;
  LaneStatus laneStatus_;
  std::shared_ptr<LaneLine> laneLine_;
};
