#pragma once
#include <chrono>
#include <memory>
#include <random>
#include <utility>

#include "lane_line_construction.h"
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

class Car {
 private:
  double car_length_ = 3.0;
  double car_width_ = 1.5;
  double vehicle_speed_ = 1.0;
  std::chrono::high_resolution_clock::time_point last_update_time_;

 private:
  std::array<double, 5> car_x_{};
  std::array<double, 5> car_y_{};

 public:
  const std::array<double, 5>& GetCarX() const { return car_x_; }
  const std::array<double, 5>& GetCarY() const { return car_y_; }

 public:
  void drawCar(double x, double y, double yaw, const std::string& color) {
    car_x_ = {x - car_length_ / 2, x + car_length_ / 2, x + car_length_ / 2,
              x - car_length_ / 2, x - car_length_ / 2};
    car_y_ = {y - car_width_ / 2, y - car_width_ / 2, y + car_width_ / 2,
              y + car_width_ / 2, y - car_width_ / 2};

    for (int i = 0; i < 5; ++i) {
      double temp_x = car_x_[i];
      car_x_[i] = (temp_x - x) * cos(yaw) - (car_y_[i] - y) * sin(yaw) + x;
      car_y_[i] = (temp_x - x) * sin(yaw) + (car_y_[i] - y) * cos(yaw) + y;
    }

    plt::plot({car_x_[0], car_x_[1], car_x_[2], car_x_[3], car_x_[4]},
              {car_y_[0], car_y_[1], car_y_[2], car_y_[3], car_y_[4]}, color);
    plt::axis("equal");
  }

  void updateCarPosition(double& car_x, double& car_y, double car_yaw) {
    auto current_time = std::chrono::high_resolution_clock::now();
    auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(
        current_time - last_update_time_);
    double time_elapsed = elapsed_time.count() / 1000.0;// Convert to seconds
    last_update_time_ = current_time;

    car_x += vehicle_speed_ * time_elapsed * std::cos(car_yaw);
    car_y += vehicle_speed_ * time_elapsed * std::sin(car_yaw);
  }
};

class Obstacle : public Car {
 private:
  Map map_;

  struct ObstaclePose {
    double x;
    double y;
    double yaw;
  };

  std::vector<ObstaclePose> obs;

 public:
  const std::vector<ObstaclePose>& GetObs() const { return obs; }

 public:
  Obstacle() = default;
  explicit Obstacle(Map map) : map_(std::move(map)) {}

  void drawObs() {
    for (const auto& pose : obs) { drawCar(pose.x, pose.y, pose.yaw, "b-"); }
  }

  void addRandomObstacles(int num_obstacles) {
    auto reference_line = map_.GetReferenceLine();
    double start_s = (reference_line.begin() + 10)->s;
    double end_s = (reference_line.end() - 10)->s;

    if (num_obstacles <= 0) return;
    if (start_s < 0) start_s = 0;
    if (end_s > reference_line.back().s) end_s = reference_line.back().s;

    for (int i = 0; i < num_obstacles; ++i) {
      double random_s = generateRandomS(i, start_s, end_s);

      int closest_index = findClosestIndex(reference_line, random_s);
      if (closest_index != -1) {
        double pose_x = reference_line[closest_index].x;
        double pose_y = reference_line[closest_index].y;
        double pose_yaw = reference_line[closest_index].theta;
        //        std::cout << "start_s = " << start_s << " ,end_s = " << end_s
        //                  << " ,s = " << random_s << " ,index = " << closest_index
        //                  << " ,pose (" << pose_x << " , " << pose_y << " , " << pose_yaw
        //                  << " )" << std::endl;
        obs.push_back({pose_x, pose_y, pose_yaw});
      }
    }
  }

 private:
  static double generateRandomS(int seed, double start_s, double end_s) {
    // 使用固定的种子值
    std::mt19937 gen(seed + 80);
    std::uniform_real_distribution<> dis(start_s, end_s);
    return dis(gen);
  }

  static int findClosestIndex(const std::vector<LinePoint>& reference_line,
                              double target_s) {
    int closest_index = -1;
    double min_distance = std::numeric_limits<double>::max();
    for (int j = 0; j < (int) reference_line.size(); ++j) {
      double distance = std::abs(reference_line[j].s - target_s);
      if (distance < min_distance) {
        min_distance = distance;
        closest_index = j;
      }
    }
    return closest_index;
  }
};

class Visualizer {
 private:
  Map map_;
  Car car_;
  Obstacle obs_;
  std::vector<LinePoint> trajectory_;

 public:
  explicit Visualizer(Map map) : map_(std::move(map)) {
    Obstacle obs(map_);
    obs_ = std::move(obs);
  }

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
    double car_x;
    double car_y;
    double car_yaw;
    int start_index = 0;
    int end_index = 200;

    int temp_index = 0;
    addTrajectoryPoint(start_index, end_index);
    obs_.addRandomObstacles(3);
    for (; temp_index < (int) trajectory_.size(); temp_index += 5) {
      if (temp_index > (int) trajectory_.size() - 100) {
        start_index += temp_index;
        end_index = start_index + 200;
        addTrajectoryPoint(start_index, end_index);
        temp_index = 0;
      }
      plt::clf();
      vis_map();
      auto point = trajectory_[temp_index];
      car_x = point.x + std::cos(car_yaw);
      car_y = point.y + std::sin(car_yaw);
      car_yaw = point.theta;
      //      updateCarPosition(car_x, car_y, car_yaw);

      car_.drawCar(car_x, car_y, car_yaw, "k-");
      obs_.drawObs();

      double x_min = car_x - 20.0;
      double x_max = car_x + 50.0;
      double y_min = car_y - 20.0;
      double y_max = car_y + 50.0;

      std::vector<double> traj_x, traj_y;
      for (int j = temp_index; j < (int) trajectory_.size(); ++j) {
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

 private:
  void addTrajectoryPoint(int start_index, int end_index) {
    auto ref = map_.GetReferenceLine();
    if (start_index < 0) start_index = 0;
    if (start_index >= (int) ref.size()) start_index = (int) ref.size() - 1;
    if (end_index <= 0) end_index = 1;
    if (end_index >= (int) ref.size()) end_index = (int) ref.size() - 1;
    trajectory_.clear();
    trajectory_.insert(trajectory_.begin(), ref.begin() + start_index,
                       ref.begin() + end_index);
  }
};
