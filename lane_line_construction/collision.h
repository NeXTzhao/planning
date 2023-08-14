#pragma once
#include <cmath>
#include <iostream>
#include <random>
#include <vector>

#include "lane_line_construction.h"
#include "matplotlibcpp.h"
#include "trajectory_planning.h"

namespace plt = matplotlibcpp;

struct ObstaclePose {
  double x;
  double y;
  double v = 0.0;
  double yaw;
};

class Vehicle {
 public:
  double x_{};
  double y_{};
  double v_{};
  double yaw_{};
  double max_a_ = 2.0;
  double car_length_ = 3.0;
  double car_width_ = 1.5;

 public:
  std::array<double, 5> car_x_{};
  std::array<double, 5> car_y_{};

  Vehicle() = default;
  //  Vehicle(double x, double y, double v) : x_(x), y_(y), v_(v) {}

 public:
  void SetX(double x) { x_ = x; }
  void SetY(double y) { y_ = y; }
  void SetV(double v) { v_ = v; }
  void SetYaw(double yaw) { yaw_ = yaw; }
  double GetX() const { return x_; }
  double GetY() const { return y_; }
  double GetV() const { return v_; }
  double GetYaw() const { return yaw_; }

  void getVertices(double x, double y, double yaw) {
    car_x_ = {x - car_length_ / 2, x + car_length_ / 2, x + car_length_ / 2, x - car_length_ / 2,
              x - car_length_ / 2};
    car_y_ = {y - car_width_ / 2, y - car_width_ / 2, y + car_width_ / 2, y + car_width_ / 2,
              y - car_width_ / 2};

    for (int i = 0; i < 5; ++i) {
      double temp_x = car_x_[i];
      car_x_[i] = (temp_x - x) * cos(yaw) - (car_y_[i] - y) * sin(yaw) + x;
      car_y_[i] = (temp_x - x) * sin(yaw) + (car_y_[i] - y) * cos(yaw) + y;
    }
  }
};

class EgoVehicle : public Vehicle {
 private:
 public:
  //  double GetDisSafe() const { return dis_safe; }

  //  EgoVehicle(double x, double y, double yaw) : Vehicle(x, y, yaw) {}

 public:
  bool checkCollision(const ObstaclePose& obs_pose, double t_min) {
    double d_c = 0.1;// 碰撞容差

    // 计算前车和自车之间的相对距离和相对速度
//    double d_rel = obs_pose.x - x_;
    double d_rel = std::hypot(obs_pose.x - x_, obs_pose.y - y_);
    double v_rel = v_ - obs_pose.v;

    // 计算最大安全距离
    double dis_safe = std::sqrt((2 * d_c * std::abs(v_) * t_min) + (0.5 * max_a_ * t_min * t_min));

    return d_rel < (dis_safe + v_rel * t_min);
  }
};

class ObsVehicle : public Vehicle {
 private:
  Map map_;
  int obs_num_{};
  std::vector<std::array<double, 5>> obs_car_x_{};
  std::vector<std::array<double, 5>> obs_car_y_{};

  std::vector<ObstaclePose> obs_pose_;

 public:
  const std::vector<ObstaclePose>& GetObsPose() const { return obs_pose_; }

 public:
  ObsVehicle() = default;
  ObsVehicle(Map map, int obs_num) : map_(std::move(map)), obs_num_(obs_num) {
    addRandomObstacles();
    getObsVertices();
  }

  void addRandomObstacles() {
    auto reference_line = map_.GetReferenceLine();
    double start_s = (reference_line.begin() + 10)->s;
    double end_s = (reference_line.end() - 10)->s;

    if (obs_num_ <= 0) return;
    if (start_s < 0) start_s = 0;
    if (end_s > reference_line.back().s) end_s = reference_line.back().s;

    for (int i = 0; i < obs_num_; ++i) {
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
        obs_pose_.push_back({pose_x, pose_y, 0.0, pose_yaw});
      }
    }
  }

  void getObsVertices() {
    for (const auto& obs : obs_pose_) {
      std::array<double, 5> car_x_{};
      std::array<double, 5> car_y_{};
      auto x = obs.x;
      auto y = obs.y;
      auto yaw = obs.yaw;
      car_x_ = {x - car_length_ / 2, x + car_length_ / 2, x + car_length_ / 2, x - car_length_ / 2,
                x - car_length_ / 2};
      car_y_ = {y - car_width_ / 2, y - car_width_ / 2, y + car_width_ / 2, y + car_width_ / 2,
                y - car_width_ / 2};

      for (int i = 0; i < 5; ++i) {
        double temp_x = car_x_[i];
        car_x_[i] = (temp_x - x) * cos(yaw) - (car_y_[i] - y) * sin(yaw) + x;
        car_y_[i] = (temp_x - x) * sin(yaw) + (car_y_[i] - y) * cos(yaw) + y;
      }
      obs_car_x_.push_back(car_x_);
      obs_car_y_.push_back(car_y_);
    }
  }

 private:
  static double generateRandomS(int seed, double start_s, double end_s) {
    // 使用固定的种子值
    std::mt19937 gen(seed + 80);
    std::uniform_real_distribution<> dis(start_s, end_s);
    return dis(gen);
  }

  static int findClosestIndex(const std::vector<LinePoint>& reference_line, double target_s) {
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