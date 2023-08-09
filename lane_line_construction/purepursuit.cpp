#include <cmath>
#include <iostream>
#include <vector>

#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

class Vehicle {
 private:
  double x_;    // Current x-coordinate
  double y_;    // Current y-coordinate
  double theta_;// Current heading angle

 public:
  Vehicle(double x, double y, double theta) : x_(x), y_(y), theta_(theta) {}

  // Method to perform path tracking control
  void trackPath(const std::vector<std::pair<double, double>>& path) {
    double speed = 1.0;             // Adjust as needed
    double lookahead_distance = 2.0;// Adjust as needed

    for (int i = 0; i < path.size(); ++i) {
      double target_x = path[i].first;
      double target_y = path[i].second;

      // Calculate desired heading angle (yaw) to face the target point
      double desired_yaw = atan2(target_y - y_, target_x - x_);

      // Calculate steering angle based on the desired yaw
      double steering_angle = desired_yaw - theta_;

      // Calculate distance to target point
      double distance = sqrt((target_x - x_) * (target_x - x_)
                             + (target_y - y_) * (target_y - y_));

      // Calculate lookahead point
      double lookahead_index = i + (lookahead_distance / distance);
      lookahead_index =
          std::min(lookahead_index, static_cast<double>(path.size() - 1));
      double lookahead_x = path[lookahead_index].first;
      double lookahead_y = path[lookahead_index].second;

      // Calculate steering angle to the lookahead point
      double lookahead_yaw = atan2(lookahead_y - y_, lookahead_x - x_);
      double lookahead_steering = lookahead_yaw - theta_;

      // Combine steering angles using a simple control law
      double combined_steering =
          0.5 * steering_angle + 0.5 * lookahead_steering;

      // Simulate applying control input (steering angle)
      // For simplicity, you can adjust the steering angle directly
      // and update the vehicle's heading angle.
      theta_ += combined_steering;

      // Simulate vehicle motion (update x and y based on heading angle)
      x_ += speed * cos(theta_);
      y_ += speed * sin(theta_);

      // Print the updated state
      std::cout << "Vehicle position: (" << x_ << ", " << y_ << ")"
                << std::endl;

      // Plot the vehicle and path
      //      plt::clf();
//      plt::plot({x_, target_x}, {y_, target_y}, "b-");// Plot path
      drawCar(x_, y_, theta_);                        // Plot vehicle
      plt::grid(true);
      plt::axis("equal");
//      plt::pause(0.01);
    }
  }

  static void drawCar(double x, double y, double yaw) {
    double car_length_ = 3.0;
    double car_width_ = 1.5;
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
};

int main() {
  std::vector<std::pair<double, double>> path;

  std::vector<double> x, y;
  int num_points = 30;
  for (int i = 0; i < num_points; ++i) {
    double t = i * M_PI * 0.3 / (num_points - 1);
    x.push_back(100 * std::cos(t));
    y.push_back(100 * std::sin(t));
    path.emplace_back(100 * std::cos(t), 100 * std::sin(t));
  }
  std::reverse(x.begin(),x.end());
  std::reverse(y.begin(),y.end());
  std::reverse(path.begin(), path.end());

  for (const auto& it : path) { printf("x=%f, y=%f\n", it.first, it.second); }
  // Create a vehicle instance
  Vehicle vehicle(60.0, 80.0, 0.0);

    vehicle.trackPath(path);

  plt::named_plot("ref", x, y, "r");
  plt::show();
  return 0;
}
