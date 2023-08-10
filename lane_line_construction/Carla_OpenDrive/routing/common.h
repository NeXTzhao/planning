#pragma once

// #include "carla/client/Actor.h"
// #include "carla/client/ActorList.h"
// #include "carla/client/TrafficLight.h"
// #include "carla/client/World.h"
#include "carla/client/Map.h"
#include "carla/client/Waypoint.h"
#include "carla/geom/Location.h"
#include <boost/geometry.hpp>
#include <boost/optional.hpp>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <functional>
#include <limits>
#include <random>
#include <string>

using namespace carla::geom;
namespace routing {
class Point3d {
 public:
  Point3d() = default;
  Point3d(float x, float y, float z) : x_(x), y_(y), z_(z) {}
  float x_{0.0};
  float y_{0.0};
  float z_{0.0};

  bool operator==(Point3d const& p) const;

  double Distance(Point3d const& p) const;
  std::string ToString() const;
};

class Line2d {
 public:
  Line2d() = default;
  Line2d(Point3d const& p1, Point3d const& p2);
  Point3d start_;
  Point3d end_;

  double DotMul(Line2d const& l) const;
  double Distance(Point3d const& p, bool is_projection = true) const;
  bool IsSameDirection(Line2d const& l) const;
  std::string ToString() const;
};

class MapUtils {
 public:
  explicit MapUtils(std::shared_ptr<carla::client::Map> map_ptr);
  bool IsInJunction(Point3d const& p);
  static size_t FindBehindIndex(std::vector<Point3d> const& points,
                                Point3d const& current_point);

 private:
  static bool IsWithinSegment(Point3d const& p1, Point3d const& p2, Point3d const& p);
  //  bool IsWithinSegment(const Line2d& l, const Point3d& p);
  std::shared_ptr<carla::client::Map> map_ptr_{nullptr};
};

// class Odom {
//  public:
//   Odom() = default;
//   Odom(Point3d pos, double speed, double yaw);
//   Point3d GetPosition() const {return pos_;}
//   double GetSpeed() const {return speed_;}
//   double GetYaw() const {return yaw_;}
//  private:
//   Point3d pos_;
//   double speed_{0};
//   double yaw_{0};
// };

// double vector3DToDoulbe(carla::geom::Vector3D const& vec){
//   return std::sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
// }

class Node {
 public:
  Node() = default;
  Node(std::shared_ptr<carla::client::Waypoint> waypoint, double distance);
  std::shared_ptr<carla::client::Waypoint> GetWaypoint() const;
  double GetDistance() const;

 private:
  std::shared_ptr<carla::client::Waypoint> waypoint_{nullptr};
  double distance_{std::numeric_limits<double>::max()};
};

struct NodeComparator {
  bool operator()(Node const& n1, Node const& n2) const {
    if (n1.GetWaypoint()->GetId() == n2.GetWaypoint()->GetId()) {
      return false;
    } else {
      if (n1.GetDistance() == n2.GetDistance()) {
        return n1.GetWaypoint()->GetId() < n2.GetWaypoint()->GetId();
      }
    }
    return (n1.GetDistance() < n2.GetDistance());
  }
};

bool CompareRouterResults(std::vector<Point3d> const& p1s,
                          std::vector<Point3d> const& p2s);

// std::string DBGPrint(carla::geom::Location const& location);

std::vector<Point3d> ConvertFromWaypointToPoint3d(
    std::vector<std::shared_ptr<carla::client::Waypoint>> const& waypoints);

double Distance(carla::geom::Location const& p1,
                carla::geom::Location const& p2);

double Distance(carla::client::Waypoint const& p1,
                carla::client::Waypoint const& p2);

double Distance(std::shared_ptr<carla::client::Waypoint> const& p1,
                std::shared_ptr<carla::client::Waypoint> const& p2);

double Distance(Point3d const& p1, Point3d const& p2);

Line2d GetLineBetweenWaypoints(
    std::shared_ptr<carla::client::Waypoint> const& w1,
    std::shared_ptr<carla::client::Waypoint> const& w2);

};  // namespace routing

namespace std {
template <>
struct hash<routing::Point3d> {
  size_t operator()(routing::Point3d const& p) const {
    return (hash<double>()(p.x_)) ^ (hash<double>()(p.y_)) ^
           (hash<double>()(p.z_));
  }
};
}  // namespace std
