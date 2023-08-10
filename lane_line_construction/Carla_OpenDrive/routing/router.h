#pragma once

// #include "carla/client/Actor.h"
// #include "carla/client/ActorList.h"
// #include "carla/client/BlueprintLibrary.h"
// #include "carla/client/Client.h"
// #include "carla/client/TrafficLight.h"
// #include "carla/client/World.h"
// #include <boost/shared_ptr.hpp>
// #include <boost/unordered_map.hpp>

#include "carla/client/Map.h"
#include "carla/client/Waypoint.h"
#include "carla/geom/Location.h"
#include "common.h"
#include <chrono>
#include <cmath>
#include <iostream>
#include <map>
#include <memory>
#include <queue>
#include <set>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace routing {
class Router {
 public:
  Router(Point3d start_point, Point3d end_point,
         std::shared_ptr<carla::client::Map> map_ptr);

  std::vector<Point3d> GetRoutePoints(double interval = 1.0);

 private:
  void SetPointInterval(double interval);
  bool AreNextWaypointsInJunction(
      std::shared_ptr<carla::client::Waypoint> const& waypoint) const;
  //  std::vector<utils::Point3d> RRT();
  //  std::vector<Point3d> Dijkstra();
  //  std::vector<Point3d> BFS();
  std::vector<Point3d> AStar();
  std::shared_ptr<carla::client::Map> map_ptr_;
  Point3d start_point_;
  Point3d end_point_;

  double point_interval_ = 2.0;
  double distance_threshold_ = 5.0;
};

};  // namespace  routing