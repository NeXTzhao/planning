//#include "carla/road/MapBuilder.h"
//#include "carla/road/element/RoadInfoElevation.h"
//#include "carla/road/element/RoadInfoGeometry.h"
//#include "opendrive/src/carla/client/Map.h"
//#include "pugixml/pugixml.hpp"
#include <iostream>
#include <memory>
#include <string>

#include "math/FEM_Smoother/include/discrete_points_reference_line_smoother.h"
#include "routing/router.h"
#include "tool/matplotlibcpp.h"

// using namespace carla::road;
// using namespace carla::road::element;
// using namespace carla::geom;
using namespace routing;
using namespace apollo::planning;

namespace plt = matplotlibcpp;

int main() {
  //    auto world = client.GetWorld();
  //    auto map = world.GetMap();
  //    auto blueprint_library = world.GetBlueprintLibrary();
  //    auto vehicles = blueprint_library->Filter("vehicle");
  //    auto vehicle_bp = (*vehicles)[0];

  std::string map_file = "/home/next/Videos/Carla_OpenDrive/map/Town03.xodr";
  auto map = std::make_shared<carla::client::Map>(map_file);
    routing::Point3d p1(221,58, 0.0);
    routing::Point3d p2(232, 97, 0.0);
//  routing::Point3d p1(0.0, 0.0,0.0);
//  routing::Point3d p2(0.0,100, 0.0);
  routing::Router router(p1, p2, map);
  auto routePoints = router.GetRoutePoints(1.0);

  std::cout << "routing points size = " << routePoints.size() << std::endl;

  auto waypoints_list = map->_map.GenerateWaypoints(1.0);
  std::vector<double> x, y, x1, y1, smoother_x, smoother_y;
  for (auto &wp : waypoints_list) {
  
    auto item = map->_map.ComputeTransform(wp).location;
    // x.emplace_back(-item.y);
    // y.emplace_back(item.x);
    x.emplace_back(item.x);
    y.emplace_back(item.y);
    //        std::cout << "XY::" << item.x << "," << item.y << std::endl;
  }

  for (const auto &item : routePoints) {
    

    x1.emplace_back(item.x_);
    y1.emplace_back(item.y_);
    // x1.emplace_back(-item.y_);
    // y1.emplace_back(item.x_);

    // auto loc = carla::geom::Location(item.x_, item.y_, item.z_);
    //        std::cout << "XY::" << item.x_ << "," << item.y_ << std::endl;
    //        std::cout << "loc::" << loc.x << "," << loc.y << std::endl;
  }

  //    auto smoother =
  //            std::make_shared<apollo::planning::DiscretePointsReferenceLineSmoother>();
  //    smoother->Smooth(x1, y1, smoother_x, smoother_y);

  /******************************************************/
  plt::named_plot("map points", x, y, ".");
  //    plt::named_plot("smoother_routing", smoother_x, smoother_y);
  plt::named_plot("routing", x1, y1, "red");

  plt::legend();
  plt::axis("equal");
  plt::show();
  /******************************************************/
  return 0;
}
