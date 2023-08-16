//#include "carla/road/MapBuilder.h"
//#include "carla/road/element/RoadInfoElevation.h"
//#include "carla/road/element/RoadInfoGeometry.h"
//#include "opendrive/src/carla/client/Map.h"
//#include "pugixml/pugixml.hpp"
#include <iostream>
#include <memory>
#include <string>

//#include "math/FEM_Smoother/include/discrete_points_reference_line_smoother.h"
#include "routing/router.h"
#include "tool/matplotlibcpp.h"
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
// using namespace carla::road;
// using namespace carla::road::element;
// using namespace carla::geom;
using namespace routing;
// using namespace apollo::planning;

namespace plt = matplotlibcpp;

int main(int argc, char **argv) {
  ros::init(argc, argv, "visualize_lanes");
  ros::NodeHandle n;
  ros::Publisher marker_pub =
      n.advertise<visualization_msgs::MarkerArray>("lane_marker_array", 10);
  // currently publishes every 10 seconds
  ros::Rate r(0.1);
  ros::NodeHandle nh;

  // marker arry to store all individual line strips
  visualization_msgs::MarkerArray line_strips;

  std::string map_file = "/home/vtd/Documents/xord_map/intersection.xodr";
  auto map = std::make_shared<carla::client::Map>(map_file);
  auto topo = map->GetTopology();
  auto lanemark = map->GetAllLandmarks();

  routing::Point3d p1(0.0, 0.0, 0.0);
  routing::Point3d p2(0.0, 100, 0.0);
  routing::Router router(p1, p2, map);
  auto routePoints = router.GetRoutePoints(1.0);

  std::cout << "routing points size = " << routePoints.size() << std::endl;

  auto waypoints_list = map->_map.GenerateWaypoints(1.0);
  std::vector<double> x, y, x1, y1, smoother_x, smoother_y;
  visualization_msgs::MarkerArray line_strips;

  while (ros::ok()) {
    int i = 0 visualization_msgs::Marker lineStripMain;
    lineStripMain.header.frame_id = "map";
    lineStripMain.header.stamp = ros::Time::now();
    lineStripMain.ns = "opendrive_visualize";
    lineStripMain.action = visualization_msgs::Marker::ADD;
    lineStripMain.pose.orientation.w = 1.0;
    lineStripMain.id = i;
    i++;
    lineStripMain.type = visualization_msgs::Marker::LINE_STRIP;

    // STRIP markers use only the x component of scale, for the line width
    lineStripMain.scale.x = 0.12;
    // STRIP is white
    lineStripMain.color.a = 1.0;
    lineStripMain.color.r = 1.0;
    lineStripMain.color.g = 1.0;
    lineStripMain.color.b = 1.0;
    for (auto &wp : waypoints_list) {
      auto item = map->_map.ComputeTransform(wp).location;
      // x.emplace_back(-item.y);
      // y.emplace_back(item.x);
      x.emplace_back(item.x);
      y.emplace_back(item.y);
      geometry_msgs::Point p;
      p.z = 0;
      p.x = item.x;
      p.y = item.y;
      lineStripMain.points.push_back(p);

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
  }

  line_strips.markers.push_back(lineStripMain);

  /*   /******************************************************/
  // plt::named_plot("map points", x, y, "g.");
  // //    plt::named_plot("smoother_routing", smoother_x, smoother_y);
  // plt::named_plot("routing", x1, y1, "red");

  // plt::legend();
  // plt::axis("equal");
  // plt::show();
  /******************************************************/
  return 0;
}
