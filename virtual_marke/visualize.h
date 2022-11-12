#pragma once

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>

#include <unordered_set>
#include <vector>

#include "modules/planning_adapter_ROS/interface_adapter/interface_ext_ROS.h"
#include "modules/planning_adapter_ROS/planning_def.h"
#include "modules/visualization/coord_converter/Geocentric.hpp"
#include "modules/visualization/coord_converter/LocalCartesian.hpp"

static void resetMarker(visualization_msgs::Marker &marker, std::string ns,
                        int id, int type) {
  marker.ns = ns;
  marker.id = id;
  marker.type = type;
  marker.points.clear();
}

static void makeCube(visualization_msgs::Marker &marker, std::string ns,
                     int id) {
  resetMarker(marker, ns, id, visualization_msgs::Marker::CUBE);
}

static void makePoints(visualization_msgs::Marker &marker, std::string ns,
                       int id) {
  resetMarker(marker, ns, id, visualization_msgs::Marker::POINTS);
}

static void makeLine(visualization_msgs::Marker &marker, std::string ns,
                     int id) {
  resetMarker(marker, ns, id, visualization_msgs::Marker::LINE_STRIP);
}

static void setMarkerPose(visualization_msgs::Marker &marker, double x,
                          double y, double yaw) {
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = 0;
  geometry_msgs::Quaternion q;
  q = tf::createQuaternionMsgFromRollPitchYaw(0, 0, yaw);
  marker.pose.orientation.x = q.x;
  marker.pose.orientation.y = q.y;
  marker.pose.orientation.z = q.z;
  marker.pose.orientation.w = q.w;
}

static void setMarkerScale(visualization_msgs::Marker &marker, double x,
                           double y, double z) {
  marker.scale.x = x;
  marker.scale.y = y;
  marker.scale.z = z;
}

static void setMarkerColor(visualization_msgs::Marker &marker, double r,
                           double g, double b, double a) {
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = a;
}

class Visualize {
 private:
  enum class VisType {
    ego,
    ego_trajectory,
    lane,
    lane_boundary,
    reference_line,
    object,
    object_trajectory,
  };

 public:
  Visualize();

  void updateHDmap(const msgs::SMAO_HDMapMsg_st &msg);

  void updatePosition(const geometry_msgs::PoseStamped &msg);

  void updatePrediction(const msgs::PredictionObstacles &msg);

  void updateReferenceLine(const std_msgs::Float32MultiArray &msg);

  void updateTrajectory(const std_msgs::Float32MultiArray &msg);

  void updatePath(const nav_msgs::Path &msg);

  void pubData();

 private:
  ros::NodeHandle n_;
  ros::Publisher pub_;

  GeographicLib::LocalCartesian geo_convert_;
  geometry_msgs::TransformStamped trans_;
  tf::TransformBroadcaster broadcaster_;

  visualization_msgs::Marker ego_marker_;
  visualization_msgs::Marker ego_trajectory_marker_;
  visualization_msgs::Marker reference_line_marker_;
  visualization_msgs::Marker lane_marker_;
  visualization_msgs::Marker lane_boundary_marker_;
  visualization_msgs::Marker path_marker_;
  std::vector<visualization_msgs::Marker> object_marker_;
  std::vector<visualization_msgs::Marker> object_trajectory_marker_;

  size_t lane_marker_idx_;
  size_t lane_boundary_marker_idx_;
};
