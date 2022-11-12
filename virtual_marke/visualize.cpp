#include "modules/visualization/visualize.h"

double gHeight = 0;

Visualize::Visualize() {
  pub_ = n_.advertise<visualization_msgs::Marker>("pub_data", 100);
  trans_.header.frame_id = "car_world";
  trans_.child_frame_id = "planning_car";
  geo_convert_.Reset(PLA_POC_INIT_LAT, PLA_POC_INIT_LON, PLA_POC_INIT_ALT);
  // init ego marker
  ego_marker_.header.frame_id = "car_world";
  ego_marker_.action = visualization_msgs::Marker::ADD;
  ego_marker_.lifetime = ros::Duration(PLA_RVIZ_LIFE_TIME);
  makeCube(ego_marker_, "ego", (int)VisType::ego);
  // init reference
  reference_line_marker_.header.frame_id = "car_world";
  reference_line_marker_.action = visualization_msgs::Marker::ADD;
  reference_line_marker_.lifetime = ros::Duration(PLA_RVIZ_LIFE_TIME);
  makeLine(reference_line_marker_, "reference_line",
           (int)VisType::reference_line);
  // init ego trajectory marker
  ego_trajectory_marker_.header.frame_id = "car_world";
  ego_trajectory_marker_.action = visualization_msgs::Marker::ADD;
  ego_trajectory_marker_.lifetime = ros::Duration(PLA_RVIZ_LIFE_TIME);
  makeLine(ego_trajectory_marker_, "ego_trajectory",
           (int)VisType::ego_trajectory);
  // init lane boundary marker
  lane_boundary_marker_.header.frame_id = "car_world";
  lane_boundary_marker_.action = visualization_msgs::Marker::ADD;
  lane_boundary_marker_.lifetime = ros::Duration(PLA_RVIZ_LIFE_TIME);
  makePoints(lane_boundary_marker_, "lane_boundary",
             (int)VisType::lane_boundary);
  lane_boundary_marker_.points.reserve(PLA_RVIZ_MAX_POINT);
  // init lane marker
  lane_marker_.header.frame_id = "car_world";
  lane_marker_.action = visualization_msgs::Marker::ADD;
  lane_marker_.lifetime = ros::Duration(PLA_RVIZ_LIFE_TIME);
  makePoints(lane_marker_, "lane", (int)VisType::lane_boundary);
  lane_marker_.points.reserve(PLA_RVIZ_MAX_POINT);
}

void Visualize::updateHDmap(const msgs::SMAO_HDMapMsg_st &msg) {
  double r, g, b;
  double x, y, z;
  geometry_msgs::Point p;
  int linkNum = msg.SMAO_Links_v.size();
  for (int32_t link_i = 0; link_i < linkNum; link_i++) {
    // for each boundary
    int boundaryNum = msg.SMAO_Links_v.at(link_i)
                          .SMAO_laneBoundaries_st.SMAO_LaneBoundaries_v.size();
    for (int32_t boundary_i = 0; boundary_i < boundaryNum; boundary_i++) {
      r = 1, g = 0, b = 0;
      setMarkerPose(lane_boundary_marker_, 0, 0, 0);
      lane_boundary_marker_.pose.orientation.w = 1;
      setMarkerScale(lane_boundary_marker_, 0.2, 0.2, 0);
      setMarkerColor(lane_boundary_marker_, r, g, b, 0.5);
      auto boundary_i_data =
          &(msg.SMAO_Links_v[link_i]
                .SMAO_laneBoundaries_st.SMAO_LaneBoundaries_v[boundary_i]);
      auto &BoundaryGeo = boundary_i_data->SMAO_BoundaryGeo_v;
      for (uint32_t k = 0; k < BoundaryGeo.size(); k++) {
        geo_convert_.Forward(BoundaryGeo[k].SMAO_Latitude_deg_d,
                             BoundaryGeo[k].SMAO_Longitude_deg_d, gHeight, x, y,
                             z);
        p.x = x;
        p.y = y;
        p.z = 0;
        if (lane_boundary_marker_.points.size() < PLA_RVIZ_MAX_POINT) {
          lane_boundary_marker_.points.emplace_back(p);
        } else {
          lane_boundary_marker_.points[lane_boundary_marker_idx_] = p;
        }
        lane_boundary_marker_idx_++;
        lane_boundary_marker_idx_ %= PLA_RVIZ_MAX_POINT;
      }
    }
    // for each lane
    int laneNum = msg.SMAO_Links_v.at(link_i)
                      .SMAO_laneCenterlines_st.SMAO_LaneCenterlines_v.size();
    for (uint32_t lane_i = 0; lane_i < laneNum; lane_i++) {
      r = 0, g = 1, b = 0;
      setMarkerPose(lane_marker_, 0, 0, 0);
      lane_marker_.pose.orientation.w = 1;
      setMarkerScale(lane_marker_, 0.2, 0.2, 0);
      setMarkerColor(lane_marker_, r, g, b, 0.5);
      auto lane_i_data =
          &(msg.SMAO_Links_v[link_i]
                .SMAO_laneCenterlines_st.SMAO_LaneCenterlines_v[lane_i]);
      auto &CenterGeo = lane_i_data->SMAO_CenterlineGeo_v;
      for (uint32_t i = 0; i < CenterGeo.size(); i++) {
        geo_convert_.Forward(CenterGeo[i].SMAO_Position_st.SMAO_Latitude_deg_d,
                             CenterGeo[i].SMAO_Position_st.SMAO_Longitude_deg_d,
                             gHeight, x, y, z);
        p.x = x;
        p.y = y;
        p.z = 0;
        if (lane_marker_.points.size() < PLA_RVIZ_MAX_POINT) {
          lane_marker_.points.emplace_back(p);
        } else {
          lane_marker_.points[lane_marker_idx_] = p;
        }
        lane_marker_idx_++;
        lane_marker_idx_ %= PLA_RVIZ_MAX_POINT;
      }
    }
  }
}

void Visualize::updatePosition(const geometry_msgs::PoseStamped &msg) {
  // update tf
  trans_.header.stamp = ros::Time::now();
  double x, y, z;
  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg.pose.orientation, quat);
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  setMarkerPose(ego_marker_, msg.pose.position.x, msg.pose.position.y, yaw);
  setMarkerScale(ego_marker_, 5, 2, 2);
  setMarkerColor(ego_marker_, 255, 255, 0, 0.4);
}

void Visualize::updatePrediction(const msgs::PredictionObstacles &msg) {
  object_marker_.clear();
  object_trajectory_marker_.clear();
  visualization_msgs::Marker marker;
  marker.header.frame_id = "car_world";
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration(PLA_RVIZ_LIFE_TIME);
  int dynamic_idx = 10;
  if (msg.obstacle_vector.size() == 0) {
    return;
  }
  // for each object
  for (uint32_t i = 0; i < msg.obstacle_vector.size(); i++) {
    // for object itself
    double x = msg.obstacle_vector[i].perception_obstacle.posX;
    double y = msg.obstacle_vector[i].perception_obstacle.posY;
    double yaw = msg.obstacle_vector[i].perception_obstacle.headingAngle;
    int r = i * 50;
    int b = 255 - i * 20;
    int g = 100 + i * 20;
    makeCube(marker, "object", dynamic_idx++);
    setMarkerPose(marker, x, y, yaw);
    setMarkerScale(marker, 5, 2, 2);
    setMarkerColor(marker, r, g, b, 0.4);
    object_marker_.emplace_back(marker);
    // for object trajectory
    if (!msg.obstacle_vector[i].trajectories.empty()) {
      uint32_t m = 0;
      auto probabilityMax = msg.obstacle_vector[i].trajectories[0].probability;
      for (uint32_t n = 0; n < msg.obstacle_vector[i].trajectories.size();
           n++) {
        if (probabilityMax <
            msg.obstacle_vector[i].trajectories[n].probability) {
          probabilityMax = msg.obstacle_vector[i].trajectories[n].probability;
          m = n;
        }
      }
      auto &trajectory_data = msg.obstacle_vector[i].trajectories[m].trajectory;
      if (trajectory_data.size() == 0) {
        continue;
      }
      makeLine(marker, "object_trajectory", dynamic_idx++);
      setMarkerPose(marker, 0, 0, 0);
      marker.pose.orientation.w = 1;
      setMarkerScale(marker, 0.1, 0.1, 0.1);
      setMarkerColor(marker, r, g, b, 1);
      geometry_msgs::Point p;
      for (uint32_t j = 0; j < trajectory_data.size(); j++) {
        x = trajectory_data[j].x;
        y = trajectory_data[j].y;
        yaw = trajectory_data[j].theta;
        p.x = x;
        p.y = y;
        p.z = 0;
        marker.points.push_back(p);
      }
      object_trajectory_marker_.emplace_back(marker);
    }
  }
}

void Visualize::updateReferenceLine(const std_msgs::Float32MultiArray &msg) {
  setMarkerPose(reference_line_marker_, 0, 0, 0);
  reference_line_marker_.pose.orientation.w = 1;
  setMarkerScale(reference_line_marker_, 0.3, 0.3, 0.3);
  setMarkerColor(reference_line_marker_, 179, 238, 58, 0.4);
  reference_line_marker_.points.clear();
  geometry_msgs::Point p;
  for (int i = 0; i < msg.data.size() / 2; i++) {
    p.x = msg.data[2 * i + 0];
    p.y = msg.data[2 * i + 1];
    p.z = 0;
    reference_line_marker_.points.push_back(p);
  }
}

void Visualize::updateTrajectory(const std_msgs::Float32MultiArray &msg) {
  setMarkerPose(ego_trajectory_marker_, 0, 0, 0);
  ego_trajectory_marker_.pose.orientation.w = 1;
  setMarkerScale(ego_trajectory_marker_, 0.3, 0.3, 0.3);
  setMarkerColor(ego_trajectory_marker_, 1, 0, 0, 0.8);
  ego_trajectory_marker_.points.clear();
  geometry_msgs::Point p;
  for (int i = 0; i < msg.data.size() / 9; i++) {
    p.x = msg.data[9 * i + 1];
    p.y = msg.data[9 * i + 2];
    p.z = 0;
    ego_trajectory_marker_.points.push_back(p);
  }
}

void Visualize::updatePath(const nav_msgs::Path &msg) {
  path_marker_.points.clear();
  geometry_msgs::Point p;
  auto poses = msg.poses;

  for (auto pose : poses) {
    p.x = pose.pose.position.x;
    p.y = pose.pose.position.y;
    p.z = 0;
    path_marker_.points.push_back(p);
  }
}

void Visualize::pubData() {
  // send transform
  broadcaster_.sendTransform(trans_);
  // send ego
  ego_marker_.header.stamp = ros::Time::now();
  pub_.publish(ego_marker_);
  // send reference line
  reference_line_marker_.header.stamp = ros::Time::now();
  pub_.publish(reference_line_marker_);
  // send ego trajectory
  ego_trajectory_marker_.header.stamp = ros::Time::now();
  pub_.publish(ego_trajectory_marker_);
  // send lane boundary
  lane_boundary_marker_.header.stamp = ros::Time::now();
  pub_.publish(lane_boundary_marker_);
  // send lane
  lane_marker_.header.stamp = ros::Time::now();
  pub_.publish(lane_marker_);
  // send object
  for (auto &it : object_marker_) {
    it.header.stamp = ros::Time::now();
    pub_.publish(it);
  }
  // send object trajectory
  for (auto &it : object_trajectory_marker_) {
    it.header.stamp = ros::Time::now();
    pub_.publish(it);
  }
};