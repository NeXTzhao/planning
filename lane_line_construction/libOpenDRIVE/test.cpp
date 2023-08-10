#include <cstdio>
#include <iostream>
#include <vector>

#include "Lane.h"
#include "LaneSection.h"
#include "Math.hpp"
#include "Mesh.h"
#include "OpenDriveMap.h"
#include "Road.h"
#include "../../libOpenDRIVE-master/include/matplotlibcpp.h"

namespace plt = matplotlibcpp;

int main(int argc, char **argv) {
  //  if (argc < 2) {
  //    printf("ERROR: too few arguments\n");
  //    return -1;
  //  }
  odr::OpenDriveMap odr_map(
      "/home/next/Videos/Carla_OpenDrive/map/Town03.xodr");
  const double eps = 0.1;

  std::vector<odr::Vec3D> lane_pts;
  std::vector<odr::Vec3D> roadmark_broken_pts;
  std::vector<odr::Vec3D> roadmark_solid_pts;

  //  get xyz point for road coordinates
  //  odr::Road odr_road = odr_map.id_to_road.at("508");
  //  odr::Vec3D pt_xyz = odr_road.get_xyz(2.1 /*s*/, 1.0 /*t*/, 0.0 /*h*/);
  //  for (auto item : pt_xyz) {
  //    std::cout << "pt_xyz::" << item << '\n';
  //  }
  //
  //  // access road network attributes
  //  std::string lane_type =
  //  odr_road.get_lanesection(0.0).id_to_lane.at(-1).type; std::cout <<
  //  "lane_type::" << lane_type << '\n';

  // use routing graph
  odr::RoutingGraph routing_graph = odr_map.get_routing_graph();
  odr::LaneKey from("516" /*road id*/, 0.0 /*lane section s0*/, 1 /*lane id*/);
  odr::LaneKey to("501", 0.0, -1);
  std::vector<odr::LaneKey> path = routing_graph.shortest_path(from, to);
  std::vector<double> ref_x, ref_y;
  for (const auto &item : path) {
    printf("s = %0.2f\n", item.lanesection_s0);
    for (const auto &id : item.road_id) {
      printf("path_id: %d\n", id);
    }
  }

  odr_map.get_roads();
  for (const odr::Road &road : odr_map.get_roads()) {
    for (int i = 0; i < road.length; i += 1) {
      ref_x.emplace_back(road.ref_line.get_xyz(i).at(0));
      ref_y.emplace_back(road.ref_line.get_xyz(i).at(1));
    }
    printf("road_id: %s, road_name: %s, length: %.2f\n", road.id.c_str(),
           road.name.c_str(), road.length);
    for (const odr::LaneSection &lanesection : road.get_lanesections()) {
      const double s_start = lanesection.s0;
      const double s_end = road.get_lanesection_end(lanesection);
      for (const odr::Lane &lane : lanesection.get_lanes()) {

        std::cout << "lane.type::" << lane.type << std::endl;
        std::cout << "lane.id::" << lane.id << std::endl;

        if (lane.type == "driving") {
          auto lane_mesh = road.get_lane_mesh(lane, eps);
          lane_pts.insert(lane_pts.end(), lane_mesh.vertices.begin(),
                          lane_mesh.vertices.end());

          auto roadmarks = lane.get_roadmarks(s_start, s_end);
          for (const auto &roadmark : roadmarks) {
            std::cout << "roadmark.type::" << roadmark.type << std::endl;
            if (roadmark.type == "broken") {
              auto roadmark_mesh = road.get_roadmark_mesh(lane, roadmark, eps);
              roadmark_broken_pts.insert(roadmark_broken_pts.end(),
                                         roadmark_mesh.vertices.begin(),
                                         roadmark_mesh.vertices.end());
            }

            if (roadmark.type == "solid") {
              auto roadmark_mesh = road.get_roadmark_mesh(lane, roadmark, eps);
              roadmark_solid_pts.insert(roadmark_solid_pts.end(),
                                        roadmark_mesh.vertices.begin(),
                                        roadmark_mesh.vertices.end());
            }
          }
        }
      }
    }
  }

  std::vector<double> x_lane, y_lane;
  std::vector<double> x_road, y_road, x1_road, y1_road, x2_road, y2_road,
      x3_road, y3_road;
  for (int i = 0; i < roadmark_solid_pts.size(); ++i) {
    if (i % 2 == 0) {
      //      std::cout <<"i=偶数？"<<i<<std::endl;
      x_road.emplace_back(roadmark_solid_pts.at(i).at(0));
      y_road.emplace_back(roadmark_solid_pts.at(i).at(1));
    } else {
      //      std::cout <<"i=奇数？"<<i<<std::endl;
      x1_road.emplace_back(roadmark_solid_pts.at(i).at(0));
      y1_road.emplace_back(roadmark_solid_pts.at(i).at(1));
    }
  }
  for (int i = 0; i < roadmark_broken_pts.size(); ++i) {
    if (i % 2 == 0) {
      //      std::cout <<"i=偶数？"<<i<<std::endl;
      x2_road.emplace_back(roadmark_broken_pts.at(i).at(0));

      y2_road.emplace_back(roadmark_broken_pts.at(i).at(1));

    } else {
      //      std::cout <<"i=奇数？"<<i<<std::endl;
      x3_road.emplace_back(roadmark_broken_pts.at(i).at(0));
      y3_road.emplace_back(roadmark_broken_pts.at(i).at(1));
    }
  }

  for (auto item : lane_pts) {
    //  printf("lane_x = %0.2f,lane_y=%0.2f,lane_z=%0.2f\n", item.at(0),
    //  item.at(1),
    //  item.at(2));
    x_lane.emplace_back(item.at(0));
    y_lane.emplace_back(item.at(1));
  }
  printf("Finished, got %lu lane points, %lu roadmark points\n",
         lane_pts.size(), roadmark_solid_pts.size());

  //  plt::named_plot("lane", x_lane, y_lane,".");
//  std::sort(ref_x.begin(), ref_x.end());
//  std::sort(ref_y.begin(), ref_y.end());
  plt::named_plot("ref", ref_x, ref_y, ".");
  //    plt::named_plot("road", x_road, y_road,".");
  //    plt::named_plot("road", x1_road, y1_road, ".");
  //    plt::named_plot("road", x2_road, y2_road, ".");

  plt::legend();
  plt::axis("equal");
  plt::show();
  return 0;
}
