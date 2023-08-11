#include "router.h"
#include "bezier.h"

namespace routing {
Router::Router(Point3d start_point, Point3d end_point,
               std::shared_ptr<carla::client::Map> map_ptr)
    : map_ptr_(std::move(map_ptr)),
      start_point_(start_point),
      end_point_(end_point) {}

void Router::SetPointInterval(double interval) { point_interval_ = interval; }

std::vector<Point3d> Router::GetRoutePoints(double interval) {
  SetPointInterval(interval);
  return AStar();
}

std::vector<Point3d> Router::AStar() {
  std::set<Node, NodeComparator> open_set;
  std::set<Node, NodeComparator>
      closed_set;  // close_set的部分功能由g_score代替其功能
  std::unordered_map<uint64_t, uint64_t>
      waypoint_predecessor;  // 记录子节点，方便回溯找到整条路径
  std::unordered_map<uint64_t, double> g_score;
  std::unordered_map<uint64_t, std::shared_ptr<carla::client::Waypoint>>
      waypoint_map;
  // 获得起点和终点
  auto start_waypoint = map_ptr_->GetWaypoint(
      carla::geom::Location(start_point_.x_, start_point_.y_, start_point_.z_));
  auto end_waypoint = map_ptr_->GetWaypoint(
      carla::geom::Location(end_point_.x_, end_point_.y_, end_point_.z_));
  // 起始点的g代价设为0
  g_score.insert({start_waypoint->GetId(), 0.0});
  // 设置地图中的点的代价
  waypoint_map[start_waypoint->GetId()] = start_waypoint;
  // 建立节点，并压入open_list中
  Node start_node(start_waypoint, Distance(start_waypoint, end_waypoint));
  open_set.insert(start_node);

  while (!open_set.empty()) {
    auto current_node_it = open_set.begin();
    auto current_waypoint = current_node_it->GetWaypoint();

    open_set.erase(current_node_it);
    auto current_waypoint_id = current_waypoint->GetId();

    // 如果已扫描过，则跳过此次判断，并将其加入到closed_set中
    if (closed_set.find(*current_node_it) != closed_set.end()) {
      continue;
    }
    closed_set.emplace(*current_node_it);

    // 如果A*找到路径，则回溯找到整条路径
    if (Distance(end_waypoint, current_waypoint) < distance_threshold_) {
      std::cout << "A* found" << std::endl;
      std::vector<std::shared_ptr<carla::client::Waypoint>> result_waypoints;
      // 记录换道的索引id
      int LaneChangeId = 1;
      std::vector<int> LaneChangeFlag;
      constexpr double target_distance = 9.4; // 2倍的车身长度
      std::vector<Point3d> LaneChangePoints;

      while (waypoint_predecessor.find(current_waypoint_id) !=
             waypoint_predecessor.end()) {
        //        std::cout << "current_waypoint road id = "
        //                  << current_waypoint->GetRoadId();
        //        std::cout << " , lane id = " << current_waypoint->GetLaneId()
        //                  << std::endl;
        result_waypoints.push_back(current_waypoint);
        current_waypoint =
            waypoint_map[waypoint_predecessor.find(current_waypoint_id)
                             ->second];
        current_waypoint_id = current_waypoint->GetId();

        auto pre_waypoint = result_waypoints.back();
        if (pre_waypoint->GetRoadId() == current_waypoint->GetRoadId()) {
          if (pre_waypoint->GetLaneId() != current_waypoint->GetLaneId()) {
            //            std::cout << "换道思密达" << '\n';
            LaneChangeFlag.emplace_back(LaneChangeId);
          }
        }
        LaneChangeId++;
      }
      auto result = ConvertFromWaypointToPoint3d(result_waypoints);
      for (auto const& id : LaneChangeFlag) {
        std::cout << "lane change id : " << id << std::endl;
//        std::cout << "lane change point : " << result.at(id - 2).ToString()
//                  << std::endl;
//        std::cout << "lane change point : " << result.at(id - 1).ToString()
//                  << std::endl;
//        std::cout << "lane change point : " << result.at(id).ToString()
//                  << std::endl;
//        std::cout << "lane change point : " << result.at(id + 1).ToString()
//                  << std::endl;
        Bezier::Point p1, p2, p3, p4, ref1, ref2, ref3, ref4;
        ref1 = {result.at(id - 10).x_, result.at(id - 10).y_};
        ref2 = {result.at(id - 9).x_, result.at(id - 9).y_};
        ref3 = {result.at(id + 9).x_, result.at(id + 9).y_};
        ref4 = {result.at(id + 10).x_, result.at(id + 10).y_};
        auto theta0 = std::atan2(ref2.y - ref1.y, ref2.x - ref1.x);
        auto theta1 = std::atan2(ref4.y - ref3.y, ref4.x - ref3.x);

        p1 = ref2;
        p4 = ref3;
        p2 = {p1.x + target_distance * 0.5 * cos(theta0),
              p1.y + target_distance * 0.5 * sin(theta0)};
        p3 = {p4.x - target_distance * 0.5 * cos(theta1),
              p4.y - target_distance * 0.5 * sin(theta1)};
//        std::cout << "point1 = " << p1.x << " , " << p1.y << '\n';
//        std::cout << "point2 = " << p2.x << " , " << p2.y << '\n';
//        std::cout << "point3 = " << p3.x << " , " << p3.y << '\n';
//        std::cout << "point4 = " << p4.x << " , " << p4.y << '\n';

        Bezier::Bezier<3> cubicBezier({p1, p2, p3, p4});
        Bezier::Point point;
        for (double i = 0; i < 1; i += 0.05) {
          point = cubicBezier.valueAt(i);
          LaneChangePoints.emplace_back(point.x, point.y, 0);
        }
        result.erase(result.begin() + id - 9, result.begin() + id + 10);
        result.insert(result.begin() + id - 9, LaneChangePoints.begin(),
                      LaneChangePoints.end());
      }
//      std::reverse(result.begin(), result.end());
      return result;
    }

    // 寻找下一个航点和可换道的路径
    auto next_waypoints = current_waypoint->GetNext(point_interval_);
    auto current_lane_change = current_waypoint->GetLaneChange();

    // 判断下一个航点是否在路口
    if (!AreNextWaypointsInJunction(current_waypoint)) {
      // 右转弯
      if (current_lane_change ==
              carla::road::element::LaneMarking::LaneChange::Both ||
          current_lane_change ==
              carla::road::element::LaneMarking::LaneChange::Right) {
        // 获得右转方向的所有航点
        auto next_right_waypoint = current_waypoint->GetRight();
        if (next_right_waypoint != nullptr) {
          // 判断车道类型，只搜索 "driving" 的路段
          auto lane_type = next_right_waypoint->GetType();
          if (lane_type == carla::road::Lane::LaneType::Driving) {
            auto next_right_waypoints =
                next_right_waypoint->GetNext(point_interval_);
            for (auto const& right_waypoint : next_right_waypoints) {
              // 将右转方向上所有driving道路上的航点加入到待判断的结合中 复合
              next_waypoints.push_back(right_waypoint);
            }
          }
        }
      }

      // 左转同右转一样的方法处理
      if (current_lane_change ==
              carla::road::element::LaneMarking::LaneChange::Both ||
          current_lane_change ==
              carla::road::element::LaneMarking::LaneChange::Left) {
        auto next_left_waypoint = current_waypoint->GetLeft();
        auto prev_current_waypoint_id_it =
            waypoint_predecessor.find(current_waypoint_id);
        if (next_left_waypoint != nullptr &&
            prev_current_waypoint_id_it != waypoint_predecessor.end()) {
          auto lane_type = next_left_waypoint->GetType();
          if (lane_type == carla::road::Lane::LaneType::Driving) {
            //            auto prev_current_waypoint =
            //                waypoint_map[prev_current_waypoint_id_it->second];
            //            auto current_line =
            //            GetLineBetweenWaypoints(prev_current_waypoint,
            //                                                        current_waypoint);
            auto next_left_waypoints =
                next_left_waypoint->GetNext(point_interval_);
            for (auto const& left_waypoint : next_left_waypoints) {
              //              auto next_line =
              //                  GetLineBetweenWaypoints(next_left_waypoint,
              //                  left_waypoint);
              next_waypoints.push_back(left_waypoint);
            }
          }
        }
      }
    }

    for (auto const& next_waypoint : next_waypoints) {
      auto next_waypoint_id = next_waypoint->GetId();
      waypoint_map[next_waypoint_id] = next_waypoint;
      double tmp_g_score = g_score[current_waypoint_id] +
                           Distance(next_waypoint, current_waypoint);
      // 这一步未用到close_set, 其查重的作用由g_score代替了
      if (g_score.find(next_waypoint_id) == g_score.end() ||
          g_score[next_waypoint_id] > tmp_g_score) {
        g_score[next_waypoint_id] = tmp_g_score;
        waypoint_predecessor[next_waypoint_id] = current_waypoint_id;
        // 启发函数Distance为欧式距离
        double f_score = tmp_g_score + Distance(end_waypoint, next_waypoint);
        Node next_node(next_waypoint, f_score);
        auto next_node_it = open_set.find(next_node);
        if (next_node_it != open_set.end()) {
          open_set.erase(next_node_it);
        }
        open_set.insert(next_node);
      }
    }
  }
  std::cout << "A* not found" << std::endl;
  std::vector<Point3d> result;
  return result;
}

bool Router::AreNextWaypointsInJunction(
    std::shared_ptr<carla::client::Waypoint> const& waypoint) const {
  auto next_waypoints = waypoint->GetNext(3 * point_interval_);
  for (auto const& next_waypoint : next_waypoints) {
    if (next_waypoint->IsJunction()) {
      return true;
    }
  }
  return false;
}
}  // namespace routing