#include "common.h"
namespace routing {
// Point3d
bool Point3d::operator==(Point3d const& p) const {
  return std::sqrt(std::pow((p.x_ - x_), 2.0) + std::pow((p.y_ - y_), 2.0) +
                   std::pow((p.z_ - z_), 2.0)) < 0.001;
}

double Point3d::Distance(Point3d const& p) const {
  return std::sqrt(std::pow((p.x_ - x_), 2.0) + std::pow((p.y_ - y_), 2.0) +
                   std::pow((p.z_ - z_), 2.0));
}

std::string Point3d::ToString() const {
  return std::string("[") + std::to_string(x_) + std::string(", ") +
         std::to_string(y_) + std::string(", ") + std::to_string(z_) +
         std::string("]");
}

// Line2d
Line2d::Line2d(Point3d const& p1, Point3d const& p2) : start_(p1), end_(p2) {}

double Line2d::DotMul(Line2d const& l) const {
  double v1_x = end_.x_ - start_.x_;
  double v1_y = end_.y_ - start_.y_;
  double v2_x = l.end_.x_ - l.start_.x_;
  double v2_y = l.end_.y_ - l.start_.y_;
  return v1_x * v2_x + v1_y * v2_y;
}

double Line2d::Distance(Point3d const& p, bool is_projection) const {
  using point_type = boost::geometry::model::d2::point_xy<double>;
  using linestring_type = boost::geometry::model::linestring<point_type>;
  point_type pb(p.x_, p.y_);
  linestring_type line;
  line.push_back(point_type(start_.x_, start_.y_));
  line.push_back(point_type(end_.x_, end_.y_));
  if (is_projection) {
    return boost::geometry::distance(line, pb);
  }
  return -1.0;
}
bool Line2d::IsSameDirection(Line2d const& l) const { return DotMul(l) >= 0; }

std::string Line2d::ToString() const {
  return start_.ToString() + " -> " + end_.ToString();
}

// Map utils
MapUtils::MapUtils(std::shared_ptr<carla::client::Map> map_ptr)
    : map_ptr_(std::move(map_ptr)) {}

bool MapUtils::IsInJunction(Point3d const& p) {
  Location location(p.x_, p.y_, p.z_);
  auto waypoint = map_ptr_->GetWaypoint(location);
  return waypoint->IsJunction();
}

size_t MapUtils::FindBehindIndex(std::vector<Point3d> const& points,
                                 Point3d const& current_point) {
  if (points.size() <= 1) {
    return 0;
  }
  double min_distance = std::numeric_limits<double>::max();
//  double alter_min_distance = std::numeric_limits<double>::max();
  bool is_find = false;
  size_t index = 0;
  size_t alter_index = 0;
  for (size_t i = 0; i < points.size() - 1; i++) {
    Line2d line(points[i], points[i + 1]);
    double distance = line.Distance(current_point);
    // if (!IsWithinSegment(points[i], points[i+1], current_point)) {
    //   if (distance < alter_min_distance) {
    //     alter_index = i;
    //     alter_min_distance = distance;
    //   }
    //   continue;
    // }
    if (distance < min_distance) {
      is_find = true;
      min_distance = distance;
      index = i;
    }
  }
  if (!is_find) {
    std::cout << "not found!! use alter: " << alter_index
              << std::endl;
    return alter_index;
  }
  return index;
}

bool MapUtils::IsWithinSegment(Point3d const& p1, Point3d const& p2,
                               Point3d const& p) {
  Line2d out_line1(p1, p);
  Line2d out_line2(p2, p);
  Line2d in_line1(p1, p2);
  Line2d in_line2(p2, p1);
  return (out_line1.DotMul(in_line1) >= 0) && (out_line2.DotMul(in_line2) > 0);
}

// Odom::Odom(Point3d pos, double speed, double yaw)
//     : pos_(pos), speed_(speed), yaw_(yaw) {}

Node::Node(std::shared_ptr<carla::client::Waypoint> waypoint, double distance)
    : waypoint_(std::move(waypoint)), distance_(distance) {}

std::shared_ptr<carla::client::Waypoint> Node::GetWaypoint() const {
  return waypoint_;
}

double Node::GetDistance() const { return distance_; }

bool CompareRouterResults(std::vector<Point3d> const& p1s,
                          std::vector<Point3d> const& p2s) {
  if (p1s.size() != p2s.size()) {
    std::cout << "sizes are not the same" << std::endl;
    std::cout << "size 1: " << p1s.size() << "  size 2: " << p2s.size()
              << std::endl;
    return false;
  }
  for (size_t i = 0; i < p1s.size(); i++) {
    if (!(p1s[i] == p2s[i])) {
      std::cout << "No. " << i << " Point not same" << std::endl;
      return false;
    }
  }
  return true;
}

// std::string DBGPrint(carla::geom::Location const& location) {
//   return std::string("[") + std::to_string(location.x) + std::string(", ") +
//          std::to_string(location.y) + std::string(", ") +
//          std::to_string(location.z) + std::string("]");
// }

std::vector<Point3d> ConvertFromWaypointToPoint3d(
    std::vector<std::shared_ptr<carla::client::Waypoint>> const& waypoints) {
  std::vector<Point3d> result;
  for (auto const& waypoint_ptr : waypoints) {
    auto location = waypoint_ptr->GetTransform().location;
    result.emplace_back(location.x, location.y, location.z);
  }
  return result;
}

double Distance(carla::geom::Location const& p1,
                carla::geom::Location const& p2) {
  return std::sqrt(std::pow((p1.x - p2.x), 2.0) + std::pow((p1.y - p2.y), 2.0) +
                   std::pow((p1.z - p2.z), 2.0));
}

double Distance(carla::client::Waypoint const& p1,
                carla::client::Waypoint const& p2) {
  return Distance(p1.GetTransform().location, p2.GetTransform().location);
}

double Distance(std::shared_ptr<carla::client::Waypoint> const& p1,
                std::shared_ptr<carla::client::Waypoint> const& p2) {
  return Distance(*p1, *p2);
}

double Distance(Point3d const& p1, Point3d const& p2) {
  return std::sqrt(std::pow((p1.x_ - p2.x_), 2.0) +
                   std::pow((p1.y_ - p2.y_), 2.0) +
                   std::pow((p1.z_ - p2.z_), 2.0));
}

Line2d GetLineBetweenWaypoints(
    std::shared_ptr<carla::client::Waypoint> const& w1,
    std::shared_ptr<carla::client::Waypoint> const& w2) {
  Point3d start(w1->GetTransform().location.x, w1->GetTransform().location.y,
                w1->GetTransform().location.z);
  Point3d end(w2->GetTransform().location.x, w2->GetTransform().location.y,
              w2->GetTransform().location.z);
  return Line2d(start, end);
}
};  // namespace routing