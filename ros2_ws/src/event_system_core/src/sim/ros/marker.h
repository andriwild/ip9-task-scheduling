#include "../../util/types.h"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

struct MapLocation
{
  std::string name;
  double x;
  double y;

  MapLocation(std::string n, double x_val, double y_val) : name(n), x(x_val), y(y_val) {}
};

class MarkerPublisher : public rclcpp::Node
{
public:
  MarkerPublisher() : Node("des_marker_publisher")
  {
    rclcpp::QoS qos_profile(1);
    qos_profile.transient_local();

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "visualization_marker_array", qos_profile);
  }

  void publishLocations(const std::vector<MapLocation>& locations)
  {
    visualization_msgs::msg::MarkerArray marker_array;
    int id_counter = 0;

    for (const auto& loc : locations) {
      visualization_msgs::msg::Marker arrow;
      arrow.header.frame_id = "map";
      arrow.header.stamp = this->now();
      arrow.ns = "locations_arrow";
      arrow.id = id_counter++;
      arrow.type = visualization_msgs::msg::Marker::ARROW;
      arrow.action = visualization_msgs::msg::Marker::ADD;

      arrow.pose.orientation.w = 1.0;

      geometry_msgs::msg::Point start_pt, end_pt;

      start_pt.x = loc.x;
      start_pt.y = loc.y;
      start_pt.z = 1.5;

      end_pt.x = loc.x;
      end_pt.y = loc.y;
      end_pt.z = 0.1;

      arrow.points.push_back(start_pt);
      arrow.points.push_back(end_pt);

      arrow.scale.x = 0.05;
      arrow.scale.y = 0.2;
      arrow.scale.z = 0.3;

      arrow.color.r = 0.0f;
      arrow.color.g = 1.0f;
      arrow.color.b = 1.0f;
      arrow.color.a = 1.0f;

      arrow.lifetime = rclcpp::Duration(0, 0);

      marker_array.markers.push_back(arrow);

      visualization_msgs::msg::Marker text;
      text.header.frame_id = "map";
      text.header.stamp = this->now();
      text.ns = "locations_names";
      text.id = id_counter++;
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::msg::Marker::ADD;

      text.pose.position.x = loc.x;
      text.pose.position.y = loc.y;
      text.pose.position.z = 1.8;

      text.scale.z = 0.3;
      text.color.r = 1.0f;
      text.color.g = 1.0f;
      text.color.b = 1.0f;
      text.color.a = 1.0f;
      text.text = loc.name;
      text.lifetime = rclcpp::Duration(0, 0);

      marker_array.markers.push_back(text);
    }

    marker_pub_->publish(marker_array);
  }

  void publishMarkers(std::map<std::string, des::Point> locationMap)
  {
    std::vector<MapLocation> rosLocations = {};
    for (auto [name, p] : locationMap) {
      rosLocations.emplace_back(name, p.m_x, p.m_y);
    }
    this->publishLocations(rosLocations);
  }

private:
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};
