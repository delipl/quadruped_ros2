#ifndef QUADRUPED_CONTROLLER_VISUALIZATION_HPP
#define QUADRUPED_CONTROLLER_VISUALIZATION_HPP

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace quadruped_controller {
class Visualization {
public:
  Visualization(rclcpp_lifecycle::LifecycleNode::SharedPtr node,
                const std::string &topic_name);
  visualization_msgs::msg::Marker
  createArrow(const geometry_msgs::msg::Point &start,
              const geometry_msgs::msg::Point &end, double scale,
              const std_msgs::msg::ColorRGBA &color,
              const std::string &frame_id = "base_link", std::size_t id = 0);

  visualization_msgs::msg::Marker createSphere(
      const Eigen::Vector3d &center, double scale,
      const std_msgs::msg::ColorRGBA &color,
      const std::string &frame_id = "base_link", std::size_t id = 0);

private:
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr
      node_; // Store the lifecycle node for timestamp
};

Visualization::Visualization(rclcpp_lifecycle::LifecycleNode::SharedPtr node,
                             const std::string &topic_name)
    : node_(node) {}

visualization_msgs::msg::Marker
Visualization::createArrow(const geometry_msgs::msg::Point &start,
                           const geometry_msgs::msg::Point &end,double scale,
                           const std_msgs::msg::ColorRGBA &color, 
                           const std::string &frame_id, std::size_t id) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = node_->now();
  marker.ns = "arrows";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  // marker.lifetime = rclcpp::Duration::from_nanoseconds(0);

  marker.scale.x = scale * 0.25; // Shaft diameter
  marker.scale.y = scale * 0.35; // Head diameter
  marker.scale.z = scale * 0.25; // Head length

  marker.color = color;

  marker.points.push_back(start);
  marker.points.push_back(end);

  return marker;
}

visualization_msgs::msg::Marker Visualization::createSphere(
    const Eigen::Vector3d &center, double scale, const std_msgs::msg::ColorRGBA &color,
    const std::string &frame_id, std::size_t id) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = node_->now();
  marker.ns = "spheres";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  // marker.lifetime = rclcpp::Duration::from_nanoseconds(0);

  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = scale;

  marker.color = color;

  marker.pose.position.x = center.x();
  marker.pose.position.y = center.y();
  marker.pose.position.z = center.z();
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  return marker;
} 
}// namespace quadruped_controller

#endif // QUADRUPED_CONTROLLER_VISUALIZATION_HPP