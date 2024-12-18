
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "quadruped_controller/leg.hpp"
#include "quadruped_msgs/msg/quadruped_control.hpp"
#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class InverseKinematicsTest : public rclcpp::Node {

  using Point = geometry_msgs::msg::Point;

public:
  InverseKinematicsTest() : Node("leg_controller_node") {
    quadruped_control_sub_ =
        this->create_subscription<quadruped_msgs::msg::QuadrupedControl>(
            "control_quadruped", 10,

            std::bind(&InverseKinematicsTest::control_callback, this,
                      std::placeholders::_1));

    base_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "quadruped_robot/command_base_pose", 10,
        std::bind(&InverseKinematicsTest::base_pose_callback, this,
                  std::placeholders::_1));

    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "/joint_states", 10);

    trajectory_pub_ =
        this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_trajectory_controller/joint_trajectory", 10);

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "quadruped_robot/visualization", 10);

    position_control_pub_ =
        this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/position_controller/commands", 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&InverseKinematicsTest::timer_callback, this));

    for (const auto &leg_name : legs_names) {
      quadruped_controller::Leg leg(leg_name);
      legs.push_back(leg);
    }

    declare_parameter("use_hardware", false);
    use_hardware_ = get_parameter("use_hardware").as_bool();
  }

private:
  void control_callback(const quadruped_msgs::msg::QuadrupedControl msg) {
    auto passive_joint_state = sensor_msgs::msg::JointState();
    auto active_joint_state = sensor_msgs::msg::JointState();
    active_joint_state.header.stamp = msg.header.stamp;
    passive_joint_state.header.stamp = msg.header.stamp;

    std::vector<Point> reference_foot_positions = {
        msg.fl_foot_position, msg.fr_foot_position, msg.rl_foot_position,
        msg.rr_foot_position};

    std::vector<Eigen::Vector3d> foot_positions;
    std::vector<bool> in_contact = {
        msg.fl_foot_in_contact.data, msg.fr_foot_in_contact.data,
        msg.rl_foot_in_contact.data, msg.rr_foot_in_contact.data};

    trajectory_msgs::msg::JointTrajectoryPoint point;
    auto l = joint_trajectory.points.size();
    double sec = (l + 1) * 0.008;
    point.time_from_start = rclcpp::Duration(0, sec * 1e9);

    std_msgs::msg::ColorRGBA blue;
    blue.r = 0.0f;
    blue.g = 0.0f;
    blue.b = 1.0f;
    blue.a = 1.0f;
    std_msgs::msg::ColorRGBA red;
    red.r = 1.0f;
    red.g = 0.0f;
    red.b = 0.0f;
    red.a = 1.0f;

    std_msgs::msg::ColorRGBA light_red;
    light_red.r = 1.0f;
    light_red.g = 0.5f;
    light_red.b = 0.5f;
    light_red.a = 1.0f;

    std_msgs::msg::ColorRGBA green;
    green.r = 0.0f;
    green.g = 1.0f;
    green.b = 0.0f;
    green.a = 1.0f;
    std_msgs::msg::Float64MultiArray pos_control;


    for (std::size_t i = 0; i < legs.size(); ++i) {
      auto &leg = legs[i];
      auto &reference_foot_position = reference_foot_positions[i];

      Eigen::Vector3d x;
      x << reference_foot_position.x, reference_foot_position.y,
          reference_foot_position.z;

      marker_array_.markers.push_back(create_foot_markers(
          legs_names[i] + "_foot_requested_state", x, blue));

      Eigen::Vector3d help;
      auto q = leg.inverse_kinematics(x);
      leg.set_joints_states(q);
      const auto foot_position = leg.forward_kinematics();
      foot_positions.push_back(foot_position);

      // std::cout << i << ": Reference: " << x.transpose() << std::endl;
      // std::cout << i << ": Inverse kinematics: " << q.transpose() <<
      // std::endl; std::cout << i << ": Forward kinematics: " <<
      // foot_position.transpose()
      //           << std::endl;

      if (in_contact[i]) {
        marker_array_.markers.push_back(create_foot_markers(
            legs_names[i] + "_foot_state", foot_position, light_red));
      } else {
        marker_array_.markers.push_back(create_foot_markers(
            legs_names[i] + "_foot_state", foot_position, red));
      }

      auto leg_active_joints = leg.get_active_joint_states();
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Point 0 of " << leg_active_joints[0].name << " "
                                       << leg_active_joints[0].position);
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Point 1 of " << leg_active_joints[1].name << " "
                                       << leg_active_joints[1].position);

      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Point 2 of " << leg_active_joints[2].name << " "
                                       << leg_active_joints[2].position);

      // joint_trajectory.points.push_back(point);

      for (const auto &joint_state : leg_active_joints) {
        if (joint_trajectory.joint_names.size() < 12) {
          joint_trajectory.joint_names.push_back(joint_state.name);
        }
        active_joint_state.name.push_back(joint_state.name);
        active_joint_state.position.push_back(joint_state.position);
        active_joint_state.velocity.push_back(joint_state.velocity);
        active_joint_state.effort.push_back(joint_state.effort);

        point.positions.push_back(joint_state.position);
        pos_control.data.push_back(joint_state.position);
      }

      // for (const auto &joint_state : joint_states) {
      //   passive_joint_state.name.push_back(joint_state.name);
      //   passive_joint_state.position.push_back(0.0 /*joint_state.position*/);
      //   passive_joint_state.velocity.push_back(joint_state.velocity);
      //   passive_joint_state.effort.push_back(joint_state.effort);

      //   passive_joint_state.name.push_back(joint_state.name);
      //   passive_joint_state.position.push_back(0.0 /*joint_state.position*/);
      //   passive_joint_state.velocity.push_back(joint_state.velocity);
      //   passive_joint_state.effort.push_back(joint_state.effort);
      // }
    }

    marker_array_.markers.push_back(
        create_surface_between_contacts(foot_positions, in_contact));

    position_control_pub_->publish(pos_control);
  }

  void
  base_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    auto transform = geometry_msgs::msg::TransformStamped();
    transform.header.stamp = now();
    transform.header.frame_id = "base_footprint";
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = msg->pose.position.x;
    transform.transform.translation.y = msg->pose.position.y;
    transform.transform.translation.z = msg->pose.position.z;

    transform.transform.rotation.x = msg->pose.orientation.x;
    transform.transform.rotation.y = msg->pose.orientation.y;
    transform.transform.rotation.z = msg->pose.orientation.z;
    transform.transform.rotation.w = msg->pose.orientation.w;

    tf_broadcaster_->sendTransform(transform);
    inverse_base_kinematics(msg);
  }

  std::array<Eigen::Vector3d, 4> inverse_base_kinematics(
      const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    std::array<Eigen::Vector3d, 4> foot_positions;

    quadruped_msgs::msg::QuadrupedControl quad_control;
    quad_control.header.stamp = now();
    quad_control.fl_foot_position.x = 0.2 - msg->pose.position.x;
    quad_control.fl_foot_position.y = 0.13 - msg->pose.position.y;
    quad_control.fl_foot_position.z = -msg->pose.position.z;

    quad_control.fr_foot_position.x = 0.2 - msg->pose.position.x;
    quad_control.fr_foot_position.y = -0.13 - msg->pose.position.y;
    quad_control.fr_foot_position.z = -msg->pose.position.z;

    quad_control.rl_foot_position.x = -0.2 - msg->pose.position.x;
    quad_control.rl_foot_position.y = 0.13 - msg->pose.position.y;
    quad_control.rl_foot_position.z = -msg->pose.position.z;

    quad_control.rr_foot_position.x = -0.2 - msg->pose.position.x;
    quad_control.rr_foot_position.y = -0.13 - msg->pose.position.y;
    quad_control.rr_foot_position.z = -msg->pose.position.z;

    control_callback(quad_control);

    return foot_positions;
  }

  void timer_callback() {
    publish_visualization();

    if (joint_trajectory.points.size() < 40 && use_hardware_) {
      return;
    }

    if (joint_trajectory.points.empty()) {
      return;
    }

    joint_trajectory.header.stamp = now();
    trajectory_pub_->publish(joint_trajectory);
    joint_trajectory.points.clear();
  }

  visualization_msgs::msg::Marker create_surface_between_contacts(
      const std::vector<Eigen::Vector3d> &reference_foot_positions,
      const std::vector<bool> &in_contact) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = now();
    marker.ns = "contact_surface";
    marker.id = marker_array_.markers.size();

    marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;

    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.5;

    for (auto i = 0u; i < in_contact.size(); ++i) {
      if (in_contact[i]) {
        geometry_msgs::msg::Point p1;
        p1.x = reference_foot_positions[i].x();
        p1.y = reference_foot_positions[i].y();
        p1.z = reference_foot_positions[i].z();
        marker.points.push_back(p1);
      }
    }

    return marker;
  }

  visualization_msgs::msg::Marker
  create_foot_markers(const std::string &ns,
                      const Eigen::Vector3d &foot_position,
                      std_msgs::msg::ColorRGBA color) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = now();
    marker.ns = ns;
    marker.id = marker_array_.markers.size();

    marker.type = visualization_msgs::msg::Marker::SPHERE;

    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = foot_position.x();
    marker.pose.position.y = foot_position.y();
    marker.pose.position.z = foot_position.z();

    marker.scale.x = 0.03;
    marker.scale.y = 0.03;
    marker.scale.z = 0.03;

    marker.color.r = color.r;
    marker.color.g = color.g;
    marker.color.b = color.b;
    marker.color.a = color.a;

    return marker;
  }

  void clear_markers() {
    visualization_msgs::msg::Marker marker;
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.push_back(marker);
    marker_pub_->publish(marker_array);
  }

  void publish_visualization() {

    marker_pub_->publish(marker_array_);
    marker_array_.markers.clear();
  }

  rclcpp::Subscription<quadruped_msgs::msg::QuadrupedControl>::SharedPtr
      quadruped_control_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      base_pose_sub_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      position_control_joint_state_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
      trajectory_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      position_control_pub_;

  std::vector<quadruped_controller::Leg> legs;
  std::vector<std::string> legs_names = {"front_left", "front_right",
                                         "rear_left", "rear_right"};

  trajectory_msgs::msg::JointTrajectory joint_trajectory;
  visualization_msgs::msg::MarkerArray marker_array_;

  bool use_hardware_ = false;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InverseKinematicsTest>());
  rclcpp::shutdown();
  return 0;
}
