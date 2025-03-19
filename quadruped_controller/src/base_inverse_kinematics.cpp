#include <algorithm>

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

class BaseInverseKinematics : public rclcpp::Node {

  using Point = geometry_msgs::msg::Point;

public:
  BaseInverseKinematics() : Node("base_controller_node") {
    quadruped_control_pub_ =
        this->create_publisher<quadruped_msgs::msg::QuadrupedControl>(
            "control_quadruped", 10 );

    base_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "quadruped_robot/command_base_pose", 10,
        std::bind(&BaseInverseKinematics::base_pose_callback, this,
                  std::placeholders::_1));

   
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    declare_parameter("use_hardware", false);
    use_hardware_ = get_parameter("use_hardware").as_bool();
  }

private:
  
  geometry_msgs::msg::TransformStamped create_footprint_transform(
      const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
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
    return transform;
  }

  void
  base_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    create_footprint_transform(msg);

    tf_broadcaster_->sendTransform(create_footprint_transform(msg));

    auto quad_control = inverse_base_kinematics(msg);
    quadruped_control_pub_->publish(quad_control);


  }

  quadruped_msgs::msg::QuadrupedControl inverse_base_kinematics(
      const geometry_msgs::msg::PoseStamped::SharedPtr msg) {

    std::array<Eigen::Vector3d, 4> foot_positions;

    quadruped_msgs::msg::QuadrupedControl quad_control;

    const auto default_leg_separation_x = 0.20;
    const auto default_leg_separation_y =  0.13;
    const auto default_base_link_height = -0.18;

    quad_control.header.stamp = now();
    quad_control.fl_foot_position.x =
        default_leg_separation_x - msg->pose.position.x;
    quad_control.fl_foot_position.y =
        default_leg_separation_y - msg->pose.position.y;
    quad_control.fl_foot_position.z =
        default_base_link_height - msg->pose.position.z;
    quad_control.fl_foot_in_contact.data = true;

    quad_control.fr_foot_position.x =
        default_leg_separation_x - msg->pose.position.x;
    quad_control.fr_foot_position.y =
        -default_leg_separation_y - msg->pose.position.y;
    quad_control.fr_foot_position.z =
        default_base_link_height - msg->pose.position.z;
    quad_control.fr_foot_in_contact.data = true;

    quad_control.rl_foot_position.x =
        -default_leg_separation_x - msg->pose.position.x;
    quad_control.rl_foot_position.y =
        default_leg_separation_y - msg->pose.position.y;
    quad_control.rl_foot_position.z =
        default_base_link_height - msg->pose.position.z;
    quad_control.rl_foot_in_contact.data = true;

    quad_control.rr_foot_position.x =
        -default_leg_separation_x - msg->pose.position.x;
    quad_control.rr_foot_position.y =
        -default_leg_separation_y - msg->pose.position.y;
    quad_control.rr_foot_position.z =
        default_base_link_height - msg->pose.position.z ;
    quad_control.rr_foot_in_contact.data = true;

    return quad_control;
  }

  rclcpp::Publisher<quadruped_msgs::msg::QuadrupedControl>::SharedPtr
  quadruped_control_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      base_pose_sub_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      position_control_pub_;

  std::vector<quadruped_controller::Leg> legs;
  std::vector<std::string> legs_names = {"front_left", "front_right",
                                         "rear_left", "rear_right"};

  trajectory_msgs::msg::JointTrajectory joint_trajectory;
  visualization_msgs::msg::MarkerArray marker_array_;

  geometry_msgs::msg::PoseStamped::SharedPtr base_command_pose_;
  std_msgs::msg::Float64MultiArray pos_control_;
  bool use_hardware_ = false;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BaseInverseKinematics>());
  rclcpp::shutdown();
  return 0;
}
