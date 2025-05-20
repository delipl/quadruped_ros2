// Copyright (c) 2024, Jakub Delicat
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

//
// Source of this file are templates in
// [RosTeamWorkspace](https://github.com/StoglRobotics/ros_team_workspace)
// repository.
//

#ifndef QUADRUPED_CONTROLLER__QUADRUPED_CONTROLLER_HPP_
#define QUADRUPED_CONTROLLER__QUADRUPED_CONTROLLER_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "quadruped_controller/quadruped_controller_parameters.hpp"
#include "quadruped_controller/visibility_control.h"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "pluginlib/class_loader.hpp"

// TODO(anyone): Replace with controller specific messages
#include "quadruped_msgs/msg/quadruped_control.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "kinematics_interface/kinematics_interface.hpp"
#include "quadruped_controller/leg.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace quadruped_controller {
// name constants for state interfaces
static constexpr size_t STATE_MY_ITFS = 0;

// name constants for command interfaces
static constexpr size_t CMD_MY_ITFS = 0;

// TODO(anyone: example setup for control mode (usually you will use some enums
// defined in messages)
enum class control_mode_type : std::uint8_t {
  FAST = 0,
  SLOW = 1,
};

using Point = geometry_msgs::msg::Point;

class QuadrupedController : public controller_interface::ControllerInterface {
public:
  QUADRUPED_CONTROLLER__VISIBILITY_PUBLIC
  QuadrupedController();

  QUADRUPED_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  QUADRUPED_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  QUADRUPED_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  QUADRUPED_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  QUADRUPED_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  QUADRUPED_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  QUADRUPED_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::return_type
  update(const rclcpp::Time &time, const rclcpp::Duration &period) override;
  // TODO(anyone): replace the state and command message types
  using ControllerReferenceMsg = quadruped_msgs::msg::QuadrupedControl;
  using ControllerModeSrvType = std_srvs::srv::SetBool;
  using ControllerStateMsg = sensor_msgs::msg::JointState;

protected:
  std::shared_ptr<quadruped_controller::ParamListener> param_listener_;
  quadruped_controller::Params params_;

  std::vector<std::string> state_joints_;

  // Command subscribers and Controller State publisher
  rclcpp::Subscription<ControllerReferenceMsg>::SharedPtr ref_subscriber_ =
      nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerReferenceMsg>>
      input_ref_;

  rclcpp::Service<ControllerModeSrvType>::SharedPtr
      set_slow_control_mode_service_;
  realtime_tools::RealtimeBuffer<control_mode_type> control_mode_;

  using ControllerStatePublisher =
      realtime_tools::RealtimePublisher<ControllerStateMsg>;
  using DataMsg = std_msgs::msg::Float64MultiArray;
  using DataRTPublisher = realtime_tools::RealtimePublisher<DataMsg>;

  // RT publishers
  std::unique_ptr<ControllerStatePublisher> state_rt_pub_;
  std::unique_ptr<DataRTPublisher> impedance_control_rt_pub_;
  std::unique_ptr<DataRTPublisher> target_position_rt_pub_;
  std::unique_ptr<DataRTPublisher> position_error_rt_pub_;
  std::unique_ptr<DataRTPublisher> velocity_error_rt_pub_;

  // Normal publishers
  rclcpp::Publisher<ControllerStateMsg>::SharedPtr state_pub_;
  rclcpp::Publisher<DataMsg>::SharedPtr impedance_control_normal_pub_;
  rclcpp::Publisher<DataMsg>::SharedPtr target_position_normal_pub_;
  rclcpp::Publisher<DataMsg>::SharedPtr position_error_normal_pub_;
  rclcpp::Publisher<DataMsg>::SharedPtr velocity_error_normal_pub_;

  // Kinematics interface plugin loader
  std::shared_ptr<
      pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>>
      kinematics_loader_;
  std::unique_ptr<kinematics_interface::KinematicsInterface> kinematics_;

private:
  // callback for topic interface
  QUADRUPED_CONTROLLER__VISIBILITY_LOCAL
  void reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg);

  std::vector<Leg> legs_map_;
};

} // namespace quadruped_controller

#endif // QUADRUPED_CONTROLLER__QUADRUPED_CONTROLLER_HPP_
