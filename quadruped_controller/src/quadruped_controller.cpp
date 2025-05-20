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

#include "quadruped_controller/quadruped_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"

namespace { // utility

// TODO(destogl): remove this when merged upstream
// Changed services history QoS to keep all so we don't lose any client service
// calls
static constexpr rmw_qos_profile_t rmw_qos_profile_services_hist_keep_all = {
    RMW_QOS_POLICY_HISTORY_KEEP_ALL,
    1, // message queue depth
    RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false};

using ControllerReferenceMsg =
    quadruped_controller::QuadrupedController::ControllerReferenceMsg;

// called from RT control loop
void reset_controller_reference_msg(
    const std::shared_ptr<ControllerReferenceMsg> &msg,
    const std::vector<std::string> &joint_names) {

  msg->header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
  msg->header.frame_id = "";
  msg->fl_foot_position.x = std::numeric_limits<double>::quiet_NaN();
  msg->fl_foot_position.y = std::numeric_limits<double>::quiet_NaN();
  msg->fl_foot_position.z = std::numeric_limits<double>::quiet_NaN();

  msg->fr_foot_position.x = std::numeric_limits<double>::quiet_NaN();
  msg->fr_foot_position.y = std::numeric_limits<double>::quiet_NaN();
  msg->fr_foot_position.z = std::numeric_limits<double>::quiet_NaN();

  msg->rl_foot_position.x = std::numeric_limits<double>::quiet_NaN();
  msg->rl_foot_position.y = std::numeric_limits<double>::quiet_NaN();
  msg->rl_foot_position.z = std::numeric_limits<double>::quiet_NaN();

  msg->rr_foot_position.x = std::numeric_limits<double>::quiet_NaN();
  msg->rr_foot_position.y = std::numeric_limits<double>::quiet_NaN();
  msg->rr_foot_position.z = std::numeric_limits<double>::quiet_NaN();

  msg->fl_foot_in_contact.data = false;
  msg->fr_foot_in_contact.data = false;
  msg->rl_foot_in_contact.data = false;
  msg->rr_foot_in_contact.data = false;
}

} // namespace

namespace quadruped_controller {
QuadrupedController::QuadrupedController()
    : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn QuadrupedController::on_init() {
  control_mode_.initRT(control_mode_type::FAST);

  try {
    param_listener_ =
        std::make_shared<quadruped_controller::ParamListener>(get_node());
  } catch (const std::exception &e) {
    fprintf(stderr,
            "Exception thrown during controller's init with message: %s \n",
            e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn QuadrupedController::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  params_ = param_listener_->get_params();

  if (!params_.state_joints.empty()) {
    state_joints_ = params_.state_joints;
  } else {
    state_joints_ = params_.joints;
  }

  if (params_.joints.size() != state_joints_.size()) {
    RCLCPP_FATAL(get_node()->get_logger(),
                 "Size of 'joints' (%zu) and 'state_joints' (%zu) parameters "
                 "has to be the same!",
                 params_.joints.size(), state_joints_.size());
    return CallbackReturn::FAILURE;
  }

  // topics QoS
  auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();

  // Reference Subscriber
  ref_subscriber_ = get_node()->create_subscription<ControllerReferenceMsg>(
      "~/reference", subscribers_qos,
      std::bind(&QuadrupedController::reference_callback, this,
                std::placeholders::_1));

  std::shared_ptr<ControllerReferenceMsg> msg =
      std::make_shared<ControllerReferenceMsg>();
  reset_controller_reference_msg(msg, params_.joints);
  input_ref_.writeFromNonRT(msg);

  auto set_slow_mode_service_callback =
      [&](const std::shared_ptr<ControllerModeSrvType::Request> request,
          std::shared_ptr<ControllerModeSrvType::Response> response) {
        if (request->data) {
          control_mode_.writeFromNonRT(control_mode_type::SLOW);
        } else {
          control_mode_.writeFromNonRT(control_mode_type::FAST);
        }
        response->success = true;
      };

  set_slow_control_mode_service_ =
      get_node()->create_service<ControllerModeSrvType>(
          "~/set_slow_control_mode", set_slow_mode_service_callback,
          rmw_qos_profile_services_hist_keep_all);

  try {
    // State publisher TODO: parametrize this
    state_pub_ = get_node()->create_publisher<ControllerStateMsg>(
        "/joint_states", rclcpp::SystemDefaultsQoS());
    state_rt_pub_ = std::make_unique<ControllerStatePublisher>(state_pub_);

    impedance_control_normal_pub_ = get_node()->create_publisher<DataMsg>(
        "~/impedance_controller/commands", rclcpp::SystemDefaultsQoS());
    impedance_control_rt_pub_ =
        std::make_unique<DataRTPublisher>(impedance_control_normal_pub_);

    target_position_normal_pub_ = get_node()->create_publisher<DataMsg>(
        "~/target_position", rclcpp::SystemDefaultsQoS());
    target_position_rt_pub_ =
        std::make_unique<DataRTPublisher>(target_position_normal_pub_);

    position_error_normal_pub_ = get_node()->create_publisher<DataMsg>(
        "~/position_error", rclcpp::SystemDefaultsQoS());
    position_error_rt_pub_ =
        std::make_unique<DataRTPublisher>(position_error_normal_pub_);
    velocity_error_normal_pub_ = get_node()->create_publisher<DataMsg>(
        "~/velocity_error", rclcpp::SystemDefaultsQoS());
    velocity_error_rt_pub_ =
        std::make_unique<DataRTPublisher>(velocity_error_normal_pub_);

  } catch (const std::exception &e) {
    fprintf(stderr,
            "Exception thrown during publisher creation at configure stage "
            "with message : %s \n",
            e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Load the differential IK plugin
  if (!params_.kinematics.plugin_name.empty()) {
    try {
      // Make sure we destroy the interface first. Otherwise we might run into a
      // segfault
      if (kinematics_loader_) {
        kinematics_.reset();
      }
      kinematics_loader_ = std::make_shared<
          pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>>(
          params_.kinematics.plugin_package,
          "kinematics_interface::KinematicsInterface");
      kinematics_ = std::unique_ptr<kinematics_interface::KinematicsInterface>(
          kinematics_loader_->createUnmanagedInstance(
              params_.kinematics.plugin_name));

      std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface>
          parameters_interface;
      if (!kinematics_->initialize(parameters_interface, "")) {
        return controller_interface::CallbackReturn::ERROR;
      }
    } catch (pluginlib::PluginlibException &ex) {
      RCLCPP_ERROR(rclcpp::get_logger("QuadrupedController"),
                   "Exception while loading the IK plugin '%s': '%s'",
                   params_.kinematics.plugin_name.c_str(), ex.what());
      return controller_interface::CallbackReturn::ERROR;
    }
  } else {
    RCLCPP_ERROR(
        rclcpp::get_logger("AdmittanceRule"),
        "A differential IK plugin name was not specified in the config file.");
    return controller_interface::CallbackReturn::ERROR;
  }

  // TODO: parametrize this
  std::vector<std::string> legs_names = {"front_left", "front_right",
                                         "rear_left", "rear_right"};
  for (const auto &leg_name : legs_names) {
    quadruped_controller::Leg leg(leg_name);
    legs_map_.push_back(leg);
  }
  // TODO(anyone): Reserve memory in state publisher depending on the message
  // type
  state_rt_pub_->lock();
  state_rt_pub_->msg_.name.resize(legs_map_.size() * 2);
  state_rt_pub_->msg_.position.resize(legs_map_.size() * 2);
  state_rt_pub_->msg_.velocity.resize(legs_map_.size() * 2);
  state_rt_pub_->msg_.effort.resize(legs_map_.size() * 2);
  state_rt_pub_->unlock();

  impedance_control_rt_pub_->lock();
  impedance_control_rt_pub_->msg_.data.resize(legs_map_.size() * 3);
  impedance_control_rt_pub_->unlock();
  target_position_rt_pub_->lock();
  target_position_rt_pub_->msg_.data.resize(legs_map_.size() * 3);
  target_position_rt_pub_->unlock();
  position_error_rt_pub_->lock();
  position_error_rt_pub_->msg_.data.resize(legs_map_.size() * 3);
  position_error_rt_pub_->unlock();
  velocity_error_rt_pub_->lock();
  velocity_error_rt_pub_->msg_.data.resize(legs_map_.size() * 3);
  velocity_error_rt_pub_->unlock();

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
QuadrupedController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type =
      controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_config.names.reserve(params_.joints.size());
  for (const auto &joint : params_.joints) {
    command_interfaces_config.names.push_back(joint + "/" +
                                              params_.command_interface);
  }

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
QuadrupedController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type =
      controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interfaces_config.names.reserve(state_joints_.size());
  for (const auto &joint : state_joints_) {
    for (const auto &interface_name : params_.state_interfaces) {
      state_interfaces_config.names.push_back(joint + "/" + interface_name);
    }
  }

  return state_interfaces_config;
}

void QuadrupedController::reference_callback(
    const std::shared_ptr<ControllerReferenceMsg> msg) {
  input_ref_.writeFromNonRT(msg);
}
controller_interface::CallbackReturn QuadrupedController::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // Set default value in command
  reset_controller_reference_msg(*(input_ref_.readFromRT()), state_joints_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn QuadrupedController::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // TODO(anyone): depending on number of interfaces, use definitions, e.g.,
  // `CMD_MY_ITFS`, instead of a loop
  for (size_t i = 0; i < command_interfaces_.size(); ++i) {
    command_interfaces_[i].set_value(std::numeric_limits<double>::quiet_NaN());
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
QuadrupedController::update(const rclcpp::Time &time,
                            const rclcpp::Duration &period) {
  // TODO(anyone): depending on number of interfaces, use definitions, e.g.,
  // `CMD_MY_ITFS`, instead of a loop
  auto msg = *(input_ref_.readFromRT());
  std::vector<Point> reference_foot_positions = {
      msg->fl_foot_position, msg->fr_foot_position, msg->rl_foot_position,
      msg->rr_foot_position};

  std::array<Eigen::Vector3d, 4> foot_positions;
  for (std::size_t i = 0; i < legs_map_.size(); ++i) {
    auto &leg = legs_map_[i];

    Eigen::Vector3d q;
    Eigen::Vector3d dq;

    // Throught joints. Remember there are positions, velocities and efforts
    for (std::size_t j = 0; j < 3; ++j) {
      q[j] = state_interfaces_[i * 9 + j * 3].get_value();
      dq[j] = state_interfaces_[i * 9 + j * 3 + 1].get_value();
    }
    leg.set_joints_states(q, dq);
    auto foot_position = leg.forward_kinematics();
    foot_positions[i] = foot_position;
  }

  // Inverse kinematics
  std::array<double, 12> target_positions;
  for (std::size_t i = 0; i < legs_map_.size(); ++i) {
    auto &leg = legs_map_[i];
    auto reference_foot_position = reference_foot_positions[i];

    Eigen::Vector3d x;
    x << reference_foot_position.x, reference_foot_position.y,
        reference_foot_position.z;

    auto q = leg.inverse_kinematics(x);
    target_positions[i * 3] = q[0];
    target_positions[i * 3 + 1] = q[1];
    target_positions[i * 3 + 2] = q[2];
  }

  if (msg->header.stamp == rclcpp::Time(0, 0, RCL_ROS_TIME)) {
    for (size_t i = 0; i < legs_map_.size(); ++i) {
      target_positions[i * 3] = 0.0;
      target_positions[i * 3 + 1] = -1.0;
      target_positions[i * 3 + 2] = 1.4;
      RCLCPP_WARN(get_node()->get_logger(),
                  "Target positions are set to default values");
    }
  }

  // TODO: parametrize this
  std::array<double, 3> Kp = {
      95.92020756982738, 51.207090397090106, 51.207090397090106,
      // 34.20749226006192
  };
  std::array<double, 3> Kd = {
      95.92020756982738 * 0.022526014640095713,
      51.207090397090106 * 0.023184721126433695,
      51.207090397090106 * 0.023184721126433695,
      // 34.20749226006192 * 0.03655939621794512
  };
  std::array<double, 3> max_torque = {50.0, 30.0, 50.0};
  std::array<double, 3> stand_up_pos = {0.0, -1.0, 1.4};

  // TODO: Use chainable controller with pid controller
  //  Joint control loop
  size_t index = 0;
  for (size_t i = 0; i < legs_map_.size(); ++i) {
    auto joint_states = legs_map_[i].get_joints_states();

    for (std::size_t j = 0; j < 3; ++j) {

      auto current_position = joint_states[j].position;
      auto current_velocity = joint_states[j].velocity;
      const auto data_index = i * 3 + j;

      position_error_rt_pub_->lock();
      velocity_error_rt_pub_->lock();
      target_position_rt_pub_->lock();
      position_error_rt_pub_->msg_.data[data_index] =
          target_positions[data_index] - current_position;
      velocity_error_rt_pub_->msg_.data[data_index] = 0.0 - current_velocity;
      target_position_rt_pub_->msg_.data[data_index] =
          target_positions[data_index];

      impedance_control_rt_pub_->msg_.data[data_index] =
          Kp[j] * position_error_rt_pub_->msg_.data[data_index] +
          Kd[j] * velocity_error_rt_pub_->msg_.data[data_index];

      if (std::isnan(impedance_control_rt_pub_->msg_.data[data_index])) {
        impedance_control_rt_pub_->msg_.data[data_index] = 0.0;
        RCLCPP_WARN(get_node()->get_logger(),
                    "Control value is NaN. Setting it to 0.0");
      }

      command_interfaces_[index].set_value(
          impedance_control_rt_pub_->msg_.data[data_index]);
      position_error_rt_pub_->unlock();
      velocity_error_rt_pub_->unlock();
      target_position_rt_pub_->unlock();
      impedance_control_rt_pub_->unlock();
      index++;
    }
  }

  // ========================Prepare the message for the state publisher
  for (size_t i = 0; i < legs_map_.size(); ++i) {
    auto passive_knee_joints = legs_map_[i].get_passive_knee_joints();
    const size_t first = 2 * i;
    const size_t second = 2 * i + 1;
    state_rt_pub_->msg_.name[first] = passive_knee_joints.first.name;
    state_rt_pub_->msg_.position[first] = passive_knee_joints.first.position;
    state_rt_pub_->msg_.velocity[first] = passive_knee_joints.first.velocity;
    state_rt_pub_->msg_.effort[first] = passive_knee_joints.first.effort;

    state_rt_pub_->msg_.name[second] = passive_knee_joints.second.name;
    state_rt_pub_->msg_.position[second] = passive_knee_joints.second.position;
    state_rt_pub_->msg_.velocity[second] = passive_knee_joints.second.velocity;
    state_rt_pub_->msg_.effort[second] = passive_knee_joints.second.effort;
  }

  if (state_rt_pub_ && state_rt_pub_->trylock()) {
    state_rt_pub_->msg_.header.stamp = time;
    state_rt_pub_->unlockAndPublish();
  }

  if (impedance_control_rt_pub_ && impedance_control_rt_pub_->trylock()) {
    impedance_control_rt_pub_->unlockAndPublish();
  }
  if (target_position_rt_pub_ && target_position_rt_pub_->trylock()) {
    target_position_rt_pub_->unlockAndPublish();
  }
  if (position_error_rt_pub_ && position_error_rt_pub_->trylock()) {
    position_error_rt_pub_->unlockAndPublish();
  }
  if (velocity_error_rt_pub_ && velocity_error_rt_pub_->trylock()) {
    velocity_error_rt_pub_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

} // namespace quadruped_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(quadruped_controller::QuadrupedController,
                       controller_interface::ControllerInterface)
