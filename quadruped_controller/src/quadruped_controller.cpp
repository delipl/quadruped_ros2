// Copyright 2026 Jakub Delicat
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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

namespace
{  // utility

// TODO(destogl): remove this when merged upstream
// Changed services history QoS to keep all so we don't lose any client service
// calls
static constexpr rmw_qos_profile_t rmw_qos_profile_services_hist_keep_all = {
  RMW_QOS_POLICY_HISTORY_KEEP_ALL,
  1,  // message queue depth
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  RMW_QOS_POLICY_DURABILITY_VOLATILE,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false};

using ControllerReferenceMsg = quadruped_controller::QuadrupedController::ControllerReferenceMsg;

// called from RT control loop
void reset_controller_reference_msg(
  const std::shared_ptr<ControllerReferenceMsg> & msg, const std::vector<std::string> & joint_names)
{
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

}  // namespace

namespace quadruped_controller
{
QuadrupedController::QuadrupedController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn QuadrupedController::on_init()
{
  control_mode_.initRT(control_mode_type::FAST);

  try {
    param_listener_ = std::make_shared<quadruped_controller::ParamListener>(get_node());
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn QuadrupedController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();

  if (!params_.state_joints.empty()) {
    state_joints_ = params_.state_joints;
  } else {
    state_joints_ = params_.joints;
  }

  if (params_.joints.size() != state_joints_.size()) {
    RCLCPP_FATAL(
      get_node()->get_logger(),
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
    std::bind(&QuadrupedController::reference_callback, this, std::placeholders::_1));

  std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();
  reset_controller_reference_msg(msg, params_.joints);
  input_ref_.writeFromNonRT(msg);

  auto set_slow_mode_service_callback =
    [&](
      const std::shared_ptr<ControllerModeSrvType::Request> request,
      std::shared_ptr<ControllerModeSrvType::Response> response) {
      if (request->data) {
        control_mode_.writeFromNonRT(control_mode_type::SLOW);
      } else {
        control_mode_.writeFromNonRT(control_mode_type::FAST);
      }
      response->success = true;
    };

  set_slow_control_mode_service_ = get_node()->create_service<ControllerModeSrvType>(
    "~/set_slow_control_mode", set_slow_mode_service_callback,
    rmw_qos_profile_services_hist_keep_all);

  try {
    // State publisher TODO: parametrize this
    state_pub_ = get_node()->create_publisher<ControllerStateMsg>(
      "/joint_states", rclcpp::SystemDefaultsQoS());
    state_rt_pub_ = std::make_unique<ControllerStatePublisher>(state_pub_);

    impedance_control_normal_pub_ = get_node()->create_publisher<DataMsg>(
      "~/impedance_controller/commands", rclcpp::SystemDefaultsQoS());
    impedance_control_rt_pub_ = std::make_unique<DataRTPublisher>(impedance_control_normal_pub_);

    target_joint_position_normal_pub_ = get_node()->create_publisher<DataMsg>(
      "~/target_position", rclcpp::SystemDefaultsQoS());
    target_joint_position_rt_pub_ =
      std::make_unique<DataRTPublisher>(target_joint_position_normal_pub_);

    position_error_normal_pub_ = get_node()->create_publisher<DataMsg>(
      "~/position_error", rclcpp::SystemDefaultsQoS());
    position_error_rt_pub_ = std::make_unique<DataRTPublisher>(position_error_normal_pub_);
    velocity_error_normal_pub_ = get_node()->create_publisher<DataMsg>(
      "~/velocity_error", rclcpp::SystemDefaultsQoS());
    velocity_error_rt_pub_ = std::make_unique<DataRTPublisher>(velocity_error_normal_pub_);

    foot_position_normal_pub_ = get_node()->create_publisher<DataMsg>(
      "~/foot_position", rclcpp::SystemDefaultsQoS());
    foot_position_rt_pub_ = std::make_unique<DataRTPublisher>(foot_position_normal_pub_);

    target_foot_position_normal_pub_ = get_node()->create_publisher<DataMsg>(
      "~/target_foot_position", rclcpp::SystemDefaultsQoS());
    target_foot_position_rt_pub_ =
      std::make_unique<DataRTPublisher>(target_foot_position_normal_pub_);
    foot_position_error_normal_pub_ = get_node()->create_publisher<DataMsg>(
      "~/foot_position_error", rclcpp::SystemDefaultsQoS());
    foot_position_error_rt_pub_ =
      std::make_unique<DataRTPublisher>(foot_position_error_normal_pub_);

    foot_control_position_normal_pub_ = get_node()->create_publisher<DataMsg>(
      "~/foot_control_position", rclcpp::SystemDefaultsQoS());
    foot_control_position_rt_pub_ =
      std::make_unique<DataRTPublisher>(foot_control_position_normal_pub_);

    // Visualization Publisher
    visualization_normal_pub_ = get_node()->create_publisher<VisualizationMsg>(
      "~/visualization_markers", rclcpp::SystemDefaultsQoS());
    visualization_rt_pub_ = std::make_unique<VisualizationRTPublisher>(visualization_normal_pub_);

    visualization_ = std::make_unique<Visualization>(get_node(), "~/visualization_markers");

    multi_dof_state_normal_pub_ = get_node()->create_publisher<MultiDofStateStampedMsg>(
      "~/multi_dof_state", rclcpp::SystemDefaultsQoS());
    multi_dof_state_rt_pub_ =
      std::make_unique<MultiDofStateStampedRTP>(multi_dof_state_normal_pub_);

    multi_dof_task_state_normal_pub_ = get_node()->create_publisher<MultiDofStateStampedMsg>(
      "~/multi_dof_task_state", rclcpp::SystemDefaultsQoS());
    multi_dof_task_state_rt_pub_ =
      std::make_unique<MultiDofStateStampedRTP>(multi_dof_task_state_normal_pub_);

    step_service_ = get_node()->create_service<example_interfaces::srv::AddTwoInts>(
      "~/step", [&](
                  const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
                  std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response) {
        if (request->a == 100) {
          for (int i = 0; i < 4; i++) {
            // Kp[3 * i] = 95.92020756982738;
            // Kp[3 * i + 1] = 25.601396;
            // Kp[3 * i + 2] = 51.207090397090106;
            // Kd[3 * i] = 95.92020756982738 * 0.022526014640095713;
            // Kd[3 * i + 1] = 25.601396 * 0.0240;
            // Kd[3 * i + 2] = 51.207090397090106 * 0.023184721126433695;
            feed_forward_[3 * i] = 0.0;
            feed_forward_[3 * i + 1] = 0.0;
            feed_forward_[3 * i + 2] = 0.0;
          }
        } else {
          feed_forward_[request->a] = 0.001 * request->b;
          Kp[request->a] = 0;
          Kd[request->a] = 0;
        }
      });

    standing_sequence_service_ = get_node()->create_service<std_srvs::srv::Trigger>(
      "~/standing_sequence", [&](
                               const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        if (standing_sequence_) {
          response->success = false;
          response->message = "Standing sequence is already running.";
          return;
        }
        standing_sequence_ = true;
        response->success = true;
        response->message = "Standing sequence started.";
      });
  } catch (const std::exception & e) {
    fprintf(
      stderr,
      "Exception thrown during publisher creation at configure stage "
      "with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  for (int i = 0; i < params_.leg_names.size(); i++) {
    Kp[3 * i] = params_.first_joint_kp;
    Kp[3 * i + 1] = params_.second_joint_kp;
    Kp[3 * i + 2] = params_.third_joint_kp;
    Kd[3 * i] = params_.first_joint_kd;
    Kd[3 * i + 1] = params_.second_joint_kd;
    Kd[3 * i + 2] = params_.third_joint_kd;
  }

  RCLCPP_INFO(get_node()->get_logger(), "Read %d leg names", params_.leg_names.size());

  for (const auto & leg_name : params_.leg_names) {
    quadruped_controller::Leg leg(leg_name);
    legs_map_.push_back(leg);
  }

  for (std::size_t i = 0; i < params_.standing_sequence_positions.size(); ++i) {
    RCLCPP_INFO(
      get_node()->get_logger(), "Standing sequence position %zu: %s", i,
      params_.standing_sequence_positions[i].c_str());
    Eigen::Vector3d vec;

    try {
      parse_string_to_eigen_vector3d(params_.standing_sequence_positions[i], vec);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Error parsing standing sequence position %zu: %s with error: %s",
        i, params_.standing_sequence_positions[i].c_str(), e.what());
      return controller_interface::CallbackReturn::ERROR;
    }
    standing_sequence_positions_.push_back(vec);
  }

  const std::size_t data_size = legs_map_.size() * JOINTS_IN_LEG;

  RCLCPP_INFO(get_node()->get_logger(), "Data size is: %d", data_size);

  joint_positions_ = Eigen::VectorXd::Zero(data_size);
  joint_positions_errors_ = Eigen::VectorXd::Zero(data_size);
  joint_velocities_ = Eigen::VectorXd::Zero(data_size);
  joint_velocity_errors_ = Eigen::VectorXd::Zero(data_size);
  joint_efforts_ = Eigen::VectorXd::Zero(data_size);

  target_joint_positions_ = Eigen::VectorXd::Zero(data_size);
  target_joint_efforts_ = Eigen::VectorXd::Zero(data_size);

  foot_positions_ = Eigen::VectorXd::Zero(data_size);
  foot_control_positions_ = Eigen::VectorXd::Zero(data_size);
  target_foot_positions_ = Eigen::VectorXd::Zero(data_size);
  foot_positions_error_ = Eigen::VectorXd::Zero(data_size);

  // TODO(anyone): Reserve memory in state publisher depending on the message
  // type
  state_rt_pub_->lock();
  state_rt_pub_->msg_.header.frame_id = "base_link";
  state_rt_pub_->msg_.name.resize(legs_map_.size() * 2);
  state_rt_pub_->msg_.position.resize(legs_map_.size() * 2);
  state_rt_pub_->msg_.velocity.resize(legs_map_.size() * 2);
  state_rt_pub_->msg_.effort.resize(legs_map_.size() * 2);
  state_rt_pub_->unlock();

  impedance_control_rt_pub_->lock();
  impedance_control_rt_pub_->msg_.data.resize(legs_map_.size() * 3);
  impedance_control_rt_pub_->unlock();
  target_joint_position_rt_pub_->lock();
  target_joint_position_rt_pub_->msg_.data.resize(legs_map_.size() * 3);
  target_joint_position_rt_pub_->unlock();
  position_error_rt_pub_->lock();
  position_error_rt_pub_->msg_.data.resize(legs_map_.size() * 3);
  position_error_rt_pub_->unlock();
  velocity_error_rt_pub_->lock();
  velocity_error_rt_pub_->msg_.data.resize(legs_map_.size() * 3);
  velocity_error_rt_pub_->unlock();

  foot_position_rt_pub_->lock();
  foot_position_rt_pub_->msg_.data.resize(legs_map_.size() * 3);
  foot_position_rt_pub_->unlock();
  target_foot_position_rt_pub_->lock();
  target_foot_position_rt_pub_->msg_.data.resize(legs_map_.size() * 3);
  target_foot_position_rt_pub_->unlock();
  foot_position_error_rt_pub_->lock();
  foot_position_error_rt_pub_->msg_.data.resize(legs_map_.size() * 3);
  foot_position_error_rt_pub_->unlock();

  foot_control_position_rt_pub_->lock();
  foot_control_position_rt_pub_->msg_.data.resize(legs_map_.size() * 3);
  foot_control_position_rt_pub_->unlock();

  multi_dof_state_rt_pub_->lock();
  multi_dof_state_rt_pub_->msg_.dof_states.resize(legs_map_.size() * 3);
  multi_dof_state_rt_pub_->msg_.header.frame_id = "base_link";
  multi_dof_state_rt_pub_->msg_.header.stamp = get_node()->now();
  multi_dof_state_rt_pub_->unlock();

  multi_dof_task_state_rt_pub_->lock();
  multi_dof_task_state_rt_pub_->msg_.dof_states.resize(legs_map_.size() * 3);
  multi_dof_state_rt_pub_->msg_.header.frame_id = "base_link";
  multi_dof_state_rt_pub_->msg_.header.stamp = get_node()->now();
  multi_dof_task_state_rt_pub_->unlock();

  // TF Publisher
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(get_node());

  auto publish_tf_callback = [&]() {
    for (std::size_t i = 0; i < legs_map_.size(); ++i) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = get_node()->now();
      transform.header.frame_id = "base_link";
      transform.child_frame_id = legs_map_[i].get_name() + "_foot_calculated_link";

      const auto foot_position = foot_positions_.segment<3>(i * 3);
      transform.transform.translation.x = foot_position.x();
      transform.transform.translation.y = foot_position.y();
      transform.transform.translation.z = foot_position.z();
      transform.transform.rotation.x = 0.0;
      transform.transform.rotation.y = 0.0;
      transform.transform.rotation.z = 0.0;
      transform.transform.rotation.w = 1.0;

      tf_broadcaster_->sendTransform(transform);
    }
  };

  tf_timer_ = get_node()->create_wall_timer(std::chrono::milliseconds(10), publish_tf_callback);

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration QuadrupedController::command_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_config.names.reserve(params_.joints.size());
  for (const auto & joint : params_.joints) {
    command_interfaces_config.names.push_back(joint + "/" + params_.command_interface);
  }

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration QuadrupedController::state_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interfaces_config.names.reserve(state_joints_.size());
  for (const auto & joint : state_joints_) {
    for (const auto & interface_name : params_.state_interfaces) {
      state_interfaces_config.names.push_back(joint + "/" + interface_name);
    }
  }

  return state_interfaces_config;
}

void QuadrupedController::reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg)
{
  input_ref_.writeFromNonRT(msg);
}
controller_interface::CallbackReturn QuadrupedController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Set default value in command
  reset_controller_reference_msg(*(input_ref_.readFromRT()), state_joints_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn QuadrupedController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): depending on number of interfaces, use definitions, e.g.,
  // `CMD_MY_ITFS`, instead of a loop
  // Maybe sitdonw the robot

  for (auto & command_interface : command_interfaces_) {
    if (!command_interface.set_value(0.0)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Unable to set command interface value to 0.0");
      return controller_interface::CallbackReturn::SUCCESS;
    }
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type QuadrupedController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // TODO(anyone): depending on number of interfaces, use definitions, e.g.,
  // `CMD_MY_ITFS`, instead of a loop

  auto start = std::chrono::high_resolution_clock::now();

  calculate_kinematics();

  if (stood_first_time_ and not standing_sequence_) {
    auto msg = *(input_ref_.readFromRT());
    set_target_states(msg);
    calculate_inverse_kinematics();
  } else if (standing_sequence_) {
    standing_sequence_control();
  } else {
    for (std::size_t i = 0; i < legs_map_.size(); ++i) {
      RCLCPP_INFO_STREAM_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000, "Waiting for standing");
      auto & leg = legs_map_[i];
      Eigen::Vector3d target_joint_pos = standing_sequence_positions_[0];
      auto directions = leg.get_joints_directions();

      target_joint_pos = directions.cwiseProduct(target_joint_pos);
      target_joint_positions_.segment<3>(i * 3) << target_joint_pos;
    }
  }

  calculate_control();

  update_passive_joints();

  if (visualization_rt_pub_ && visualization_rt_pub_->trylock()) {
    visualization_rt_pub_->unlockAndPublish();
  }

  if (state_rt_pub_ && state_rt_pub_->trylock()) {
    state_rt_pub_->msg_.header.stamp = time;
    state_rt_pub_->unlockAndPublish();
  }

  set_msg_data_from_vector_and_publish(impedance_control_rt_pub_, target_joint_efforts_);
  set_msg_data_from_vector_and_publish(target_joint_position_rt_pub_, target_joint_positions_);
  set_msg_data_from_vector_and_publish(position_error_rt_pub_, joint_positions_errors_);
  set_msg_data_from_vector_and_publish(velocity_error_rt_pub_, joint_velocity_errors_);

  set_msg_data_from_vector_and_publish(foot_position_rt_pub_, foot_positions_);
  set_msg_data_from_vector_and_publish(target_foot_position_rt_pub_, target_foot_positions_);
  set_msg_data_from_vector_and_publish(foot_position_error_rt_pub_, foot_positions_error_);

  set_msg_data_from_vector_and_publish(foot_control_position_rt_pub_, foot_control_positions_);

  auto end = std::chrono::high_resolution_clock::now();
  auto elapsed_time = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
  RCLCPP_DEBUG(
    get_node()->get_logger(), "Update loop elapsed time: %ld microseconds", elapsed_time);

  return controller_interface::return_type::OK;
}

void QuadrupedController::calculate_kinematics()
{
  // Setting states and forward kinematics
  for (std::size_t i = 0; i < legs_map_.size(); ++i) {
    auto & leg = legs_map_[i];

    Eigen::Vector3d q;
    Eigen::Vector3d v;
    Eigen::Vector3d tau;

    // Throught joints. Remember there are positions, velocities and efforts
    for (std::size_t j = 0; j < JOINTS_IN_LEG; ++j) {
      const auto q_index = get_state_interface_index(i, j, 0);
      const auto v_index = get_state_interface_index(i, j, 1);
      const auto tau_index = get_state_interface_index(i, j, 2);
      q[j] = state_interfaces_[q_index].get_value();
      v[j] = state_interfaces_[v_index].get_value();
      tau[j] = state_interfaces_[tau_index].get_value();
      const auto idx = get_command_interface_index(i, j);

      joint_positions_[idx] = state_interfaces_[q_index].get_value();
      joint_velocities_[idx] = state_interfaces_[v_index].get_value();
      joint_efforts_[idx] = state_interfaces_[tau_index].get_value();
    }

    leg.set_joints_states(q, v, tau);
    const auto foot_position = leg.forward_kinematics();

    foot_positions_.segment<3>(i * 3) = foot_position;
  }
}

void QuadrupedController::set_target_states(
  const quadruped_msgs::msg::QuadrupedControl::SharedPtr & msg)
{
  for (size_t i = 0; i < legs_map_.size(); ++i) {
    if (legs_map_[i].get_name() == "front_left") {
      target_foot_positions_.segment<3>(i * 3) << msg->fl_foot_position.x, msg->fl_foot_position.y,
        msg->fl_foot_position.z;
    } else if (legs_map_[i].get_name() == "front_right") {
      target_foot_positions_.segment<3>(i * 3) << msg->fr_foot_position.x, msg->fr_foot_position.y,
        msg->fr_foot_position.z;
    } else if (legs_map_[i].get_name() == "rear_left") {
      target_foot_positions_.segment<3>(i * 3) << msg->rl_foot_position.x, msg->rl_foot_position.y,
        msg->rl_foot_position.z;
    } else if (legs_map_[i].get_name() == "rear_right") {
      target_foot_positions_.segment<3>(i * 3) << msg->rr_foot_position.x, msg->rr_foot_position.y,
        msg->rr_foot_position.z;
    }
  }
}

void QuadrupedController::calculate_inverse_kinematics()
{
  for (std::size_t i = 0; i < legs_map_.size(); ++i) {
    auto & leg = legs_map_[i];

    foot_positions_error_.segment<3>(i * 3) = target_foot_positions_.segment<3>(i * 3) -
                                              foot_positions_.segment<3>(i * 3);

    foot_control_positions_.segment<3>(i * 3) = target_foot_positions_.segment<3>(i * 3);

    if (std::isnan(foot_control_positions_[0])) {
      RCLCPP_WARN_STREAM_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "No valid reference received yet,holding this position.");
      // target_joint_positions_.segment<3>(i * 3) << joint_positions_.segment<3>(i * 3);
      continue;
    }

    auto q = leg.inverse_kinematics(foot_control_positions_.segment<3>(i * 3));
    for (std::size_t j = 0; j < JOINTS_IN_LEG; ++j) {
      if (std::isnan(q[j])) {
        target_joint_positions_[i * 3 + j] = joint_positions_[i * 3 + j];
        RCLCPP_WARN(
          get_node()->get_logger(), "Inverse kinematics failed for leg %zu joint %zu", i, j);
      } else {
        target_joint_positions_[i * 3 + j] = q[j];
      }
    }
  }
}

void QuadrupedController::calculate_control()
{
  joint_positions_errors_ = target_joint_positions_ - joint_positions_;
  joint_velocity_errors_ = -joint_velocities_;

  // RCLCPP_INFO_STREAM(get_node()->get_logger(), feed_forward);

  target_joint_efforts_ = Kp.cwiseProduct(joint_positions_errors_) +
                          Kd.cwiseProduct(joint_velocity_errors_) + feed_forward_;

  for (size_t i = 0; i < legs_map_.size() * JOINTS_IN_LEG; ++i) {
    command_interfaces_[i].set_value(target_joint_efforts_[i]);
  }
}

void QuadrupedController::standing_sequence_control()
{
  std::vector<bool> leg_reached_position(legs_map_.size(), false);

  for (std::size_t i = 0; i < legs_map_.size(); ++i) {
    RCLCPP_INFO_STREAM_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000, "Standing sequence active.");
    auto & leg = legs_map_[i];

    Eigen::Vector3d target_joint_pos = standing_sequence_positions_[standing_sequence_step_];
    auto directions = leg.get_joints_directions();

    target_joint_pos = directions.cwiseProduct(target_joint_pos);
    target_joint_positions_.segment<3>(i * 3) << target_joint_pos;
    const auto joints_position = joint_positions_.segment<3>(i * 3);

    const double error = (joints_position - target_joint_pos).norm();
    if (error < params_.standing_sequence_positions_accuracy) {
      RCLCPP_INFO_STREAM(
        get_node()->get_logger(),
        "Leg " << leg.get_name() << " reached standing position step " << standing_sequence_step_);
      leg_reached_position[i] = true;
    }
  }

  if (std::all_of(
        leg_reached_position.begin(), leg_reached_position.end(), [](bool v) { return v; })) {
    standing_sequence_step_++;
    if (standing_sequence_step_ >= standing_sequence_positions_.size()) {
      standing_sequence_ = false;
      standing_sequence_step_ = 0;
      stood_first_time_ = !stood_first_time_;
      std::reverse(standing_sequence_positions_.begin(), standing_sequence_positions_.end());

      RCLCPP_INFO(
        get_node()->get_logger(), "Standing sequence finished, switching to reference input.");
    }
  }
}

void QuadrupedController::update_passive_joints()
{
  // ========================Prepare the message for the state publisher
  visualization_msgs::msg::MarkerArray vis_msg;
  std_msgs::msg::ColorRGBA red;
  red.r = 1.0;
  red.g = 0.0;
  red.b = 0.0;
  red.a = 1.0;
  std_msgs::msg::ColorRGBA green;
  green.r = 0.0;
  green.g = 1.0;
  green.b = 0.0;
  green.a = 1.0;

  for (size_t i = 0; i < legs_map_.size(); ++i) {
    auto joint_states = legs_map_[i].get_joints_states();

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

    vis_msg.markers.push_back(
      visualization_->createSphere(foot_positions_.segment<3>(i * 3), 0.02, red, "base_link", i));
    vis_msg.markers.push_back(visualization_->createSphere(
      target_foot_positions_.segment<3>(i * 3), 0.02, green, "base_link", 10 + i));
  }
  visualization_rt_pub_->msg_ = vis_msg;
}

std::size_t QuadrupedController::get_state_interface_index(
  std::size_t leg_index, std::size_t joint_index, std::size_t interface_index) const
{
  return leg_index * 9 + joint_index * 3 + interface_index;
}

std::size_t QuadrupedController::get_command_interface_index(
  std::size_t leg_index, std::size_t joint_index) const
{
  return leg_index * 3 + joint_index;
}

void QuadrupedController::set_msg_data_from_vector_and_publish(
  std::unique_ptr<DataRTPublisher> & rt_pub, const Eigen::VectorXd & data)
{
  if (rt_pub && rt_pub->trylock()) {
    for (size_t i = 0; i < data.size(); ++i) {
      rt_pub->msg_.data[i] = data[i];
    }

    rt_pub->unlockAndPublish();
  }
}
void QuadrupedController::parse_string_to_eigen_vector3d(
  const std::string & str, Eigen::Vector3d & vec)
{
  std::string s = str;
  s.erase(std::remove(s.begin(), s.end(), '['), s.end());
  s.erase(std::remove(s.begin(), s.end(), ']'), s.end());
  std::replace(s.begin(), s.end(), ',', ' ');
  std::istringstream iss(s);
  iss >> vec[0] >> vec[1] >> vec[2];
}

}  // namespace quadruped_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  quadruped_controller::QuadrupedController, controller_interface::ControllerInterface)
