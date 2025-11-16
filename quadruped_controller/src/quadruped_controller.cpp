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

namespace
{  // utility

// TODO(destogl): remove this when merged upstream
// Changed services history QoS to keep all so we don't lose any client service
// calls
static constexpr rmw_qos_profile_t rmw_qos_profile_services_hist_keep_all = { RMW_QOS_POLICY_HISTORY_KEEP_ALL,
                                                                              1,  // message queue depth
                                                                              RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                                                                              RMW_QOS_POLICY_DURABILITY_VOLATILE,
                                                                              RMW_QOS_DEADLINE_DEFAULT,
                                                                              RMW_QOS_LIFESPAN_DEFAULT,
                                                                              RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
                                                                              RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
                                                                              false };

using ControllerReferenceMsg = quadruped_controller::QuadrupedController::ControllerReferenceMsg;

// called from RT control loop
void reset_controller_reference_msg(const std::shared_ptr<ControllerReferenceMsg>& msg,
                                    const std::vector<std::string>& joint_names)
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
QuadrupedController::QuadrupedController() : controller_interface::ControllerInterface()
{
}

controller_interface::CallbackReturn QuadrupedController::on_init()
{
  control_mode_.initRT(control_mode_type::FAST);

  try
  {
    param_listener_ = std::make_shared<quadruped_controller::ParamListener>(get_node());
  }
  catch (const std::exception& e)
  {
    fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
QuadrupedController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  params_ = param_listener_->get_params();

  if (!params_.state_joints.empty())
  {
    state_joints_ = params_.state_joints;
  }
  else
  {
    state_joints_ = params_.joints;
  }

  if (params_.joints.size() != state_joints_.size())
  {
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
      "~/reference", subscribers_qos, std::bind(&QuadrupedController::reference_callback, this, std::placeholders::_1));

  std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();
  reset_controller_reference_msg(msg, params_.joints);
  input_ref_.writeFromNonRT(msg);

  auto set_slow_mode_service_callback = [&](const std::shared_ptr<ControllerModeSrvType::Request> request,
                                            std::shared_ptr<ControllerModeSrvType::Response> response) {
    if (request->data)
    {
      control_mode_.writeFromNonRT(control_mode_type::SLOW);
    }
    else
    {
      control_mode_.writeFromNonRT(control_mode_type::FAST);
    }
    response->success = true;
  };

  set_slow_control_mode_service_ = get_node()->create_service<ControllerModeSrvType>(
      "~/set_slow_control_mode", set_slow_mode_service_callback, rmw_qos_profile_services_hist_keep_all);

  try
  {
    // State publisher TODO: parametrize this
    state_pub_ = get_node()->create_publisher<ControllerStateMsg>("/joint_states", rclcpp::SystemDefaultsQoS());
    state_rt_pub_ = std::make_unique<ControllerStatePublisher>(state_pub_);

    impedance_control_normal_pub_ =
        get_node()->create_publisher<DataMsg>("~/impedance_controller/commands", rclcpp::SystemDefaultsQoS());
    impedance_control_rt_pub_ = std::make_unique<DataRTPublisher>(impedance_control_normal_pub_);

    target_joint_position_normal_pub_ =
        get_node()->create_publisher<DataMsg>("~/target_position", rclcpp::SystemDefaultsQoS());
    target_joint_position_rt_pub_ = std::make_unique<DataRTPublisher>(target_joint_position_normal_pub_);

    position_error_normal_pub_ = get_node()->create_publisher<DataMsg>("~/position_error", rclcpp::SystemDefaultsQoS());
    position_error_rt_pub_ = std::make_unique<DataRTPublisher>(position_error_normal_pub_);
    velocity_error_normal_pub_ = get_node()->create_publisher<DataMsg>("~/velocity_error", rclcpp::SystemDefaultsQoS());
    velocity_error_rt_pub_ = std::make_unique<DataRTPublisher>(velocity_error_normal_pub_);

    foot_position_normal_pub_ = get_node()->create_publisher<DataMsg>("~/foot_position", rclcpp::SystemDefaultsQoS());
    foot_position_rt_pub_ = std::make_unique<DataRTPublisher>(foot_position_normal_pub_);

    target_foot_position_normal_pub_ =
        get_node()->create_publisher<DataMsg>("~/target_foot_position", rclcpp::SystemDefaultsQoS());
    target_foot_position_rt_pub_ = std::make_unique<DataRTPublisher>(target_foot_position_normal_pub_);
    foot_position_error_normal_pub_ =
        get_node()->create_publisher<DataMsg>("~/foot_position_error", rclcpp::SystemDefaultsQoS());
    foot_position_error_rt_pub_ = std::make_unique<DataRTPublisher>(foot_position_error_normal_pub_);

    foot_control_position_normal_pub_ =
        get_node()->create_publisher<DataMsg>("~/foot_control_position", rclcpp::SystemDefaultsQoS());
    foot_control_position_rt_pub_ = std::make_unique<DataRTPublisher>(foot_control_position_normal_pub_);

    // Visualization Publisher
    visualization_normal_pub_ =
        get_node()->create_publisher<VisualizationMsg>("~/visualization_markers", rclcpp::SystemDefaultsQoS());
    visualization_rt_pub_ = std::make_unique<VisualizationRTPublisher>(visualization_normal_pub_);

    visualization_ = std::make_unique<Visualization>(get_node(), "~/visualization_markers");

    multi_dof_state_normal_pub_ =
        get_node()->create_publisher<MultiDofStateStampedMsg>("~/multi_dof_state", rclcpp::SystemDefaultsQoS());
    multi_dof_state_rt_pub_ = std::make_unique<MultiDofStateStampedRTP>(multi_dof_state_normal_pub_);

    multi_dof_task_state_normal_pub_ =
        get_node()->create_publisher<MultiDofStateStampedMsg>("~/multi_dof_task_state", rclcpp::SystemDefaultsQoS());
    multi_dof_task_state_rt_pub_ = std::make_unique<MultiDofStateStampedRTP>(multi_dof_task_state_normal_pub_);

    step_service_ = get_node()->create_service<example_interfaces::srv::AddTwoInts>(
        "~/step", [&](const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
                      std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response) {
          if (request->a == 100)
          {
            for (int i = 0; i < 4; i++)
            {
              Kp[3 * i] = 95.92020756982738;
              Kp[3 * i + 1] = 25.601396;
              Kp[3 * i + 2] = 51.207090397090106;
              Kd[3 * i] = 95.92020756982738 * 0.022526014640095713;
              Kd[3 * i + 1] = 25.601396 * 0.0240;
              Kd[3 * i + 2] = 51.207090397090106 * 0.023184721126433695;
              feed_forward_[3 * i] = 0.0;
              feed_forward_[3 * i + 1] = 0.0;
              feed_forward_[3 * i + 2] = 0.0;
            }
          }
          else
          {
            feed_forward_[request->a] = 0.001 * request->b;
            Kp[request->a] = 0;
            Kd[request->a] = 0;
          }
        });
  }
  catch (const std::exception& e)
  {
    fprintf(stderr,
            "Exception thrown during publisher creation at configure stage "
            "with message : %s \n",
            e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

    // Kp << 5.85966683083936, 0, 0,
    Kp << 22.5633300297324, 22.5633300297324, 22.5633300297324,

    0,0,0,
    0,0,0,
    0,0,0;
    // Kd << 5.85966683083936 * 0.0264537851168775,
    Kd << 22.5633300297324 * 0.0102549019607843,
     22.5633300297324 * 0.0102549019607843,
     22.5633300297324 * 0.0102549019607843,
    0,0,0,
    0,0,0,
    0,0,0;
  // Kp << 95.92020756982738, 51.207090397090106, 51.207090397090106,
  //     95.92020756982738, 51.207090397090106, 51.207090397090106,
  //     95.92020756982738, 51.207090397090106, 51.207090397090106,
  //     95.92020756982738, 51.207090397090106, 51.207090397090106;
  // Kd << 95.92020756982738 * 0.022526014640095713,
  //     51.207090397090106 * 0.023184721126433695,
  //     51.207090397090106 * 0.023184721126433695,
  //     95.92020756982738 * 0.022526014640095713,
  //     51.207090397090106 * 0.023184721126433695,
  //     51.207090397090106 * 0.023184721126433695,
  //     95.92020756982738 * 0.022526014640095713,
  //     51.207090397090106 * 0.023184721126433695,
  //     51.207090397090106 * 0.023184721126433695,
  //     95.92020756982738 * 0.022526014640095713,
  //     51.207090397090106 * 0.023184721126433695,
  //     51.207090397090106 * 0.023184721126433695;


  RCLCPP_INFO(get_node()->get_logger(), "Read %d leg names", params_.leg_names.size());

  for (const auto& leg_name : params_.leg_names)
  {
    quadruped_controller::Leg leg(leg_name);
    legs_map_.push_back(leg);
  }

  const std::size_t data_size = legs_map_.size() * 3;

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
    for (std::size_t i = 0; i < legs_map_.size(); ++i)
    {
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = get_node()->now();
      transform.header.frame_id = "base_link";
      transform.child_frame_id = legs_map_[i].get_name() + "_foot_link";

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

  tf_timer_ = get_node()->create_wall_timer(std::chrono::milliseconds(10),
                                            publish_tf_callback);

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration QuadrupedController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_config.names.reserve(params_.joints.size());
  for (const auto& joint : params_.joints)
  {
    command_interfaces_config.names.push_back(joint + "/" + params_.command_interface);
  }

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration QuadrupedController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interfaces_config.names.reserve(state_joints_.size());
  for (const auto& joint : state_joints_)
  {
    for (const auto& interface_name : params_.state_interfaces)
    {
      state_interfaces_config.names.push_back(joint + "/" + interface_name);
    }
  }

  return state_interfaces_config;
}

void QuadrupedController::reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg)
{
  input_ref_.writeFromNonRT(msg);
}
controller_interface::CallbackReturn QuadrupedController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  // Set default value in command
  reset_controller_reference_msg(*(input_ref_.readFromRT()), state_joints_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
QuadrupedController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  // TODO(anyone): depending on number of interfaces, use definitions, e.g.,
  // `CMD_MY_ITFS`, instead of a loop
  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    command_interfaces_[i].set_value(std::numeric_limits<double>::quiet_NaN());
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type QuadrupedController::update(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  // TODO(anyone): depending on number of interfaces, use definitions, e.g.,
  // `CMD_MY_ITFS`, instead of a loop

  auto start = std::chrono::high_resolution_clock::now();

  // Setting states and forward kinematics
  for (std::size_t i = 0; i < legs_map_.size(); ++i)
  {
    auto& leg = legs_map_[i];

    Eigen::Vector3d q;
    Eigen::Vector3d v;
    Eigen::Vector3d tau;

    // Throught joints. Remember there are positions, velocities and efforts
    for (std::size_t j = 0; j < 3; ++j)
    {
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

  // Read target states
  auto msg = *(input_ref_.readFromRT());

  target_foot_positions_ << msg->fl_foot_position.x, msg->fl_foot_position.y, msg->fl_foot_position.z,
      msg->fr_foot_position.x, msg->fr_foot_position.y, msg->fr_foot_position.z, msg->rl_foot_position.x,
      msg->rl_foot_position.y, msg->rl_foot_position.z, msg->rr_foot_position.x, msg->rr_foot_position.y,
      msg->rr_foot_position.z;

  // Inverse kinematics
  for (std::size_t i = 0; i < legs_map_.size(); ++i)
  {
    // ADDED!!!!
    // double s = std::sin(time.seconds()*5);
    // double target = (s+1)/2.0 * 0.2 - 0.3; 
    // double target_2 = (s+1)/2.0 * 1.57 ; 

    // RCLCPP_INFO(get_node()->get_logger(), "time: %f sin: %f, target: %f", time.seconds(), s, target);

    // target_joint_positions_.segment<3>(i * 3) << 0.0, target_2, 1.4;
    
    auto& leg = legs_map_[i];

    if (msg->header.stamp == rclcpp::Time(0, 0, RCL_ROS_TIME))
    {
      target_joint_positions_.segment<3>(i * 3) << -0.86, 0.52, 1.8;
      continue;
    }

    foot_positions_error_.segment<3>(i * 3) =
        target_foot_positions_.segment<3>(i * 3) - foot_positions_.segment<3>(i * 3);

    foot_control_positions_.segment<3>(i * 3) = target_foot_positions_.segment<3>(i * 3);

    // z coordinate  PID
    foot_control_positions_[i * 3 + 2] += params_.pid_z_leg_trajectory * foot_positions_error_[i * 3 + 2];

    foot_control_positions_[i * 3 + 2] = std::max(foot_control_positions_[i * 3 + 2], -0.22);

    auto q = leg.inverse_kinematics(foot_control_positions_.segment<3>(i * 3));
    for (std::size_t j = 0; j < 3; ++j)
    {
      if (std::isnan(q[j]))
      {
        target_joint_positions_[i * 3 + j] = joint_positions_[i * 3 + j];
        RCLCPP_WARN(get_node()->get_logger(), "Inverse kinematics failed for leg %zu joint %zu", i, j);
      }
      else
      {
        target_joint_positions_[i * 3 + j] = q[j];
      }
    }
  }

  // TODO: parametrize this

  joint_positions_errors_ = target_joint_positions_ - joint_positions_;
  joint_velocity_errors_ = -joint_velocities_;

  // RCLCPP_INFO_STREAM(get_node()->get_logger(), feed_forward);

  target_joint_efforts_ =
      Kp.cwiseProduct(joint_positions_errors_) + Kd.cwiseProduct(joint_velocity_errors_) + feed_forward_;

  for (size_t i = 0; i < legs_map_.size() * 3; ++i)
  {
    command_interfaces_[i].set_value(target_joint_efforts_[i]);
  }

  // ========================Prepare the message for the state publisher
  visualization_msgs::msg::MarkerArray vis_msg;

  for (size_t i = 0; i < legs_map_.size(); ++i)
  {
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

    // auto J = legs_map_[i].jacobian_2d();
    // joint_states = legs_map_[i].get_joints_states();

    // auto J_t = J.transpose();
    // auto J_t_pinv = J.completeOrthogonalDecomposition().pseudoInverse();
    // Eigen::Vector2d ddq;
    // // ddq << joint_states[0].effort, joint_states[1].effort,
    // //     joint_states[4].effort;
    // ddq << joint_states[1].effort, joint_states[3].effort;

    // Eigen::Vector2d f = J_t_pinv * ddq;

    geometry_msgs::msg::Point start;
    geometry_msgs::msg::Point target_p;
    auto foot_position = foot_positions_.segment<3>(i * 3);
    Eigen::Vector2d f;
    f << legs_map_[i].bar_acc_(0), legs_map_[i].bar_acc_(1);

    start.x = foot_position.x();
    start.y = foot_position.y();
    start.z = foot_position.z();

    target_p.x = foot_position.x();
    target_p.y = foot_position.y();
    target_p.z = foot_position.z();

    target_p.x += f(0) / 5;
    target_p.z += f(1) / 5;
    const std::string frame_id = "base_link";

    std_msgs::msg::ColorRGBA red;
    red.r = 1.0;
    red.a = 1.0;
    std_msgs::msg::ColorRGBA green;
    green.g = 1.0;
    green.a = 1.0;
    std_msgs::msg::ColorRGBA blue;
    blue.b = 1.0;
    blue.a = 1.0;

    vis_msg.markers.push_back(visualization_->createArrow(start, target_p, 0.05, red, frame_id, i));

    geometry_msgs::msg::Point target_q2;
    Eigen::Vector2d f2 = legs_map_[i].bar_q2_acc_;
    target_q2.x = foot_position.x() + f2(0);
    target_q2.y = foot_position.y();
    target_q2.z = foot_position.z() + f2(1);

    vis_msg.markers.push_back(visualization_->createArrow(start, target_q2, 0.05, blue, frame_id, i + 4));

    Eigen::Vector2d f_bar;
    f_bar = legs_map_[i].bar_acc_ + legs_map_[i].bar_q2_acc_;
    geometry_msgs::msg::Point target_bar;
    target_bar.x = foot_position.x() + f_bar(0);
    target_bar.y = foot_position.y();
    target_bar.z = foot_position.z() + f_bar(1);
    vis_msg.markers.push_back(visualization_->createArrow(start, target_bar, 0.05, green, frame_id, i + 8));
  }

  if (visualization_rt_pub_ && visualization_rt_pub_->trylock())
  {
    // visualization_msgs::msg::Marker vis_clean_marker;
    // vis_clean_marker.header.stamp = get_node()->now();
    // vis_clean_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    // visualization_msgs::msg::MarkerArray vis_clean_msg;
    // vis_clean_msg.markers.push_back(vis_clean_marker);

    // visualization_rt_pub_->msg_ = vis_clean_msg;
    // visualization_rt_pub_->unlockAndPublish();

    visualization_rt_pub_->msg_ = vis_msg;

    visualization_rt_pub_->unlockAndPublish();
  }

  if (state_rt_pub_ && state_rt_pub_->trylock())
  {
    state_rt_pub_->msg_.header.stamp = time;
    state_rt_pub_->unlockAndPublish();
  }

  if (multi_dof_state_rt_pub_ && multi_dof_state_rt_pub_->trylock())
  {
    multi_dof_state_rt_pub_->msg_.header.stamp = time;
    for (size_t i = 0; i < legs_map_.size() * 3; ++i)
    {
      auto& dof = multi_dof_state_rt_pub_->msg_.dof_states[i];
      // dof.name =
      dof.error = joint_positions_errors_[i];
      dof.error_dot = joint_velocity_errors_[i];

      dof.feedback = joint_positions_[i];
      dof.feedback_dot = joint_velocities_[i];
      dof.reference = target_joint_positions_[i];

      dof.output = target_joint_efforts_[i];
    }

    multi_dof_state_rt_pub_->unlockAndPublish();
  }

  if (multi_dof_task_state_rt_pub_ && multi_dof_task_state_rt_pub_->trylock())
  {
    multi_dof_task_state_rt_pub_->msg_.header.stamp = time;
    for (size_t i = 0; i < legs_map_.size() * 3; ++i)
    {
      auto& dof = multi_dof_task_state_rt_pub_->msg_.dof_states[i];
      // dof.name =
      dof.error = foot_positions_error_[i];

      dof.feedback = foot_positions_[i];
      dof.reference = target_foot_positions_[i];
      dof.output = foot_control_positions_[i];
    }

    multi_dof_task_state_rt_pub_->unlockAndPublish();
  }

  set_msg_data_from_vector_and_publish(impedance_control_rt_pub_,
                                       target_joint_efforts_);
  set_msg_data_from_vector_and_publish(target_joint_position_rt_pub_,
                                       target_joint_positions_);
  set_msg_data_from_vector_and_publish(position_error_rt_pub_,
                                       joint_positions_errors_);
  set_msg_data_from_vector_and_publish(velocity_error_rt_pub_,
                                       joint_velocity_errors_);

  set_msg_data_from_vector_and_publish(foot_position_rt_pub_,
  foot_positions_);
  set_msg_data_from_vector_and_publish(target_foot_position_rt_pub_,
                                       target_foot_positions_);
  set_msg_data_from_vector_and_publish(foot_position_error_rt_pub_,
                                       foot_positions_error_);

  set_msg_data_from_vector_and_publish(foot_control_position_rt_pub_,
                                       foot_control_positions_);

  auto end = std::chrono::high_resolution_clock::now();
  auto elapsed_time = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
  RCLCPP_DEBUG(get_node()->get_logger(), "Update loop elapsed time: %ld microseconds", elapsed_time);

  return controller_interface::return_type::OK;
}

std::size_t QuadrupedController::get_state_interface_index(std::size_t leg_index, std::size_t joint_index,
                                                           std::size_t interface_index) const
{
  return leg_index * 9 + joint_index * 3 + interface_index;
}

std::size_t QuadrupedController::get_command_interface_index(std::size_t leg_index, std::size_t joint_index) const
{
  return leg_index * 3 + joint_index;
}

void QuadrupedController::set_msg_data_from_vector_and_publish(std::unique_ptr<DataRTPublisher>& rt_pub,
                                                               const Eigen::VectorXd& data)
{
  if (rt_pub && rt_pub->trylock())
  {
    for (size_t i = 0; i < data.size(); ++i)
    {
      rt_pub->msg_.data[i] = data[i];
    }

    rt_pub->unlockAndPublish();
  }
}

}  // namespace quadruped_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(quadruped_controller::QuadrupedController, controller_interface::ControllerInterface)
