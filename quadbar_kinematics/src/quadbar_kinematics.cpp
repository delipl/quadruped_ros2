#include "quadbar_kinematics/quadbar_kinematics.hpp"
#include <iostream>
#include <stdexcept>

namespace quadbar_kinematics {

bool QuadbarKinematics::initialize(
    std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface>
        parameters_interface,
    const std::string &end_effector_name) {
  // Initialize kinematic parameters from the robot_description or
  // parameters_interface Here you would parse the URDF or set up specific
  // parameters

  legs_map_ = {{"front_left", Leg("front_left")},
               {"front_right", Leg("front_right")},
               {"rear_left", Leg("rear_left")},
               {"rear_right", Leg("rear_right")}};
  return true;
}

bool QuadbarKinematics::convert_cartesian_deltas_to_joint_deltas(
    const Eigen::VectorXd &joint_pos,
    const Eigen::Matrix<double, 6, 1> &delta_x, const std::string &link_name,
    Eigen::VectorXd &delta_theta) {
  // Example: Compute joint deltas using Jacobian inverse (stub implementation)
  if (!validate_joint_positions(joint_pos)) {
    throw std::invalid_argument("Invalid joint positions");
  }

  delta_theta = Eigen::VectorXd::Zero(joint_pos.size()); // Stub implementation
  std::cout << "Computed joint deltas for link: " << link_name << std::endl;
  return true;
}

bool QuadbarKinematics::convert_joint_deltas_to_cartesian_deltas(
    const Eigen::VectorXd &joint_pos, const Eigen::VectorXd &delta_theta,
    const std::string &link_name, Eigen::Matrix<double, 6, 1> &delta_x) {
  // Example: Compute Cartesian deltas using the Jacobian (stub implementation)
  if (!validate_joint_positions(joint_pos)) {
    throw std::invalid_argument("Invalid joint positions");
  }

  delta_x = Eigen::Matrix<double, 6, 1>::Zero(); // Stub implementation

  std::cout << "Computed Cartesian deltas for link: " << link_name << std::endl;
  return true;
}

bool QuadbarKinematics::calculate_link_transform(
    const Eigen::VectorXd &joint_pos, const std::string &link_name,
    Eigen::Isometry3d &transform) {
  // Stub: Compute the link transform based on joint positions
  if (!validate_joint_positions(joint_pos)) {
    throw std::invalid_argument("Invalid joint positions");
  }

  auto &leg = legs_map_.at(link_name);
  auto index = get_leg_index(link_name) * 3;
  Eigen::Vector3d q = {joint_pos(index), joint_pos(index + 1),
                       joint_pos(index + 2)};
  leg.set_joints_states(q);

  auto x =leg.forward_kinematics();

  transform =
      Eigen::Translation3d(x) *
      Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());

  std::cout << "Computed transform for link: " << link_name << std::endl;
  return true;
}

bool QuadbarKinematics::calculate_jacobian(
    const Eigen::VectorXd &joint_pos, const std::string &link_name,
    Eigen::Matrix<double, 6, Eigen::Dynamic> &jacobian) {
  // Stub: Compute the Jacobian for the given link
  if (!validate_joint_positions(joint_pos)) {
    throw std::invalid_argument("Invalid joint positions");
  }

  jacobian = Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(
      6, joint_pos.size()); // Stub implementation
  std::cout << "Computed Jacobian for link: " << link_name << std::endl;
  return true;
}

bool QuadbarKinematics::calculate_jacobian_inverse(
    const Eigen::VectorXd &joint_pos, const std::string &link_name,
    Eigen::Matrix<double, Eigen::Dynamic, 6> &jacobian_inverse) {
  // Stub: Compute the Jacobian inverse for the given link
  if (!validate_joint_positions(joint_pos)) {
    throw std::invalid_argument("Invalid joint positions");
  }

  jacobian_inverse = Eigen::Matrix<double, Eigen::Dynamic, 6>::Zero(
      joint_pos.size(), 6); // Stub implementation
  std::cout << "Computed Jacobian inverse for link: " << link_name << std::endl;
  return true;
}

bool QuadbarKinematics::validate_joint_positions(
    const Eigen::VectorXd &joint_pos) const {
  // Stub: Add checks for valid joint positions if necessary
  return joint_pos.size() > 0;
}

QuadbarKinematics::LegIndex QuadbarKinematics::get_leg_index(const std::string &leg_name) const {
  if (leg_name == "front_left") {
    return LegIndex::FRONT_LEFT;
  } else if (leg_name == "front_right") {
    return LegIndex::FRONT_RIGHT;
  } else if (leg_name == "rear_left") {
    return LegIndex::REAR_LEFT;
  } else if (leg_name == "rear_right") {
    return LegIndex::REAR_RIGHT;
  } else {
    throw std::invalid_argument("Invalid leg name: " + leg_name);
  }
}



} // namespace quadbar_kinematics

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(quadbar_kinematics::QuadbarKinematics, kinematics_interface::KinematicsInterface)