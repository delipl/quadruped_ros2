#ifndef QUADRUPED_CONTROLLER_LEG_HPP
#define QUADRUPED_CONTROLLER_LEG_HPP

#include <cmath>
#include <limits>

#include <Eigen/Dense>

#include <sensor_msgs/msg/joint_state.hpp>

namespace quadruped_controller {

struct JointState {
  std::string name;
  double position;
  double velocity;
  double effort;
};

const double UPPER_BONE_LENGTH = 0.125;
const double LOWER_BONE_LENGTH = 0.21;
const double FOOT_LENGTH = 0.0;

class Leg {
public:
  Leg(const std::string &name);

  void set_positions_from_joint_states(
      const sensor_msgs::msg::JointState::SharedPtr msg);

  Eigen::Vector3d forward_kinematics(const Eigen::Vector3d &q);
  Eigen::Vector3d forward_kinematics() {
    return forward_kinematics(
        Eigen::Vector3d(first_.position, second_.position, third_.position));
  }

  Eigen::Vector3d inverse_kinematics(const Eigen::Vector3d &x);

  std::pair<JointState, JointState> get_passive_knee_joints() const {
    return {forth_, fifth_};
  }

  std::array<JointState, 5> get_joints_states() const {
    return {first_, second_, third_, forth_, fifth_};
  }

  std::array<JointState, 3> get_active_joint_states() {
    return {first_, second_, third_};
  }

  void set_joints_states(const Eigen::Vector3d &q,
                         const Eigen::Vector3d &dq = {0, 0, 0}) {
    first_.position = q(0);
    second_.position = q(1);
    third_.position = q(2);

    first_.velocity = dq(0);
    second_.velocity = dq(1);
    third_.velocity = dq(2);
  }

  double get_distance_to_effector() const { return distance_to_effector_; }

  std::string get_name() const { return name_; }
  Eigen::Matrix4d kinematics(const Eigen::Vector3d &q);
  Eigen::Matrix<double, 6, 3> jacobian(const Eigen::Vector3d &q_open);

private:
  const std::string name_;

  const double l1 = UPPER_BONE_LENGTH;
  const double l2 = UPPER_BONE_LENGTH;
  const double l3 = LOWER_BONE_LENGTH;
  const double l4 = LOWER_BONE_LENGTH;
  const double l5 = FOOT_LENGTH;

  const double d0 = 0.06;
  const double a0 = 0.06;
  const double a1 = 0.223;
  const double d2 = 0.0898;
  const double a2 = l1;
  const double a3 = l4;

  std::array<Eigen::DiagonalMatrix<double, 4>, 4> directions_;
  Eigen::Vector3d inv_directions_;
  double z_axis_q0_direction_ = 1.0;
  double z_axis_q1_direction_ = 1.0;
  double z_axis_q2_direction_ = 1.0;
  double passive_side_multiplier_ = 1.0;
  double third_joint_gear_correction_;

  JointState first_;
  JointState second_;
  JointState third_;
  JointState forth_;
  JointState fifth_;

  void update_effector_position(double q3);
  Eigen::Matrix4d denavite_hartenberg(double alpha, double a, double d,
                                      double theta);
  Eigen::Matrix4d denavite_hartenberg(const Eigen::Vector4d &v);
  double distance_to_effector_;
};

} // namespace quadruped_controller

#endif // QUADRUPED_CONTROLLER_LEG_HPP
