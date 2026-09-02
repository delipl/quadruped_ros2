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

#include "four_bar_bot_leg/four_bar_leg.hpp"

#include <iostream>

namespace four_bar_bot_leg
{

void FourBarLeg::setup_directions()
{
  if (name_ == "front_left") {
    z_axis_q0_direction_ = 1.0;
    z_axis_q1_direction_ = -1.0;
    z_axis_q2_direction_ = -1.0;
    passive_side_multiplier_ = -1.0;

    directions_[0].diagonal() << 1, 1, 1, 1;
    directions_[1].diagonal() << -1, 1, 1, 1;
    directions_[2].diagonal() << -1, 1, -1, 1;
    directions_[3].diagonal() << 1, 1, 1, 1;

    inv_directions_ << 1, 1, 0;

  } else if (name_ == "front_right") {
    z_axis_q0_direction_ = 1.0;
    z_axis_q1_direction_ = 1.0;
    z_axis_q2_direction_ = 1.0;
    passive_side_multiplier_ = 1.0;

    directions_[0].diagonal() << -1, 1, 1, 1;
    directions_[1].diagonal() << 1, 1, 1, 1;
    directions_[2].diagonal() << -1, -1, -1, 1;
    directions_[3].diagonal() << 1, -1, 1, 1;

    inv_directions_ << 1, -1, 0;

  } else if (name_ == "rear_left") {
    z_axis_q0_direction_ = -1.0;
    z_axis_q1_direction_ = 1.0;
    z_axis_q2_direction_ = 1.0;
    passive_side_multiplier_ = 1.0;

    directions_[0].diagonal() << 1, 1, 1, 1;
    directions_[1].diagonal() << -1, 1, -1, 1;
    directions_[2].diagonal() << 1, 1, 1, -1;
    directions_[3].diagonal() << -1, 1, -1, 1;

    inv_directions_ << -1, 1, 0;

  } else if (name_ == "rear_right") {
    z_axis_q0_direction_ = -1.0;
    z_axis_q1_direction_ = -1.0;
    z_axis_q2_direction_ = -1.0;
    passive_side_multiplier_ = -1.0;

    directions_[0].diagonal() << -1, 1, 1, 1;
    directions_[1].diagonal() << 1, 1, -1, 1;
    directions_[2].diagonal() << 1, -1, 1, -1;
    directions_[3].diagonal() << -1, -1, -1, 1;

    inv_directions_ << -1, -1, 0;
  }

  // Ustawienie korekty przekładni trzeciego stawu
  double init_value = 3.09;
  third_joint_gear_correction_ = passive_side_multiplier_ > 0 ? init_value
                                                              : init_value - 1.198 * 2.0;
  third_joint_gear_correction_ = M_PI;
}

void FourBarLeg::initialize(const std::string & name)
{
  name_ = name;

  setup_directions();

  first_.name = name + "_first_joint";
  second_.name = name + "_second_joint";
  third_.name = name + "_third_joint";
  forth_.name = name + "_fourth_joint";
  fifth_.name = name + "_fifth_joint";
}

void FourBarLeg::set_parameters(const std::vector<double> & params)
{
  // Możliwość zmiany parametrów - na razie puste, bo używamy stałych
  (void)params;
}

void FourBarLeg::set_positions_from_joint_states(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  for (size_t i = 0; i < msg->name.size(); ++i) {
    if (msg->name[i] == first_.name) {
      first_.position = msg->position[i];
      first_.velocity = msg->velocity[i];
      first_.effort = msg->effort[i];
    } else if (msg->name[i] == second_.name) {
      second_.position = msg->position[i];
      second_.velocity = msg->velocity[i];
      second_.effort = msg->effort[i];
    } else if (msg->name[i] == third_.name) {
      third_.position = msg->position[i];
      third_.velocity = msg->velocity[i];
      third_.effort = msg->effort[i];
    } else if (msg->name[i] == forth_.name) {
      forth_.position = msg->position[i];
      forth_.velocity = msg->velocity[i];
      forth_.effort = msg->effort[i];
    } else if (msg->name[i] == fifth_.name) {
      fifth_.position = msg->position[i];
      fifth_.velocity = msg->velocity[i];
      fifth_.effort = msg->effort[i];
    }
  }
}

Eigen::Matrix4d FourBarLeg::denavite_hartenberg(double theta, double d, double a, double alpha)
{
  Eigen::Matrix4d T;

  T << cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta), sin(theta),
    cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta), 0, sin(alpha), cos(alpha), d,
    0, 0, 0, 1;

  return T;
}

Eigen::Matrix4d FourBarLeg::denavite_hartenberg(const Eigen::Vector4d & v)
{
  Eigen::Matrix4d T;
  const auto theta = v(0);
  const auto d = v(1);
  const auto a = v(2);
  const auto alpha = v(3);

  T << cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta), sin(theta),
    cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta), 0, sin(alpha), cos(alpha), d,
    0, 0, 0, 1;

  return T;
}

Eigen::Matrix4d FourBarLeg::kinematics(const Eigen::Vector3d & q_open)
{
  Eigen::Vector4d v_A01 = {M_PI_2, d0_, a0_, 0.0};
  Eigen::Vector4d v_A12 = {M_PI_2, 0.0, a1_, z_axis_q0_direction_ * q_open(0) - M_PI_2};
  Eigen::Vector4d v_A23 = {z_axis_q1_direction_ * q_open(1) + M_PI, d2_, a2_, 0.0};
  Eigen::Vector4d v_A34 = {z_axis_q2_direction_ * q_open(2), 0.0, a3_, 0.0};

  auto A01 = denavite_hartenberg(directions_[0] * v_A01);
  auto A12 = denavite_hartenberg(directions_[1] * v_A12);
  auto A23 = denavite_hartenberg(directions_[2] * v_A23);
  auto A34 = denavite_hartenberg(directions_[3] * v_A34);

  return A01 * A12 * A23 * A34;
}

Eigen::Vector3d FourBarLeg::forward_kinematics(const Eigen::Vector3d & q)
{
  update_passive_joints(q(2));

  Eigen::Vector3d q_open = {q(0), q(1), fifth_.position};

  auto K = kinematics(q_open);

  distance_to_effector_ = K(0, 3);

  return K.block<3, 1>(0, 3);
}

Eigen::Vector3d FourBarLeg::forward_kinematics()
{
  return forward_kinematics(Eigen::Vector3d(first_.position, second_.position, third_.position));
}

Eigen::Vector3d FourBarLeg::get_joints_directions() const
{
  Eigen::Vector3d directions;

  // NOTE: @delipl I have no idea why the first joint direction is the same as inverted passive
  // side multiplier but it works
  directions << -passive_side_multiplier_, z_axis_q1_direction_, z_axis_q2_direction_;
  return directions;
}

// Page 87 of
// http://160592857366.free.fr/joe/ebooks/Mechanical%20Engineering%20Books%20Collection/THEORY%20OF%20MACHINES/machines%20and%20mechanisms.pdf
void FourBarLeg::update_passive_joints(double q3)
{
  const double th2 = passive_side_multiplier_ * (q3);
  const auto c2 = std::cos(th2);
  const auto s2 = std::sin(th2);

  const auto BD = std::sqrt(l1_ * l1_ + l2_ * l2_ - 2 * l1_ * l2_ * c2);
  const auto gamma = std::acos((l3_ * l3_ + l4_ * l4_ - BD * BD) / (2 * l3_ * l4_));
  const auto sg = -std::sin(gamma);
  const auto cg = std::cos(gamma);

  const auto th1 = 2 * std::atan2(l2_ * s2 - l3_ * sg, l2_ * c2 + l4_ - l1_ - l3_ * cg);
  const auto th3 = 2 * std::atan2(l4_ * sg - l2_ * s2, l1_ + l3_ - l2_ * c2 - l4_ * cg);

  fifth_.position = passive_side_multiplier_ * (M_PI - th2 + th3);
  forth_.position = -passive_side_multiplier_ * (M_PI - th1);
}

// Szrek PhD thesis
Eigen::Vector3d FourBarLeg::inverse_kinematics(const Eigen::Vector3d & x)
{
  Eigen::Vector3d q;

  Eigen::Vector3d x_foot = x;

  // Move to the legs base frame
  x_foot << x.x() - inv_directions_.x() * a1_, x.y() - inv_directions_.y() * a0_, x.z() - d0_;

  const double xe = inv_directions_.x() * x_foot(0);
  const double ye = x_foot.y();
  const double ze_b = x_foot.z();

  const double d_e = std::sqrt(ze_b * ze_b + ye * ye);

  double phi_0 = std::acos(d2_ / d_e);
  double kappa = std::atan2(x_foot.z(), inv_directions_.y() * x_foot.y());

  q(0) = inv_directions_.y() * z_axis_q0_direction_ * (phi_0 + kappa);

  // Fix y and z for 1, 2 joints
  double ze = -std::sqrt(d_e * d_e - d2_ * d2_);

  const double l_BE = l4_ + l5_;
  const double l_AB = l1_;

  const double xe2 = xe * xe;
  const double ze2 = ze * ze;

  const double Q = (l_AB * l_AB - l_BE * l_BE + ze2 + xe2) / (2 * ze);
  const double W = Q * Q - l_AB * l_AB;
  const double T = -(2 * Q * xe) / (ze);
  const double V = 1 + (xe2) / (ze2);

  const double delta = T * T - 4 * V * W;
  const double sqrt_delta = std::sqrt(delta);

  double e_dist = std::sqrt(xe * xe + ze * ze);

  const double phi = std::acos((l_AB * l_AB + l_BE * l_BE - e_dist * e_dist) / (2 * l_AB * l_BE));

  const double yb1 = (-T + sqrt_delta) / (2 * V);
  // const double yb2 = (-T - sqrt_delta) / (2 * V);
  const double xb1 = std::sqrt(l_AB * l_AB - yb1 * yb1);
  // const double xb2 = std::sqrt(l_AB * l_AB - yb2 * yb2);

  double xb = yb1;
  double yb = xb1;

  Eigen::Vector2d b = {xb, yb};
  auto theta_b = std::atan2(b(1), b(0));
  const auto q1_dir = 1.0;
  q(1) = z_axis_q1_direction_ * theta_b;

  // This part is to check if we have chosen the correct solution among the two possible
  // If the solution is not correct, the inverse kinematics is rotated by 90 degrees in the middle
  // Check if angle is correct
  const auto ze_based_on_q1 = l_AB * std::sin(theta_b) + l_BE * std::sin(M_PI + theta_b + phi);
  const auto xe_based_on_q1 = l_AB * std::cos(theta_b) + l_BE * std::cos(M_PI + theta_b + phi);

  const auto ze_error = std::abs(ze - ze_based_on_q1);
  const auto xe_error = std::abs(xe - xe_based_on_q1);
  const auto is_error = (ze_error > 0.001 || xe_error > 0.001);
  if ((is_error && q1_dir > 0) || (!is_error && q1_dir < 0)) {
    b << b(0), -b(1);
    theta_b = std::atan2(b(1), b(0));
    q(1) = z_axis_q1_direction_ * (theta_b);
  }

  const Eigen::Vector2d e = {xe, ze};
  const Eigen::Vector2d be = e - b;

  const double gamma = std::atan2(be(1), (be(0)));
  const double l_BC = l4_;
  Eigen::Vector2d C = b + Eigen::Vector2d{std::cos(gamma) * l_BC, std::sin(gamma) * l_BC};
  const auto c = C.norm();
  const auto alpha = std::acos((c * c + l2_ * l2_ - l3_ * l3_) / (2 * c * l2_));
  const auto beta = std::acos((c * c + l1_ * l1_ - l4_ * l4_) / (2 * c * l1_));

  q(2) = z_axis_q2_direction_ * (-beta - alpha + M_PI + third_joint_gear_correction_);

  return q;
}

std::pair<quadruped_controller::JointState, quadruped_controller::JointState>
FourBarLeg::get_passive_knee_joints() const
{
  return {forth_, fifth_};
}

std::array<quadruped_controller::JointState, 5> FourBarLeg::get_joints_states() const
{
  return {first_, second_, third_, forth_, fifth_};
}

std::array<quadruped_controller::JointState, 3> FourBarLeg::get_active_joint_states()
{
  return {first_, second_, third_};
}

void FourBarLeg::set_joints_states(
  const Eigen::Vector3d & q, const Eigen::Vector3d & v, const Eigen::Vector3d & tau)
{
  first_.position = q(0);
  second_.position = q(1);
  third_.position = q(2);

  first_.velocity = v(0);
  second_.velocity = v(1);
  third_.velocity = v(2);

  first_.effort = tau(0);
  second_.effort = tau(1);
  third_.effort = tau(2);

  // Aktualizuj pasywne stawy na podstawie q2 (third joint)
  update_passive_joints(q(2));
}

}  // namespace four_bar_bot_leg

// Rejestracja pluginu - używamy interfejsu z quadruped_controller
PLUGINLIB_EXPORT_CLASS(four_bar_bot_leg::FourBarLeg, quadruped_controller::LegInterface)
