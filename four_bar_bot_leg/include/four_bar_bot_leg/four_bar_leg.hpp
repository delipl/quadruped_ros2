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

#ifndef FOUR_BAR_BOT_LEG__FOUR_BAR_LEG_HPP_
#define FOUR_BAR_BOT_LEG__FOUR_BAR_LEG_HPP_

#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

// Import interfejsu z quadruped_controller
#include "quadruped_controller/leg_plugin.hpp"

namespace four_bar_bot_leg
{

class FourBarLeg : public quadruped_controller::LegPlugin
{
public:
  FourBarLeg() = default;
  ~FourBarLeg() override = default;

  // Implementacja interfejsu z quadruped_controller
  void initialize(const std::string & name) override;

  void set_positions_from_joint_states(const sensor_msgs::msg::JointState::SharedPtr msg) override;

  Eigen::Vector3d forward_kinematics(const Eigen::Vector3d & q) override;
  Eigen::Vector3d forward_kinematics() override;
  Eigen::Vector3d inverse_kinematics(const Eigen::Vector3d & x) override;

  std::pair<quadruped_controller::JointState, quadruped_controller::JointState>
  get_passive_knee_joints() const override;

  std::array<quadruped_controller::JointState, 5> get_joints_states() const override;
  std::array<quadruped_controller::JointState, 3> get_active_joint_states() override;

  void set_joints_states(
    const Eigen::Vector3d & q, const Eigen::Vector3d & v = Eigen::Vector3d::Zero(),
    const Eigen::Vector3d & tau = Eigen::Vector3d::Zero()) override;

  double get_distance_to_effector() const override { return distance_to_effector_; }
  std::string get_name() const override { return name_; }
  Eigen::Matrix4d kinematics(const Eigen::Vector3d & q) override;
  Eigen::Vector3d get_joints_directions() const override;

  void set_parameters(const std::vector<double> & params) override;

private:
  std::string name_;

  // Długości członów (stałe dla four_bar)
  const double l1_ = 0.125;  // Upper bone
  const double l2_ = 0.125;  // Upper bone (second)
  const double l3_ = 0.21;   // Lower bone
  const double l4_ = 0.21;   // Lower bone (second)
  const double l5_ = 0.0;    // Foot

  // Parametry geometryczne
  const double d0_ = 0.06;
  const double a0_ = 0.06;
  const double a1_ = 0.223;
  const double d2_ = 0.0898;
  const double a2_ = 0.125;
  const double a3_ = 0.21;

  // Kierunki osi i współczynniki
  double z_axis_q0_direction_ = 1.0;
  double z_axis_q1_direction_ = 1.0;
  double z_axis_q2_direction_ = 1.0;
  double passive_side_multiplier_ = 1.0;
  double third_joint_gear_correction_ = M_PI;

  // Macierze kierunków DH
  std::array<Eigen::DiagonalMatrix<double, 4>, 4> directions_;
  Eigen::Vector3d inv_directions_;

  // Stany stawów
  quadruped_controller::JointState first_;
  quadruped_controller::JointState second_;
  quadruped_controller::JointState third_;
  quadruped_controller::JointState forth_;
  quadruped_controller::JointState fifth_;

  double distance_to_effector_ = 0.0;

  // Metody pomocnicze
  void update_passive_joints(double q3);
  void setup_directions();
  Eigen::Matrix4d denavite_hartenberg(double theta, double d, double a, double alpha);
  Eigen::Matrix4d denavite_hartenberg(const Eigen::Vector4d & v);
};

}  // namespace four_bar_bot_leg

#endif  // FOUR_BAR_BOT_LEG__FOUR_BAR_LEG_HPP_
