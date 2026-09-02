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

#ifndef QUADRUPED_CONTROLLER__LEG_INTERFACE_HPP_
#define QUADRUPED_CONTROLLER__LEG_INTERFACE_HPP_

#include <Eigen/Dense>
#include <array>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>
#include <utility>
#include <vector>

namespace quadruped_controller
{

struct JointState
{
  std::string name;
  double position = 0.0;
  double velocity = 0.0;
  double effort = 0.0;
};

class LegInterface
{
public:
  virtual ~LegInterface() = default;

  virtual void initialize(const std::string & name) = 0;
  virtual void set_positions_from_joint_states(
    const sensor_msgs::msg::JointState::SharedPtr msg) = 0;
  virtual Eigen::Vector3d forward_kinematics(const Eigen::Vector3d & q) = 0;
  virtual Eigen::Vector3d forward_kinematics() = 0;
  virtual Eigen::Vector3d inverse_kinematics(const Eigen::Vector3d & x) = 0;
  virtual std::pair<JointState, JointState> get_passive_knee_joints() const = 0;
  virtual std::array<JointState, 5> get_joints_states() const = 0;
  virtual std::array<JointState, 3> get_active_joint_states() = 0;
  virtual void set_joints_states(
    const Eigen::Vector3d & q, const Eigen::Vector3d & v = Eigen::Vector3d::Zero(),
    const Eigen::Vector3d & tau = Eigen::Vector3d::Zero()) = 0;
  virtual double get_distance_to_effector() const = 0;
  virtual std::string get_name() const = 0;
  virtual Eigen::Matrix4d kinematics(const Eigen::Vector3d & q) = 0;
  virtual Eigen::Vector3d get_joints_directions() const = 0;
  virtual void set_parameters(const std::vector<double> & params) = 0;
};

}  // namespace quadruped_controller

#endif
