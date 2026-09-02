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

#include "quadruped_controller/leg_plugin.hpp"

#include <iostream>

namespace quadruped_controller
{

void LegPlugin::initialize(const std::string & name)
{
  name_ = name;

  first_.name = name + "_first_joint";
  second_.name = name + "_second_joint";
  third_.name = name + "_third_joint";
  forth_.name = name + "_fourth_joint";
  fifth_.name = name + "_fifth_joint";
}

void LegPlugin::set_parameters(const std::vector<double> & params)
{
  // Domyślna implementacja - można nadpisać w klasach pochodnych
  (void)params;
}

void LegPlugin::set_positions_from_joint_states(const sensor_msgs::msg::JointState::SharedPtr msg)
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

std::pair<JointState, JointState> LegPlugin::get_passive_knee_joints() const
{
  return {forth_, fifth_};
}

std::array<JointState, 5> LegPlugin::get_joints_states() const
{
  return {first_, second_, third_, forth_, fifth_};
}

std::array<JointState, 3> LegPlugin::get_active_joint_states() { return {first_, second_, third_}; }

void LegPlugin::set_joints_states(
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

  // Aktualizuj pasywne stawy
  update_passive_joints(q(2));
}

}  // namespace quadruped_controller
