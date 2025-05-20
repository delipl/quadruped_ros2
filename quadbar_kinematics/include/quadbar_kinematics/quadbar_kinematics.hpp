// Copyright Jakub Delicat
// Copyright (c) 2022, PickNik, Inc.
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
//

#ifndef QUADBAR_KINEMATICS__QUADBAR_KINEMATICS_HPP_
#define QUADBAR_KINEMATICS__QUADBAR_KINEMATICS_HPP_

#include "kinematics_interface/kinematics_interface.hpp"

#include <array>
#include <string>
#include <vector>

#include "quadbar_kinematics/leg.hpp"

namespace quadbar_kinematics {

class QuadbarKinematics : public kinematics_interface::KinematicsInterface {
public:
  QuadbarKinematics() = default;
  ~QuadbarKinematics() override = default;

  enum LegIndex {
    FRONT_LEFT = 0,
    FRONT_RIGHT,
    REAR_LEFT,
    REAR_RIGHT,
    LEG_COUNT
  };

  bool
  initialize(std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface>
                 parameters_interface,
             const std::string &end_effector_name) override;

  bool convert_cartesian_deltas_to_joint_deltas(
      const Eigen::VectorXd &joint_pos,
      const Eigen::Matrix<double, 6, 1> &delta_x, const std::string &link_name,
      Eigen::VectorXd &delta_theta) override;

  bool convert_joint_deltas_to_cartesian_deltas(
      const Eigen::VectorXd &joint_pos, const Eigen::VectorXd &delta_theta,
      const std::string &link_name,
      Eigen::Matrix<double, 6, 1> &delta_x) override;

  bool calculate_link_transform(const Eigen::VectorXd &joint_pos,
                                const std::string &link_name,
                                Eigen::Isometry3d &transform) override;

  bool calculate_jacobian(
      const Eigen::VectorXd &joint_pos, const std::string &link_name,
      Eigen::Matrix<double, 6, Eigen::Dynamic> &jacobian) override;

  // It will be avaiable after sync
  bool calculate_jacobian_inverse(const Eigen::VectorXd &joint_pos,
                                  const std::string &link_name,
                                  Eigen::Matrix<double, Eigen::Dynamic, 6>
                                      &jacobian_inverse) /* override */;

private:
  // Private helper methods specific to QuadbarKinematics can be added here
  bool validate_joint_positions(const Eigen::VectorXd &joint_pos) const;
  LegIndex get_leg_index(const std::string &leg_name) const;



  std::map<std::string, quadbar_kinematics::Leg> legs_map_;
};

} // namespace quadbar_kinematics

#endif // QUADBAR_KINEMATICS__QUADBAR_KINEMATICS_HPP_
