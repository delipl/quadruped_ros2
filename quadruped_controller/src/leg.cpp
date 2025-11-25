
#include "quadruped_controller/leg.hpp"

#include <iostream>

namespace quadruped_controller {
Leg::Leg(const std::string &name) : name_(name) {

  if (name == "front_left") {
    // same likeRR
    z_axis_q0_direction_ = 1.0;
    z_axis_q1_direction_ = -1.0;
    z_axis_q2_direction_ = -1.0;
    passive_side_multiplier_ = -1.0;

    directions_[0].diagonal() << 1, 1, 1, 1;
    directions_[1].diagonal() << -1, 1, 1, 1;
    directions_[2].diagonal() << -1, 1, -1, 1;
    directions_[3].diagonal() << 1, 1, 1, 1;

    inv_directions_ << 1, 1, 0;

  } else if (name == "front_right") {
    // same like RL
    z_axis_q0_direction_ = 1.0;
    z_axis_q1_direction_ = 1.0;
    z_axis_q2_direction_ = 1.0;
    passive_side_multiplier_ = 1.0;

    directions_[0].diagonal() << -1, 1, 1, 1;
    directions_[1].diagonal() << 1, 1, 1, 1;
    directions_[2].diagonal() << -1, -1, -1, 1;
    directions_[3].diagonal() << 1, -1, 1, 1;

    inv_directions_ << 1, -1, 0;

  } else if (name == "rear_left") {
    z_axis_q0_direction_ = -1.0;
    z_axis_q1_direction_ = 1.0;
    z_axis_q2_direction_ = 1.0;
    passive_side_multiplier_ = 1.0;

    directions_[0].diagonal() << 1, 1, 1, 1;
    directions_[1].diagonal() << -1, 1, -1, 1;
    directions_[2].diagonal() << 1, 1, 1, -1;
    directions_[3].diagonal() << -1, 1, -1, 1;

    inv_directions_ << -1, 1, 0;

  } else if (name == "rear_right") {
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

  third_joint_gear_correction_ = -1.117+M_PI;

  first_.name = name + "_first_joint";
  second_.name = name + "_second_joint";
  third_.name = name + "_third_joint";

  forth_.name = name + "_fourth_joint";
  fifth_.name = name + "_fifth_joint";
}

void Leg::set_positions_from_joint_states(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  for (size_t i = 0; i < msg->name.size(); ++i) {
    if (msg->name[i] == first_.name) {
      first_.position = msg->position[i];
    } else if (msg->name[i] == second_.name) {
      second_.position = msg->position[i];
    } else if (msg->name[i] == third_.name) {
      third_.position = msg->position[i];
    }
  }
}

Eigen::Matrix4d Leg::denavite_hartenberg(double theta, double d, double a,
                                         double alpha) {
  Eigen::Matrix4d T;

  T << cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha),
      a * cos(theta), sin(theta), cos(theta) * cos(alpha),
      -cos(theta) * sin(alpha), a * sin(theta), 0, sin(alpha), cos(alpha), d, 0,
      0, 0, 1;

  return T;
}

Eigen::Matrix4d Leg::denavite_hartenberg(const Eigen::Vector4d &v) {
  Eigen::Matrix4d T;
  const auto theta = v(0);
  const auto d = v(1);
  const auto a = v(2);
  const auto alpha = v(3);

  T << cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha),
      a * cos(theta), sin(theta), cos(theta) * cos(alpha),
      -cos(theta) * sin(alpha), a * sin(theta), 0, sin(alpha), cos(alpha), d, 0,
      0, 0, 1;

  return T;
}

Eigen::Matrix4d Leg::kinematics(const Eigen::Vector3d &q_open) {
  //  TODO: @delipl check if kinematics works after changing default values 
  Eigen::Vector4d v_A01 = {M_PI_2, d0, a0, 0.0};
  Eigen::Vector4d v_A12 = {M_PI_2, 0.0, a1,
                           z_axis_q0_direction_ * q_open(0) - M_PI_2};
  Eigen::Vector4d v_A23 = {z_axis_q1_direction_ * q_open(1) + M_PI  -0.2793 , d2, a2,
                           0.0};
  Eigen::Vector4d v_A34 = {z_axis_q2_direction_ * q_open(2), 0.0, a3, 0.0};

  auto A01 = denavite_hartenberg(directions_[0] * v_A01);
  auto A12 = denavite_hartenberg(directions_[1] * v_A12);
  auto A23 = denavite_hartenberg(directions_[2] * v_A23);
  auto A34 = denavite_hartenberg(directions_[3] * v_A34);

  return A01 * A12 * A23 * A34;
}

Eigen::Vector3d Leg::forward_kinematics(const Eigen::Vector3d &q) {
  update_passive_joints(q(2));

  Eigen::Vector3d q_open = {q(0), q(1), fifth_.position};

  auto K = kinematics(q_open);

  return K.block<3, 1>(0, 3);
}

// Page 87 of
// http://160592857366.free.fr/joe/ebooks/Mechanical%20Engineering%20Books%20Collection/THEORY%20OF%20MACHINES/machines%20and%20mechanisms.pdf
void Leg::update_passive_joints(double q3) {
  const double th2 = M_PI -
                     passive_side_multiplier_ * q3 +
                     third_joint_gear_correction_;
  const auto c2 = std::cos(th2);
  const auto s2 = std::sin(th2);

  const auto BD = std::sqrt(l1 * l1 + l2 * l2 - 2 * l1 * l2 * c2);
  const auto gamma = std::acos((l3 * l3 + l4 * l4 - BD * BD) / (2 * l3 * l4));
  const auto sg = std::sin(gamma);
  const auto cg = std::cos(gamma);

  const auto th1 =
      2 * std::atan2(l2 * s2 - l3 * sg, l2 * c2 + l4 - l1 - l3 * cg);
  const auto th3 =
      2 * std::atan2(l4 * sg - l2 * s2, l1 + l3 - l2 * c2 - l4 * cg);

  fifth_.position = -passive_side_multiplier_ * (M_PI - th2 + th3);
  forth_.position = passive_side_multiplier_ * (M_PI - th1);
}

// Szrek PhD thesis
Eigen::Vector3d Leg::inverse_kinematics(const Eigen::Vector3d &x) {
  Eigen::Vector3d q;

  Eigen::Vector3d x_foot = x;

  //  Move to the legs base frame
  x_foot << x.x() - inv_directions_.x() * a1, x.y() - inv_directions_.y() * a0,
      x.z() - d0;

  const double xe = inv_directions_.x() * x_foot(0);
  const double ye = x_foot.y();
  const double ze_b = x_foot.z();

  const double d_e = std::sqrt(ze_b * ze_b + ye * ye);

  double phi_0 = std::acos(d2 / d_e);
  double kappa = std::atan2(x_foot.z(), inv_directions_.y() * x_foot.y());

  q(0) = inv_directions_.y() * z_axis_q0_direction_ * (phi_0 + kappa);

  // Fix y and z for 1, 2 joints
  double ze = -std::sqrt(d_e * d_e - d2 * d2);

  const double l_BE = l4 + l5;
  const double l_AB = l1;

  const double xe2 = xe * xe;
  const double ze2 = ze * ze;

  const double Q = (l_AB * l_AB - l_BE * l_BE + ze2 + xe2) / (2 * ze);
  const double W = Q * Q - l_AB * l_AB;
  const double T = -(2 * Q * xe) / (ze);
  const double V = 1 + (xe2) / (ze2);

  const double delta = T * T - 4 * V * W;
  const double sqrt_delta = std::sqrt(delta);

  double e_dist = std::sqrt(xe * xe + ze * ze);

  const double phi = std::acos((l_AB * l_AB + l_BE * l_BE - e_dist * e_dist) /
                               (2 * l_AB * l_BE));

  const double yb1 = (-T + sqrt_delta) / (2 * V);
  const double yb2 = (-T - sqrt_delta) / (2 * V);
  const double xb1 = std::sqrt(l_AB * l_AB - yb1 * yb1);
  const double xb2 = std::sqrt(l_AB * l_AB - yb2 * yb2);

  double xb = yb1;
  double yb = xb1;

  Eigen::Vector2d b = {xb, yb};
  auto theta_b = std::atan2(b(1), b(0));
  const auto q1_dir = 1.0;
  q(1) = z_axis_q1_direction_ * theta_b + passive_side_multiplier_*0.2793;

  // This part is to check if we have chosen the correct solution among the two possible
  // If the solution is not correct, the inverse kinematics is roteted by 90 degrees in the middle
  // Check if angle is correct
  const auto ze_based_on_q1 =
      l_AB * std::sin(theta_b) + l_BE * std::sin(M_PI + theta_b + phi);
  const auto xe_based_on_q1 =
      l_AB * std::cos(theta_b) + l_BE * std::cos(M_PI + theta_b + phi);

  const auto ze_error = std::abs(ze - ze_based_on_q1);
  const auto xe_error = std::abs(xe - xe_based_on_q1);
  const auto is_error = (ze_error > 0.001 || xe_error > 0.001);
  if ((is_error && q1_dir > 0) || (!is_error && q1_dir < 0)) {
    b << b(0), -b(1);
    theta_b = std::atan2(b(1), b(0));
    q(1) = z_axis_q1_direction_ * (theta_b + 0.2793);
  }


  const Eigen::Vector2d e = {xe, ze};
  const Eigen::Vector2d be = e - b;

  const double gamma = std::atan2(be(1), (be(0)));
  const double l_BC = l4;
  Eigen::Vector2d C =
      b + Eigen::Vector2d{std::cos(gamma) * l_BC, std::sin(gamma) * l_BC};
  const auto c = C.norm();
  const auto alpha = std::acos((c * c + l2 * l2 - l3 * l3) / (2 * c * l2));
  const auto beta = std::acos((c * c + l1 * l1 - l4 * l4) / (2 * c * l1));

  q(2) = z_axis_q2_direction_ * (-beta - alpha + M_PI + third_joint_gear_correction_);

  return q;
}

} // namespace quadruped_controller
