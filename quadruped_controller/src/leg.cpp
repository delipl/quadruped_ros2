
#include "quadruped_controller/leg.hpp"

#include <iostream>

namespace quadruped_controller {
Leg::Leg(const std::string &name) : name_(name) {

  if (name == "front_left") {
    z_axis_q0_direction_ = 1.0;
    z_axis_q1_direction_ = 1.0;
    z_axis_q2_direction_ = -1.0;
    passive_side_multiplier_ = -1.0;

    directions_[0].diagonal() << 1, 1, 1, 1;
    directions_[1].diagonal() << -1, 1, 1, 1;
    directions_[2].diagonal() << 1, 1, -1, 1;
    directions_[3].diagonal() << -1, 1, 1, 1;

    inv_directions_ << 1, 1, 1;

  } else if (name == "front_right") {
    z_axis_q0_direction_ = 1.0;
    z_axis_q1_direction_ = -1.0;
    z_axis_q2_direction_ = 1.0;
    passive_side_multiplier_ = 1.0;

    directions_[0].diagonal() << -1, 1, 1, 1;
    directions_[1].diagonal() << 1, 1, 1, 1;
    directions_[2].diagonal() << -1, -1, -1, 1;
    directions_[3].diagonal() << 1, -1, 1, 1;

    inv_directions_ << 1, -1, 1;

  } else if (name == "rear_left") {
    z_axis_q0_direction_ = -1.0;
    z_axis_q1_direction_ = 1.0;
    z_axis_q2_direction_ = 1.0;
    passive_side_multiplier_ = 1.0;

    directions_[0].diagonal() << 1, 1, 1, 1;
    directions_[1].diagonal() << -1, 1, -1, 1;
    directions_[2].diagonal() << -1, 1, 1, -1;
    directions_[3].diagonal() << -1, 1, -1, 1;

    inv_directions_ << -1, 1, -1;

  } else if (name == "rear_right") {
    z_axis_q0_direction_ = -1.0;
    z_axis_q1_direction_ = -1.0;
    z_axis_q2_direction_ = -1.0;
    passive_side_multiplier_ = -1.0;

    directions_[0].diagonal() << -1, 1, 1, 1;
    directions_[1].diagonal() << 1, 1, -1, 1;
    directions_[2].diagonal() << 1, -1, 1, -1;
    directions_[3].diagonal() << 1, -1, -1, 1;

    inv_directions_ << -1, -1, -1;
  }

  const double gear_ratio = 1.0 / 6.0;
  third_joint_gear_correction_ = 3 * gear_ratio * 2 * M_PI;

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
      //   std::cout << "Hip side position: " << first_.position <<
      //   std::endl; first_.velocity = msg->velocity[i]; first_.effort =
      //   msg->effort[i];
    } else if (msg->name[i] == second_.name) {
      second_.position = msg->position[i];
      //   std::cout << "Hip forward position: " << second_.position
      //             << std::endl;
      //   second_.velocity = msg->velocity[i];
      //   second_.effort = msg->effort[i];
    } else if (msg->name[i] == third_.name) {
      third_.position = msg->position[i];

      //   std::cout << "Knee position: " << third_.position << std::endl;
      //   third_.velocity = msg->velocity[i];
      //   third_.effort = msg->effort[i];
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
  // TODO: warning this pi will be negative in another side

  Eigen::Vector4d v_A01 = {M_PI_2, d0, a0, 0.0};
  Eigen::Vector4d v_A12 = {M_PI_2, 0.0, a1,
                           z_axis_q0_direction_ * q_open(0) - M_PI_2};
  Eigen::Vector4d v_A23 = {z_axis_q1_direction_ * q_open(1) + M_PI, d2, a2,
                           0.0};
  Eigen::Vector4d v_A34 = {q_open(2), 0.0, a3, 0.0};

  auto A01 = denavite_hartenberg(directions_[0] * v_A01);
  auto A12 = denavite_hartenberg(directions_[1] * v_A12);
  auto A23 = denavite_hartenberg(directions_[2] * v_A23);
  auto A34 = denavite_hartenberg(directions_[3] * v_A34);

  return A01 * A12 * A23 * A34;
}

Eigen::Vector3d Leg::forward_kinematics(const Eigen::Vector3d &q) {
  update_effector_position(q(2));

  Eigen::Vector3d q_open = {q(0), q(1), fifth_.position};

  auto K = kinematics(q_open);

  return K.block<3, 1>(0, 3);
}

Eigen::Matrix<double, 6, 3> Leg::jacobian(const Eigen::Vector3d &q_open) {
  // DH parameters
  Eigen::Vector4d v_A01 = {M_PI_2, d0, a0, 0.0};
  Eigen::Vector4d v_A12 = {M_PI_2, 0.0, a1,
                           z_axis_q0_direction_ * q_open(0) - M_PI_2};
  Eigen::Vector4d v_A23 = {z_axis_q1_direction_ * q_open(1) + M_PI, d2, a2,
                           0.0};
  Eigen::Vector4d v_A34 = {q_open(2), 0.0, a3, 0.0};

  // Transformations
  Eigen::Matrix4d A01 = denavite_hartenberg(directions_[0] * v_A01);
  Eigen::Matrix4d A12 = denavite_hartenberg(directions_[1] * v_A12);
  Eigen::Matrix4d A23 = denavite_hartenberg(directions_[2] * v_A23);
  Eigen::Matrix4d A34 = denavite_hartenberg(directions_[3] * v_A34);

  Eigen::Matrix4d T01 = A01;
  Eigen::Matrix4d T02 = T01 * A12;
  Eigen::Matrix4d T03 = T02 * A23;
  Eigen::Matrix4d T04 = T03 * A34;

  // Position of end-effector
  Eigen::Vector3d p0 = Eigen::Vector3d::Zero();
  Eigen::Vector3d p1 = T01.block<3, 1>(0, 3);
  Eigen::Vector3d p2 = T02.block<3, 1>(0, 3);
  Eigen::Vector3d p3 = T03.block<3, 1>(0, 3);
  Eigen::Vector3d pe = T04.block<3, 1>(0, 3);

  // Z axes of each joint
  Eigen::Vector3d z0 = Eigen::Vector3d::UnitZ(); // base frame
  Eigen::Vector3d z1 = T01.block<3, 1>(0, 2);
  Eigen::Vector3d z2 = T02.block<3, 1>(0, 2);

  // Jacobian columns
  Eigen::Matrix<double, 6, 3> J;

  J.block<3, 1>(0, 0) = z0.cross(pe - p0); // linear
  J.block<3, 1>(3, 0) = z0;                // angular

  J.block<3, 1>(0, 1) = z1.cross(pe - p1);
  J.block<3, 1>(3, 1) = z1;

  J.block<3, 1>(0, 2) = z2.cross(pe - p2);
  J.block<3, 1>(3, 2) = z2;

  return J;
}

// Page 87 of
// http://160592857366.free.fr/joe/ebooks/Mechanical%20Engineering%20Books%20Collection/THEORY%20OF%20MACHINES/machines%20and%20mechanisms.pdf
void Leg::update_effector_position(double q3) {
  const double th2 = M_PI -
                     passive_side_multiplier_ * z_axis_q2_direction_ * q3 +
                     third_joint_gear_correction_;
  const auto c2 = std::cos(th2);
  const auto s2 = std::sin(th2);

  const auto BD = std::sqrt(l1 * l1 + l2 * l2 - 2 * l1 * l2 * c2);
  const auto gamma = std::acos((l3 * l3 + l4 * l4 - BD * BD) / (2 * l3 * l4));
  const auto sg = std::sin(gamma);
  const auto cg = std::cos(gamma);

  const auto th3 =
      2 * std::atan2(-l2 * s2 + l4 * sg, l1 + l3 - l2 * c2 - l4 * cg);
  const auto th4 =
      2 * std::atan2(l2 * s2 - l3 * sg, l2 * c2 + l4 - l1 - l3 * cg);

  fifth_.position = -passive_side_multiplier_ * (M_PI - th2 + th3);
  forth_.position = passive_side_multiplier_ * (M_PI - th4);
}

// Szrek PhD thesis
Eigen::Vector3d Leg::inverse_kinematics(const Eigen::Vector3d &x) {
  Eigen::Vector3d q;

  Eigen::Vector3d x_foot = x;
  //  -0.0105???

  //  Move to the legs base frame
  x_foot << x.x() - inv_directions_.x() * a1, x.y() - inv_directions_.y() * a0,
      x.z() - d0;

  // 0.0125???
  const double xe = inv_directions_.x() * x_foot(0);
  // -      inv_directions_.z() * inv_directions_.x() * (0.031 + 0.064 +
  // 0.0125);

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

  // FIXME: REAR LEGS INV
  const double yb1 = (-T + sqrt_delta) / (2 * V);
  const double yb2 = (-T - sqrt_delta) / (2 * V);
  const double xb1 = std::sqrt(l_AB * l_AB - yb1 * yb1);
  const double xb2 = std::sqrt(l_AB * l_AB - yb2 * yb2);

  double xb = yb1;
  double yb = xb1;

  Eigen::Vector2d b = {xb, yb};
  auto theta_b = std::atan2(b(1), b(0));
  const auto q1_dir = 1.0;
  q(1) = (-theta_b);

  // Check if angle is correct
  const auto ze_based_on_q1 =
      l_AB * std::sin(q(1)) + l_BE * std::sin(M_PI + q(1) + phi);
  const auto xe_based_on_q1 =
      l_AB * std::cos(q(1)) + l_BE * std::cos(M_PI + q(1) + phi);

  const auto ze_error = std::abs(ze - ze_based_on_q1);
  const auto xe_error = std::abs(xe - xe_based_on_q1);
  const auto is_error = (ze_error > 0.001 || xe_error > 0.001);
  if ((is_error && q1_dir < 0) || (!is_error && q1_dir > 0)) {
    yb *= -1;
    b << b(0), -b(1);
    theta_b = std::atan2(b(1), b(0));
    q(1) = q1_dir * (-theta_b);
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

  q(2) = (-beta - alpha + M_PI + third_joint_gear_correction_);

  // Normalize angles

  return q;
}

} // namespace quadruped_controller

// ros2 topic pub /effector_position  geometry_msgs/msg/PoseStamped  "{ header:
// {stamp: now,  frame_id: front_left_second_link},  pose: { position: { x:
// 0.064, y: 0.2, z: 0.0785 } }}"