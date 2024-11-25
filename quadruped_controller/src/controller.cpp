#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

class JointStateListener : public rclcpp::Node {
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFollowJointTrajectory =
      rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  JointStateListener()
      : Node("joint_state_listener"),
        action_client_(rclcpp_action::create_client<FollowJointTrajectory>(
            this, "/joint_trajectory_controller/follow_joint_trajectory")) {
    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "control_joint_states", 10,
        std::bind(&JointStateListener::jointStateCallback, this,
                  std::placeholders::_1));
  }

private:
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (initialized_) {
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Received joint state");
    auto goal_msg = FollowJointTrajectory::Goal();
    trajectory_msgs::msg::JointTrajectory traj_msg;

    // Fill traj_msg with appropriate data
    traj_msg.joint_names = msg->name;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = msg->position;
    point.time_from_start = rclcpp::Duration(1, 0); // 1 second duration
    traj_msg.points.push_back(point);

    goal_msg.trajectory = traj_msg;

    if (!action_client_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Action server not available after waiting");
      return;
    }

    auto send_goal_options =
        rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(
        &JointStateListener::goalResponseCallback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&JointStateListener::feedbackCallback, this,
                  std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback = std::bind(
        &JointStateListener::resultCallback, this, std::placeholders::_1);

    action_client_->async_send_goal(goal_msg, send_goal_options);
    initialized_ = true;
  }

  void goalResponseCallback(
      const GoalHandleFollowJointTrajectory::SharedPtr &goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Goal accepted by server, waiting for result");
    }
  }

  void feedbackCallback(
      GoalHandleFollowJointTrajectory::SharedPtr,
      const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback) {
    RCLCPP_INFO(this->get_logger(), "Received feedback");
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Received feedback: "
                           << feedback->actual.positions[0] << " "
                           << feedback->actual.positions[1] << " "
                           << feedback->actual.positions[2]);
  }

  void
  resultCallback(const GoalHandleFollowJointTrajectory::WrappedResult &result) {
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
      initialized_ = false;
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(this->get_logger(), "Goal was canceled");
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      break;
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;

  bool initialized_ = false;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointStateListener>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
