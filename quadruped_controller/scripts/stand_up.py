import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class ZeroTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__("zero_trajectory_publisher")

        # Publisher for the joint_trajectory_controller
        self.publisher_ = self.create_publisher(
            JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10
        )

        # Define joint names
        self.joint_names = [
            "front_left_first_joint",
            "front_left_second_joint",
            "front_left_third_joint",
            "front_right_first_joint",
            "front_right_second_joint",
            "front_right_third_joint",
            "rear_left_first_joint",
            "rear_left_second_joint",
            "rear_left_third_joint",
            "rear_right_first_joint",
            "rear_right_second_joint",
            "rear_right_third_joint",
        ]

        # Initialize and publish the zero trajectory
        # self.publish_zero_trajectory()

    def publish_zero_trajectory(self):
        # Create a JointTrajectory message
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names

        # Create a JointTrajectoryPoint with all positions set to 0.0
        point = JointTrajectoryPoint()
        point.positions = [0.0] * len(
            self.joint_names
        )  # Set position of each joint to 0.0
        point.time_from_start.sec = (
            0  # Set a 1-second time from start for demonstration
        )
        point.time_from_start.nanosec = 500000000
        # Add the point to the trajectory message
        trajectory_msg.points.append(point)

        # Publish the message
        self.publisher_.publish(trajectory_msg)
        self.get_logger().info("Published zero trajectory to all joints.")

        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    zero_trajectory_publisher = ZeroTrajectoryPublisher()
    zero_trajectory_publisher.publish_zero_trajectory()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
