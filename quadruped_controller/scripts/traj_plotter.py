import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Float64MultiArray


class Joint3Listener(Node):
    def __init__(self):
        super().__init__("joint_3_listener")
        # Subskrybuje wiadomości JointTrajectory
        self.subscription = self.create_subscription(
            JointTrajectory, "/joint_trajectory_controller/joint_trajectory", self.listener_callback, 10
        )
        self.subscription  # zapobiega usunięciu zmiennej przez GC

        # Publikuje pozycję joint 3
        self.publisher_ = self.create_publisher(
            Float64MultiArray, "/control_positions", 10
        )
        self.control_positions = [[], [], [], [], [], [], [], [], [], [], [], []]
        self.timer = self.create_timer(0.008, self.timer_callback)

    def listener_callback(self, msg):

        for point in msg.points:
            for joints_position, i in zip(point.positions, range(len(point.positions))):
                # print(joints_position)
                self.control_positions[i].append(joints_position)

    def timer_callback(self):
        if len(self.control_positions[0]) == 0:
            self.get_logger().warn("No data")
            return
        self.get_logger().info("Timer callback")
        msg = Float64MultiArray()
        msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        for i, joint_positions in enumerate(self.control_positions):
            msg.data[i] = joint_positions[0]
            joint_positions.remove(joint_positions[0])

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    joint_3_listener = Joint3Listener()
    rclpy.spin(joint_3_listener)
    joint_3_listener.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
