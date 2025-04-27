import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState


class ImpedanceControllerNode(Node):
    def __init__(self):
        super().__init__("impedance_controller_node")

        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 10
        )

        self.effort_pub = self.create_publisher(
            Float64MultiArray, "/effort_controllers/commands", 10
        )
        
        self.desired_positions = [0.0, 0.0, 3.0] 
        self.get_logger().info(
            f"Desired positions initialized to: {self.desired_positions}"
        )
        
        self.current_positions = [0.0] * 3
        self.current_velocities = [0.0] * 3
        # self.kp = [25.0, 7.5, 7.5]   # Proportional gains
        # self.kd = [5.0, 1.0, 1.0]   # Derivative gains
        
        self.kp = [1.9, 1.5, 1.5]   # Proportional gains
        self.kd = [0.15, 0.1, 0.1]   # Derivative gains

        self.timer = self.create_timer(0.1, self.compute_and_publish_effort)  # 100 Hz

    def joint_state_callback(self, msg):
        # Zakładamy, że msg.position i msg.velocity są długości 12
        if len(msg.position) == 8:
            return
        if len(msg.position) < 3 or len(msg.velocity) < 3:
            self.get_logger().error("JointState does not contain enough data!")
            return

        self.current_positions = list(msg.position[:3])
        self.current_velocities = list(msg.velocity[:3])

    def compute_and_publish_effort(self):
        effort_commands = []
        for i in range(3):
            pos_error = self.desired_positions[i] - self.current_positions[i]
            vel_error = -self.current_velocities[i]
            self.get_logger().info(
                f"Joint {i}: pos_error={pos_error}, vel_error={vel_error}"
            )
            effort = self.kp[i] * pos_error + self.kd[i] * vel_error
            effort_commands.append(effort)


        effort_msg = Float64MultiArray()
        effort_msg.data = effort_commands
        self.effort_pub.publish(effort_msg)
        
        self.get_logger().info(
            f"Published effort commands: {effort_commands}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ImpedanceControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
