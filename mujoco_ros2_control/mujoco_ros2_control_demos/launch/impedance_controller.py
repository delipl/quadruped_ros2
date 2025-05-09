import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np


class ImpedanceControllerNode(Node):
    def __init__(self):
        super().__init__("impedance_controller_node")

        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 10
        )

        self.effort_pub = self.create_publisher(
            Float64MultiArray, "/effort_controllers/commands", 10
        )

        self.position_error_pub = self.create_publisher(
            Float64MultiArray, "/impedance_controller/position_error", 10
        )

        self.velocity_error_pub = self.create_publisher(
            Float64MultiArray, "/impedance_controller/velocity_error", 10
        )

        self.position_sub = self.create_subscription(
            Float64MultiArray,
            "/position_controller/commands",
            self.position_callback,
            10,
        )

        self.kp = [30.0, 30.5, 30.5] * 4
        self.kd = [1.2, 1.2, 1.2] * 4
        self.max_torque = [40.0, 40.0, 40.0] * 4

        self.get_logger().info(
            f"Desired positions initialized to: {self.kp}, {self.kd}, {self.max_torque}"
        )
        self.target_position = None
        self.position_error = None
        self.velocity_error = None

    def position_callback(self, msg):
        if self.target_position is None:
            return

        if len(msg.data) != len(self.target_position):
            self.get_logger().error(
                f"Received {len(msg.data)} positions, expected {len(self.target_position)}"
            )
            return

        for i in range(len(msg.data)):
            self.target_position[i] = msg.data[i]

    def joint_state_callback(self, msg):
        if len(msg.position) != 12:
            return
        if self.target_position is None:

            self.target_position = [0.0] * len(msg.position)
            self.position_error = [0.0] * len(msg.position)
            self.velocity_error = [0.0] * len(msg.position)

            for i in range(len(msg.position)):
                self.target_position[i] = msg.position[i]

        torque = Float64MultiArray()
        position_error = Float64MultiArray()
        velocity_error = Float64MultiArray()
        torque.data = [0.0] * len(self.target_position)
        position_error.data = [0.0] * len(self.target_position)
        velocity_error.data = [0.0] * len(self.target_position)

        joint_order = [
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

        joint_indices = {name: i for i, name in enumerate(msg.name)}

        ordered_positions = [0.0] * len(joint_order)
        ordered_velocities = [0.0] * len(joint_order)

        for i, joint_name in enumerate(joint_order):
            if joint_name in joint_indices:
                idx = joint_indices[joint_name]
                ordered_positions[i] = msg.position[idx]
                ordered_velocities[i] = msg.velocity[idx]
            else:
                self.get_logger().warn(f"Joint {joint_name} not found in joint states.")

        msg.position = ordered_positions
        msg.velocity = ordered_velocities

        for i in range(len(self.target_position)):
            if self.target_position is not None:
                e_q = self.target_position[i] - msg.position[i]
                e_v = 0.0 - msg.velocity[i]

                position_error.data[i] = e_q
                velocity_error.data[i] = e_v

                torque.data[i] = self.kp[i] * e_q + self.kd[i] * e_v
                torque.data[i] = np.clip(
                    torque.data[i], -self.max_torque[i], self.max_torque[i]
                )

        self.effort_pub.publish(torque)
        self.position_error_pub.publish(position_error)
        self.velocity_error_pub.publish(velocity_error)

    def send_zero_torque(self):
        self.exit = True
        torque_msg = Float64MultiArray()
        torque_msg.data = [0.0, 0.0, 0.0]
        self.effort_pub.publish(torque_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImpedanceControllerNode()
    try:
        node.get_logger().info("Impedance Controller Node started.")
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Impedance Controller Node stopped.")
        node.send_zero_torque()
        rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
