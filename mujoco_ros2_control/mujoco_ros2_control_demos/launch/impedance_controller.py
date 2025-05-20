import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np
from std_srvs.srv import Empty
from example_interfaces.srv import AddTwoInts
import time
import copy


###
# Third PD:
t_skala = 0.1 / 26
T0 = 3896.898 - 3896.807
T = 3897.393 - T0 - 3896.807
K = 0.072

print(f"T0: {T0}, T: {T}, K: {K}")
# K = 0.01
tau = T0 / (T0 + T)
a = K * T0 / T

Kp = 1.24 / a * (1 + (0.13 * tau) / (1 - tau))

Td = (0.27 - 0.36 * tau) / (1 - 0.87 * tau) * T0
print(f"{Kp}, {Td}")

# 3:
# 34.20749226006192, 0.03655939621794512

# 2:
# 51.207090397090106 0.023184721126433695


###


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

        self.create_service(
            Empty, "/impedance_controller/stand_up", self.add_on_set_parameters_callback
        )

        self.create_service(
            AddTwoInts, "/impedance_controller/first_step", self.step_callback
        )

        self.ready_kp = [30.0, 10.5, 20.5] * 4
        self.ready_kd = [1.0, 1.0, 1.0] * 4
        self.ready_max_torque = [20.0, 5.0, 5.0] * 4

        # welded
        # self.ready_kp = [10.0, 10.5, 10.5] * 4
        # self.ready_kd = [1.0, 1.0, 1.0] * 4
        # self.ready_max_torque = [20.0, 20.0, 20.0] * 4

        self.stand_up_kp = [70.0, 100.5, 100.5] * 4
        self.stand_up_kd = [1.0, 1.8, 1.8] * 4
        self.stand_up_max_torque = [50.0, 30.0, 30.0] * 4

        self.stand_up_kp = [100.0, 20.5, 20.5] * 4
        self.stand_up_kd = [1.8, 1.0, 1.0] * 4
        self.stand_up_max_torque = [30.0, 5.0, 5.0] * 4

        # welded
        # self.stand_up_kp = [10.0, 10.5, 10.5] * 4
        # self.stand_up_kd = [1.0, 1.0, 1.0] * 4
        # self.stand_up_max_torque = [40.0, 20.0, 20.0] * 4

        # self.walk_kp = [100.0, 30.5, 30.5] * 4
        # self.walk_kd = [1.4, 1.0, 1.0] * 4
        # self.walk_max_torque = [40.0, 10.0, 25.0] * 4
        # ,, 0.025412436548223352
        self.walk_kp = [95.92020756982738, 51.207090397090106, 34.20749226006192] * 4
        self.walk_kd = [
            95.92020756982738 * 0.022526014640095713 ,
            51.207090397090106 * 0.023184721126433695,
            # 51.207090397090106 * 0.023184721126433695,
            34.20749226006192 * 0.03655939621794512,
        ] * 4
        self.walk_max_torque = [50.0, 30.0, 50.0] * 4

        # welded
        # self.walk_kp = [20.0, 10.0, 10.0] * 4
        # self.walk_kd = [0.6, 0.8, 0.8] * 4
        # self.walk_max_torque = [20.0, 20.0, 20.0] * 4

        self.kp = [100.0, 10.5, 10.5] * 4
        self.kd = [1.0, 1.0, 1.0] * 4
        self.max_torque = [20.0, 20.0, 20.0] * 4

        self.start_positions = [0.0, -1.0, 1.4] * 4
        self.stand_up_positions = [0.0, -1.0, 1.4] * 4

        first_start = 0.8
        self.start_positions[0] = first_start
        self.start_positions[3] = -first_start
        self.start_positions[6] = -first_start
        self.start_positions[9] = first_start

        second_start = -1.0
        self.start_positions[1] = second_start
        self.start_positions[4] = second_start
        self.start_positions[7] = second_start
        self.start_positions[10] = second_start

        self.ready = False
        self.stand_up = False

        self.get_logger().info(
            f"Desired positions initialized to: {self.kp}, {self.kd}, {self.max_torque}"
        )
        self.target_position = None
        self.position_error = None
        self.velocity_error = None

        self.feed_forward_torque = [0] * 12

        self.checking_time = time.time()

    def step_callback(self, request, response):
        for i in range(4):

            index = (i * 3) + request.a
            dir = 1
            if request.a == 0:
                if i in [1, 2]:
                    dir = -1

            self.feed_forward_torque[index] = dir * float(request.b) / 1000.0
            self.kp[index] = 0.0
            self.kd[index] = 0.0

        return response

    def stand_up_callback(self, request, response):
        self.stand_up = False
        self.ready = False
        self.target_position = None
        self.get_logger().info("Stand up sequence reset.")

        return response

    def position_callback(self, msg):
        if self.target_position is None or self.stand_up is False:
            return

        if len(msg.data) != len(self.target_position):
            self.get_logger().error(
                f"Received {len(msg.data)} positions, expected {len(self.target_position)}"
            )
            return

        for i in range(len(msg.data)):
            self.target_position[i] = msg.data[i]

    def order_joint_states(self, msg):
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
        ordered_names = [0.0] * len(joint_order)

        for i, joint_name in enumerate(joint_order):
            if joint_name in joint_indices:
                idx = joint_indices[joint_name]
                ordered_positions[i] = msg.position[idx]
                ordered_velocities[i] = msg.velocity[idx]
                ordered_names[i] = joint_name
            else:
                self.get_logger().warn(f"Joint {joint_name} not found in joint states.")

        msg.position = ordered_positions
        msg.velocity = ordered_velocities
        msg.name = ordered_names
        return msg

    def is_in_position(
        self, positions, target_positions, epsilon=0.1, stabilize_time=0.0
    ):
        for i in range(len(positions)):
            if abs(positions[i] - target_positions[i]) > epsilon:

                return False

        return True

    def ready_sequence(self, msg):
        if not self.ready:
            self.get_logger().info("In ready sequence...")
            self.kp = self.ready_kp
            self.kd = self.ready_kd
            self.max_torque = self.ready_max_torque

        if (
            self.is_in_position(msg.position, self.start_positions, 0.3)
            and not self.ready
            and time.time() - self.checking_time > 1.0
        ):

            self.get_logger().info("Ready to stand up...")
            self.ready = True
            self.target_position = copy.copy(self.stand_up_positions)
        elif self.is_in_position(msg.position, self.start_positions, 0.3):
            self.get_logger().info("Stabilizing...")

        elif not self.ready:
            self.checking_time = time.time()

    def stand_up_sequence(self, msg):
        if not self.stand_up and self.ready:
            self.get_logger().info("In standup sequence...")

            self.kp = self.stand_up_kp
            self.kd = self.stand_up_kd
            self.max_torque = self.stand_up_max_torque

        if (
            self.is_in_position(msg.position, self.stand_up_positions, 0.2)
            and self.ready
            and not self.stand_up
            and time.time() - self.checking_time > 1.0
        ):
            self.get_logger().info("Stood up.")
            self.stand_up = True

            self.kp = self.walk_kp
            self.kd = self.walk_kd
            self.max_torque = self.walk_max_torque

            self.get_logger().info(f"Standing torque: {self.feed_forward_torque}")

        elif (
            self.ready
            and not self.stand_up
            and self.is_in_position(msg.position, self.stand_up_positions, 0.3)
        ):
            self.get_logger().info("Stabilizing...")

        elif self.ready and not self.stand_up:
            self.checking_time = time.time()

    def first_sequence(self, msg):
        if self.target_position is None:

            self.target_position = [0.0] * len(msg.position)
            self.position_error = [0.0] * len(msg.position)
            self.velocity_error = [0.0] * len(msg.position)

            for i in range(len(msg.position)):
                self.target_position[i] = copy.copy(self.start_positions[i])

    def joint_state_callback(self, msg):
        if len(msg.position) != 12:
            return

        msg = self.order_joint_states(msg)

        self.first_sequence(msg)
        self.ready_sequence(msg)
        self.stand_up_sequence(msg)

        torque = Float64MultiArray()
        position_error = Float64MultiArray()
        velocity_error = Float64MultiArray()
        torque.data = [0.0] * len(self.target_position)
        position_error.data = [0.0] * len(self.target_position)
        velocity_error.data = [0.0] * len(self.target_position)

        for i in range(len(self.target_position)):

            e_q = self.target_position[i] - msg.position[i]
            e_v = 0.0 - msg.velocity[i]

            position_error.data[i] = e_q
            velocity_error.data[i] = e_v

            torque.data[i] = (
                self.kp[i] * e_q + self.kd[i] * e_v + self.feed_forward_torque[i]
            )
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
