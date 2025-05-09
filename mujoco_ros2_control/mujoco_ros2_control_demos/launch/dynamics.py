import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import pinocchio as pin
import numpy as np
from four_bar_kinematics import analyze_fourbar

MAX_TORQUE = 1.0


class ImpedanceControllerNode(Node):
    def __init__(self):
        super().__init__("impedance_controller_node")

        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 10
        )

        self.effort_pub = self.create_publisher(
            Float64MultiArray, "/effort_controllers/commands", 10
        )

        self.desired_pos_pub = self.create_publisher(
            Float64MultiArray, "/desired_positions", 10
        )

        self.passive_state_pub = self.create_publisher(
            Float64MultiArray, "/passive_joints/state", 10
        )

        self.desired_positions = [0.0, 0.0, 3.0]
        self.get_logger().info(
            f"Desired positions initialized to: {self.desired_positions}"
        )

        self.model = pin.buildModelFromUrdf(
            "/home/rabin/Documents/quadruped_robot/src/mujoco_ros2_control/mujoco_ros2_control_demos/urdf/quadruped_to_mujoco.urdf"
        )
        self.data = self.model.createData()

        for i, name in enumerate(self.model.names):
            self.get_logger().info(f"Joint name: {name}")
        for frame in self.model.frames:
            self.get_logger().info(f"Frame name: {frame.name}")

        self.joint_name_to_index = {
            name: idx - 1 for idx, name in enumerate(self.model.names) if idx > 0
        }

        print(self.joint_name_to_index)

        self.q = np.zeros(self.model.nq)
        self.v = np.zeros(self.model.nv)
        self.tau = np.zeros(self.model.nv)

        self.epsilon = 1.0

        self.i = 0
        self.exit = False

    def joint_state_callback(self, msg):
        if self.exit:
            self.get_logger().info("Exiting...")
            return
        # Zakładamy, że msg.position i msg.velocity są długości 12
        if len(msg.position) == 8:
            return
        if len(msg.position) < 3 or len(msg.velocity) < 3:
            self.get_logger().error("JointState does not contain enough data!")
            return

        self.current_positions = list(msg.position[:3])
        self.current_velocities = list(msg.velocity[:3])

        for name, position, velocity, effort in zip(
            msg.name, msg.position, msg.velocity, msg.effort
        ):
            if name in self.joint_name_to_index:
                idx = self.model.getJointId(name)
                q_idx = self.model.joints[idx].idx_q
                v_idx = self.model.joints[idx].idx_v

                self.q[q_idx] = position
                self.v[v_idx] = velocity
                self.tau[v_idx] = effort

        theta2 = 2 * np.pi - self.q[self.joint_name_to_index["front_left_third_joint"]]
        omega_2 = -self.v[self.joint_name_to_index["front_left_third_joint"]]
        epsilon_2 = -self.tau[self.joint_name_to_index["front_left_third_joint"]]
        kinematic_values = analyze_fourbar(
            theta2,
            omega_2,
            epsilon_2,
            0.125,
            0.125,
            0.21,
            0.21,
        )

        self.q[self.joint_name_to_index["front_left_fifth_joint"]] = (
            theta2 - kinematic_values["theta3"] - np.pi
        )

        self.v[self.joint_name_to_index["front_left_fifth_joint"]] = (
            omega_2 - kinematic_values["omega3"]
        )
        self.tau[self.joint_name_to_index["front_left_fifth_joint"]] = (
            epsilon_2 - kinematic_values["epsilon3"]
        )

        self.q[self.joint_name_to_index["front_left_fourth_joint"]] = (
            2 * np.pi
            - theta2
            - kinematic_values["gamma"]
            + self.q[self.joint_name_to_index["front_left_fifth_joint"]]
        )
        self.v[self.joint_name_to_index["front_left_fourth_joint"]] = 0.0
        self.tau[self.joint_name_to_index["front_left_fourth_joint"]] = 0.0

        passive_state_msg = Float64MultiArray()
        passive_state_msg.data = [
            self.q[self.joint_name_to_index["front_left_fifth_joint"]],
            self.v[self.joint_name_to_index["front_left_fifth_joint"]],
            self.tau[self.joint_name_to_index["front_left_fifth_joint"]],
            self.q[self.joint_name_to_index["front_left_fourth_joint"]],
        ]
        self.passive_state_pub.publish(passive_state_msg)

        # Możesz np. od razu policzyć h(q, v)
        # h = pin.nonLinearEffects(self.model, self.data, self.q, self.v)

        # Policz GRFy dla każdej nogi
        # grfs = self.compute_grfs(h)

        t = self.i / 500
        a = np.pi / 2
        q_des = [np.sin(t) * a - a / 2] * 3
        dq_des = [np.cos(t) * a] * 3
        ddq_des = [-np.sin(t) * a] * 3
        q_des[1] *= 0.5
        dq_des[1] *= 0.5
        ddq_des[1] *= 0.5
        q_des[1] += np.pi

        # q_des[2] *= 0.1
        # q_des[2] += -5.2

        vd_des = np.zeros(self.model.nv)
        kp = [1000.0, 1000.0, 1000.0]
        # kp = [1000.0, 1000.0, 1000.0] / 1000
        kd = [100.0, 100.0, 100.0]

        joint_names = [
            "front_left_first_joint",
            "front_left_second_joint",
            "front_left_fifth_joint",
        ]
        q_motors = [self.q[self.joint_name_to_index[name]] for name in joint_names]
        dq_motors = [self.v[self.joint_name_to_index[name]] for name in joint_names]

        q_motors = np.array(q_motors)
        dq_motors = np.array(dq_motors)

        ddq_des[0] = 0.0
        dq_des[0] = 0.0
        q_des[0] = np.pi / 2
        ddq_des[1] = 0.0
        dq_des[1] = 0.0
        q_des[1] = np.pi - 0.5

        ddq_des[2] = 0.0
        dq_des[2] = 0.0
        q_des[2] = 1.0
        vd_des = ddq_des + kp * (q_des - q_motors) + kd * (dq_des - dq_motors)

        # vd_des[2] = -np.pi / 2

        vd_des = np.concatenate((vd_des, np.zeros(self.model.nv - len(vd_des))), axis=0)
        # vd_des[3] = np.pi / 2
        tau = pin.rnea(self.model, self.data, self.q, self.v, vd_des)
        new_theta2 = self.q[self.joint_name_to_index["front_left_fourth_joint"]]
        new_kinematic_values = analyze_fourbar(
            new_theta2,
            (np.random.rand() * 0.1) - 0.05,
            (np.random.rand() * 0.1) - 0.05,

            0.125,
            0.125,
            0.21,
            0.21,
        )
        
        tau[2] = (
             -new_kinematic_values["epsilon3"]
        )


        if np.nan in tau:
            self.get_logger().warn("Naaaan")
            return

        torque_msg = Float64MultiArray()
        torque_msg.data = [
            tau[0],
            # 0.0,
            tau[1],
            #    -0.1
            # tau[2],
            np.sin(50*t) * 0.8,
            # 0.0,
        ]
        for i in range(torque_msg.data.__len__()):
            torque_msg.data[i] = np.clip(torque_msg.data[i], -MAX_TORQUE, MAX_TORQUE)

        self.effort_pub.publish(torque_msg)
        desired_pos_msg = Float64MultiArray()
        desired_pos_msg.data = [q_des[0], q_des[1], q_des[2]]
        self.desired_pos_pub.publish(desired_pos_msg)

        self.i += 1
        if self.i > np.pi * 1000:
            self.i = 0

        self.get_logger().info(f"Torque: {torque_msg.data}")
        # self.shutdown()

    def send_zero_torque(self):
        self.exit = True
        torque_msg = Float64MultiArray()
        torque_msg.data = [0.0, 0.0, 0.0]
        self.effort_pub.publish(torque_msg)


    # def compute_grfs(self, h):
    #     grfs = {}
    #     for foot_name in ["front_left_foot_link"]:
    #         frame_id = self.model.getFrameId(foot_name)

    #         # Oblicz Jacobian stopy
    #         J = pin.computeFrameJacobian(
    #             self.model,
    #             self.data,
    #             self.q,
    #             frame_id,
    #             pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
    #         )
    #         J_leg = J[:3, :3]

    #         joints_names = [
    #             "front_left_first_joint",
    #             "front_left_second_joint",
    #             "front_left_fifth_joint",
    #         ]
    #         indexes = [
    #             indx
    #             for name, indx in self.joint_name_to_index.items()
    #             if name in joints_names
    #         ]
    #         tau_leg = np.zeros(3)
    #         for i in range(len(indexes)):
    #             tau_leg[i] = self.tau[indexes[i]]
    #         h_leg = h[:3]

    #         try:
    #             JT_inv = np.linalg.inv(J_leg[:, : self.model.nv].T)
    #             F_leg = -JT_inv @ (tau_leg - h_leg)
    #         except np.linalg.LinAlgError:
    #             self.get_logger().warn(
    #                 f"Jacobian for {foot_name} is singular, cannot invert."
    #             )
    #             F_leg = np.zeros(3)

    #         # Liczenie alpha'
    #         norm_F_leg = np.linalg.norm(F_leg)
    #         alpha = 1.0 if norm_F_leg > self.epsilon else 0.0

    #         # Finalne GRF
    #         grfs[foot_name] = alpha * F_leg

    #     return grfs


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
