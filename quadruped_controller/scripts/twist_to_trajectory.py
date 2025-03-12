#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from quadruped_msgs.msg import QuadrupedControl
from std_msgs.msg import  Bool
from geometry_msgs.msg import Twist

import numpy as np



class PosePublisher(Node):
    def __init__(self):
        super().__init__("pose_publisher")

        # Utworzenie publishera, który publikuje wiadomości PoseStamped na topiku 'pose'
        self.publisher_ = self.create_publisher(
            QuadrupedControl, "/control_quadruped", 10
        )

        self.subscription = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callback, 10
        )

        # Częstotliwość publikacji co sekundę
        timer_period = 0.01
        # timer_period = 0.1
        self.i = 0
        self.trajs_x = []
        self.trajs_y = []
        self.trajs_z = []
        self.trajs_contact = []

        ticks = 30
        self.generate_trot(ticks, 0.0, 0.0, 0.0, 0.0)

        self.timer = self.create_timer(timer_period, self.publish_pose)

    def cmd_vel_callback(self, msg):
        if msg.linear.x == 0 and msg.linear.y == 0 and msg.angular.z == 0:
            ticks = 30
            dx = 0.0
            dy = 0.0
            dw = 0.0
            self.generate_trot(ticks, dx, dy, dw, 0.0)
            return

        ticks = 30
        dx = msg.linear.x / 5.0
        dy = msg.linear.y / 5.0
        dw = msg.angular.z / 5.0
        self.generate_trot(ticks, dx, dy, dw, 0.185)

    def generate_trot(self, ticks, dx, dy, dw, dz):
        ises = [0, 1, 1, 0]
        dxes = dx * np.array([1, 1, 1, 1])
        dyes = dy * np.array([1, 1, 1, 1])
        if dw != 0:
            self.get_logger().info(f"dw: {dw}")
            dyes = dw * np.array([1, 1, -1, -1])

        x0es = 0.227 * np.array([1, 1, -1, -1])
        y0es = 0.15 * np.array([1, -1, 1, -1])
        self.trajs_x = []
        self.trajs_y = []
        self.trajs_z = []
        self.trajs_contact = []
        self.get_logger().info(f"x0: {x0es}, y0: {y0es}")
        self.get_logger().info(f"dx: {dxes}, dy: {dyes}")
        index = 0
        for i in ises:

            traj_x, traj_y, traj_z, traj_contact = self.generate_leg_trajectory(
                ticks, i, 1, x0es[index], y0es[index], dxes[index], dyes[index], dz
            )
            self.trajs_x.append(traj_x)
            self.trajs_y.append(traj_y)
            self.trajs_z.append(traj_z)
            self.trajs_contact.append(traj_contact)
            index += 1

    def generate_leg_trajectory(
        self, ticks, phase=0, periods=3, x0=0.3, y0=0.148, dx=0.0, dy=0.0, dz=0.15
    ):
        z0 = -0.23
        x = np.linspace(x0, x0 + dx, ticks)
        xb = np.linspace(x0 + dx, x0, periods * ticks)
        y = np.linspace(y0, y0 + dy, ticks)
        yb = np.linspace(y0 + dy, y0, periods * ticks)

        s = np.linspace(0, np.pi, ticks)
        sb = np.linspace(np.pi, 0, periods * ticks)

        z = z0 + dz * np.sin(s)
        if dz == 0:
            dminz = 0.0
        else:
            dminz = 0.01 

        x = np.concatenate((x, xb)) - dx / 2
        y = np.concatenate((y, yb)) - dy / 2
        z = np.concatenate((z, z0 -dminz * np.sin(sb)))

        cut = ticks * phase
        traj_x = np.hstack((x[cut:], x[:cut]))
        traj_y = np.hstack((y[cut:], y[:cut]))
        traj_z = np.hstack((z[cut:], z[:cut]))

        traj_contact = np.where(traj_z <= z0, True, False)
        return traj_x, traj_y, traj_z, traj_contact

    def publish_pose(self):
        # Tworzenie wiadomości PoseStamped
        msg = QuadrupedControl()

        # Ustawienie nagłówka z czasem i identyfikatorem ramki
        msg.header.stamp = self.get_clock().now().to_msg()

        fl = Bool()
        fr = Bool()
        rl = Bool()
        rr = Bool()

        fl.data = bool(self.trajs_contact[0][self.i])
        fr.data = bool(self.trajs_contact[1][self.i])
        rl.data = bool(self.trajs_contact[2][self.i])
        rr.data = bool(self.trajs_contact[3][self.i])

        msg.fl_foot_position.x = self.trajs_x[0][self.i]
        msg.fl_foot_position.y = self.trajs_y[0][self.i]
        msg.fl_foot_position.z = self.trajs_z[0][self.i]
        msg.fl_foot_in_contact = fl

        msg.fr_foot_position.x = self.trajs_x[1][self.i]
        msg.fr_foot_position.y = self.trajs_y[1][self.i]
        msg.fr_foot_position.z = self.trajs_z[1][self.i]
        msg.fr_foot_in_contact = fr

        msg.rl_foot_position.x = self.trajs_x[2][self.i]
        msg.rl_foot_position.y = self.trajs_y[2][self.i]
        msg.rl_foot_position.z = self.trajs_z[2][self.i]
        msg.rl_foot_in_contact = rl

        msg.rr_foot_position.x = self.trajs_x[3][self.i]
        msg.rr_foot_position.y = self.trajs_y[3][self.i]
        msg.rr_foot_position.z = self.trajs_z[3][self.i]
        msg.rr_foot_in_contact = rr

        if self.i == len(self.trajs_x[0]) - 1:
            self.i = 0
        else:
            self.i += 1

        self.publisher_.publish(msg)


def main(args=None):

    try:
        rclpy.init(args=args)

        pose_publisher = PosePublisher()

        rclpy.spin(pose_publisher)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
