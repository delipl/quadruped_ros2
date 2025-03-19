#!/usr/bin/env python3
import rclpy
import numpy as np
import math
from rclpy.node import Node
from copy import copy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped


def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci * ck
    cs = ci * sk
    sc = si * ck
    ss = si * sk

    q = np.empty((4,))
    q[0] = cj * sc - sj * cs
    q[1] = cj * ss + sj * cc
    q[2] = cj * cs - sj * sc
    q[3] = cj * cc + sj * ss

    return q


class QuadrupedJoy(Node):

    def __init__(self):
        super().__init__("quadruped_joy")

        self.joy_sub_ = self.create_subscription(Joy, "joy", self.joy_callback, 10)
        self.joy_pub_ = self.create_publisher(Joy, "joy_cmd_vel", 10)
        self.pose_cmd_pub_ = self.create_publisher(
            PoseStamped, "quadruped_robot/command_base_pose" , 10
        )

        self.button_index_map = {
            "axis": {
                "position_x": 1,
                "position_y": 0,
                "position_z": 2,
                "orientation_r": 0,
                "orientation_p": 1,
                "orientation_y": 3,
            },
        }

        self.movement_type_button_map = {
            "XYZ_YAW": 5,
            "RPY_Z": 4,
            "WALK": 6,
        }

        self.factor_map = {
            "position_factor": {
                "x": -0.1,
                "y": 0.1,
                "z": -0.1,
            },
            "orientation_factor": {
                "r": -0.3,
                "p": -0.3,
                "y": 0.3,
            },
        }

        self.get_logger().info("Hello world from the Python node quadruped_joy")
        self.was_walk_before = False

    def joy_callback(self, msg):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "base_footprint"


        if msg.buttons[self.movement_type_button_map["WALK"]]:
            self.joy_pub_.publish(msg)
            self.was_walk_before = True
            return
        elif msg.buttons[self.movement_type_button_map["XYZ_YAW"]]:
            pose.pose.position.x = (
                msg.axes[self.button_index_map["axis"]["position_x"]]
                * self.factor_map["position_factor"]["x"]
            )
            pose.pose.position.y = (
                msg.axes[self.button_index_map["axis"]["position_y"]]
                * self.factor_map["position_factor"]["y"]
            )
            pose.pose.position.z = (
                msg.axes[self.button_index_map["axis"]["position_z"]]
                * self.factor_map["position_factor"]["z"]
            )
            
            yaw = (
                msg.axes[self.button_index_map["axis"]["orientation_y"]]
                * self.factor_map["orientation_factor"]["y"]
            )
            
            q = quaternion_from_euler(0, 0, yaw)
            
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]

        elif msg.buttons[self.movement_type_button_map["RPY_Z"]]:
            r = (
                msg.axes[self.button_index_map["axis"]["orientation_r"]]
                * self.factor_map["orientation_factor"]["r"]
            )
            p = (
                msg.axes[self.button_index_map["axis"]["orientation_p"]]
                * self.factor_map["orientation_factor"]["p"]
            )
            y = (
                msg.axes[self.button_index_map["axis"]["orientation_y"]]
                * self.factor_map["orientation_factor"]["y"]
            )

            q = quaternion_from_euler(r, p, y)

            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]

            pose.pose.position.z = (
                msg.axes[self.button_index_map["axis"]["position_z"]]
                * self.factor_map["position_factor"]["z"]
            )


        if self.was_walk_before:
            self.was_walk_before = False

            zero_msg = copy(msg)
            zero_msg.axes[0] = 0.0
            zero_msg.axes[1] = 0.0
            zero_msg.axes[2] = 0.0
            zero_msg.axes[3] = 0.0
            self.joy_pub_.publish(msg)
            self.get_logger().info("publishing zero joy")
        else:
            self.pose_cmd_pub_.publish(pose)
            # self.get_logger().info("publishing pose command")


def main(args=None):
    rclpy.init(args=args)

    quadruped_joy = QuadrupedJoy()

    try:
        rclpy.spin(quadruped_joy)
    except KeyboardInterrupt:
        pass

    quadruped_joy.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()