#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped


class QuadrupedJoy(Node):

    def __init__(self):
        super().__init__("quadruped_joy")

        self.joy_sub_ = self.create_subscription(Joy, "joy", self.joy_callback, 10)
        self.pose_cmd_pub_ = self.create_publisher(
            PoseStamped, "quadruped/command_base_pose", 10
        )

        self.button_index_map = {
            "axis": {
                "position_x": 0,
                "position_y": 1,
                "position_z": 3,
                "orientation_r": 4,
                "orientation_p": 6,
                "orientation_y": 7,
            },
        }

        self.factor_map = {
            "position_factor": {
                "x": 0.1,
                "y": 0.1,
                "z": 0.1,
            },
            "orientation_factor": {
                "r": 0.1,
                "p": 0.1,
                "y": 0.1,
            },
        }

        for button_name, button_index in self.button_index_map["axis"].items():
            self.button_index_map["axis"][button_name] = self.declare_parameter(
                f"button_index_map.axis.{button_name}", button_index
            ).value

        for factor_name, factor_value in self.factor_map["position_factor"].items():
            self.factor_map["position_factor"][factor_name] = self.declare_parameter(
                f"factor_map.position_factor.{factor_name}", factor_value
            ).value

        for factor_name, factor_value in self.factor_map["orientation_factor"].items():
            self.factor_map["orientation_factor"][factor_name] = self.declare_parameter(
                f"factor_map.orientation_factor.{factor_name}", factor_value
            ).value

        example_param = self.declare_parameter("example_param", "default_value").value

        self.get_logger().info(
            f"Declared parameter 'example_param'. Value: {example_param}"
        )
        self.get_logger().info("Hello world from the Python node quadruped_joy")

    def joy_callback(self, msg):
        pose = PoseStamped()
        
        pose.pose.position.x = msg.axes[self.button_index_map["axis"]["position_x"]] * self.factor_map["position_factor"]["x"]
        pose.pose.position.y = msg.axes[self.button_index_map["axis"]["position_y"]] * self.factor_map["position_factor"]["y"]
        pose.pose.position.z = msg.axes[self.button_index_map["axis"]["position_z"]] * self.factor_map["position_factor"]["z"]
        


        self.get_logger().info(f"Received Joy message: {msg}")


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
