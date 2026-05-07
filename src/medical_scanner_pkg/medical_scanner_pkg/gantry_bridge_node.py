#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from sensor_msgs.msg import JointState


class GantryBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__('gantry_bridge_node')

        self.declare_parameter('input_topic', '/current_position')
        self.declare_parameter('joint_name', 'gantry_joint')

        self.input_topic = self.get_parameter('input_topic').value
        self.joint_name = self.get_parameter('joint_name').value

        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.position_sub = self.create_subscription(
            Float32,
            self.input_topic,
            self.position_callback,
            10,
        )

        self.get_logger().info(
            f'Bridging {self.input_topic} (mm) -> /joint_states (m) for joint "{self.joint_name}"'
        )

    def position_callback(self, msg: Float32) -> None:
        position_m = float(msg.data) / 1000.0  # mm -> m

        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = [self.joint_name]
        joint_state.position = [position_m]

        self.joint_pub.publish(joint_state)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GantryBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()