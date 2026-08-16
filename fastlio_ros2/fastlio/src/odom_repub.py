#!/usr/bin/env python3
"""
odom_repub.py — /Odometry → /odom 话题重发 (帧名不变)

FAST-LIO 发布 /Odometry (frame_id=odom, child=baselink);
本节点转发为小写 /odom, 兼容下游导航栈惯例。

用法:
  ros2 run fast_lio odom_repub.py
参数:
  odom_in_topic   默认 /Odometry
  odom_out_topic  默认 /odom
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class OdomRepub(Node):
    def __init__(self):
        super().__init__('odom_repub')
        self.declare_parameter('odom_in_topic', '/Odometry')
        self.declare_parameter('odom_out_topic', '/odom')
        in_topic = self.get_parameter('odom_in_topic').value
        out_topic = self.get_parameter('odom_out_topic').value
        self._sub = self.create_subscription(Odometry, in_topic, self._cb, 10)
        self._pub = self.create_publisher(Odometry, out_topic, 10)
        self.get_logger().info(f"republishing {in_topic} -> {out_topic}")

    def _cb(self, msg: Odometry):
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = OdomRepub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
