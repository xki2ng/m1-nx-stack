#!/usr/bin/env python3
"""Relay Nav2 costmap from TRANSIENT_LOCAL to VOLATILE durability for Zenoh cross-machine.

Problem: Nav2 costmap publishes with TRANSIENT_LOCAL durability (hardcoded).
rmw_zenoh_cpp doesn't deliver TRANSIENT_LOCAL messages to remote VOLATILE subscribers.
Remote RViz shows "no map received, some messages were lost".
Point clouds work because they use BEST_EFFORT.

Fix: Subscribe locally (NX) with TRANSIENT_LOCAL (match publisher),
republish with VOLATILE (Zenoh-cross-machine compatible).

CRITICAL: Subscriber MUST use TRANSIENT_LOCAL. VOLATILE subs can't receive from
TRANSIENT_LOCAL publishers even on the same machine with rmw_zenoh_cpp.

Usage:
  # Stop ros2 daemon first (rmw mismatch crashes subscriber nodes):
  ros2 daemon stop
  sleep 1
  # Run:
  python3 costmap_zenoh_relay.py
  # Then point RViz to /local_costmap/costmap_v and /global_costmap/costmap_v
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid


class CostmapRelay(Node):
    def __init__(self):
        super().__init__("costmap_relay")

        # Match publisher exactly: RELIABLE + TRANSIENT_LOCAL + depth=1
        sub_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # Republish: RELIABLE + VOLATILE (Zenoh-cross-machine safe)
        pub_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.local_sub = self.create_subscription(
            OccupancyGrid, "/local_costmap/costmap", self.local_cb, sub_qos)
        self.global_sub = self.create_subscription(
            OccupancyGrid, "/global_costmap/costmap", self.global_cb, sub_qos)
        self.local_pub = self.create_publisher(
            OccupancyGrid, "/local_costmap/costmap_v", pub_qos)
        self.global_pub = self.create_publisher(
            OccupancyGrid, "/global_costmap/costmap_v", pub_qos)

        self.lcount = 0
        self.gcount = 0
        self.get_logger().info(
            "Costmap relay started (TRANSIENT_LOCAL sub -> VOLATILE pub)")

    def local_cb(self, msg):
        self.local_pub.publish(msg)
        self.lcount += 1
        if self.lcount <= 3 or self.lcount % 50 == 0:
            self.get_logger().info(f"Local costmap #{self.lcount} relayed")

    def global_cb(self, msg):
        self.global_pub.publish(msg)
        self.gcount += 1
        if self.gcount <= 3 or self.gcount % 10 == 0:
            self.get_logger().info(f"Global costmap #{self.gcount} relayed")


def main():
    rclpy.init()
    node = CostmapRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
