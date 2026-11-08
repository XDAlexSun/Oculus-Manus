#!/usr/bin/env python3
"""
Publishes a synthetic hand on /hand/world_landmarks so teleop + simulation can be tested
with NO camera. Fingers slowly open and close, each with a small phase offset.

Geometry: first bone (wrist -> MCP) is straight along +y; each following bone is bent by
`theta` more than the one before, in the y-z plane. The teleop node therefore measures
theta at every joint, and its summed curl is 3*theta. Sweeping theta over 0.1 .. 1.25 rad
gives curl 0.3 .. 3.75, matching the default calibration (curl_open 0.3, curl_closed 3.8),
so no calibration is needed for this test.
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

CHAINS = [(0, 1, 2, 3, 4), (0, 5, 6, 7, 8), (0, 9, 10, 11, 12),
          (0, 13, 14, 15, 16), (0, 17, 18, 19, 20)]
BONE = 0.03  # metres


def make_landmarks(thetas):
    """thetas: 5 values (rad), one per finger. Returns 63 floats (21 landmarks x 3)."""
    pts = [[0.0, 0.0, 0.0] for _ in range(21)]
    for finger, chain in enumerate(CHAINS):
        pts[chain[1]] = [0.0, BONE, 0.0]
        y, z, heading = BONE, 0.0, 0.0
        for k in range(2, 5):
            heading += thetas[finger]
            y += BONE * math.cos(heading)
            z += BONE * math.sin(heading)
            pts[chain[k]] = [0.0, y, z]
    return [c for p in pts for c in p]


class FakeHandNode(Node):
    def __init__(self):
        super().__init__('fake_hand_node')
        self.declare_parameter('period', 6.0)   # seconds per open-close cycle
        self.period = self.get_parameter('period').value
        self.pub = self.create_publisher(Float64MultiArray, '/hand/world_landmarks', 10)
        self.t0 = self.get_clock().now()
        self.timer = self.create_timer(1.0 / 30.0, self.tick)
        self.get_logger().info('Fake hand started (synthetic open/close sweep)')

    def tick(self):
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        thetas = []
        for f in range(5):
            phase = 2.0 * math.pi * (t / self.period - 0.08 * f)
            s = 0.5 * (1.0 - math.cos(phase))          # 0..1
            thetas.append(0.1 + s * (1.25 - 0.1))
        msg = Float64MultiArray()
        msg.data = make_landmarks(thetas)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeHandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
