"""scan_tilt_gate — drop laser scans taken while the legged robot's body is tilted.

Spot is a LEGGED robot: the body (and the camera rigidly mounted on it) pitches and rolls
while walking, so the depth-derived 2D `/scan` is not always taken in a horizontal plane.
Tilted scans sweep the floor/ceiling and get inserted into the slam_toolbox map and the Nav2
costmaps as phantom walls (diagonal streaks smearing the map).

This node sits between the producer and every consumer:

    depthimage_to_laserscan -> /scan_raw -> [scan_tilt_gate] -> /scan -> slam_toolbox + Nav2

It republishes a scan ONLY when the latest body orientation (from `/odom`, published by the
Isaac bridge) has |roll| and |pitch| below `max_tilt_deg`. Consumers keep their usual `/scan`
topic and get a cleaner, level-only stream; while Spot stands still every scan passes.

    ros2 run spot_sar_nav scan_tilt_gate --ros-args -p use_sim_time:=true -p max_tilt_deg:=3.0
"""
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


def roll_pitch(q):
    """Roll and pitch (rad) from a geometry_msgs Quaternion (ZYX convention)."""
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
    pitch = math.asin(sinp)
    return roll, pitch


class ScanTiltGate(Node):
    def __init__(self):
        super().__init__("scan_tilt_gate")
        self.declare_parameter("max_tilt_deg", 3.0)
        self.declare_parameter("scan_in", "/scan_raw")
        self.declare_parameter("scan_out", "/scan")
        self.declare_parameter("odom_topic", "/odom")

        self.max_tilt = math.radians(float(self.get_parameter("max_tilt_deg").value))
        scan_in = str(self.get_parameter("scan_in").value)
        scan_out = str(self.get_parameter("scan_out").value)
        odom_topic = str(self.get_parameter("odom_topic").value)

        self._tilt = None          # latest (roll, pitch); None until the first /odom
        self._passed = 0
        self._dropped = 0

        # Sub best-effort (compatible with any publisher QoS); pub reliable (compatible with
        # any subscriber QoS) — so inserting the gate never creates a QoS mismatch.
        sub_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        pub_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Odometry, odom_topic, self._on_odom, sub_qos)
        self.create_subscription(LaserScan, scan_in, self._on_scan, sub_qos)
        self.pub = self.create_publisher(LaserScan, scan_out, pub_qos)

        self.create_timer(5.0, self._report)
        self.get_logger().info(
            f"gating {scan_in} -> {scan_out}: pass when |roll|,|pitch| <= "
            f"{math.degrees(self.max_tilt):.1f} deg (body attitude from {odom_topic})")

    def _on_odom(self, msg: Odometry):
        self._tilt = roll_pitch(msg.pose.pose.orientation)

    def _on_scan(self, msg: LaserScan):
        if self._tilt is None:
            self._dropped += 1  # no attitude yet — be conservative, never pass unknown tilt
            return
        roll, pitch = self._tilt
        if abs(roll) <= self.max_tilt and abs(pitch) <= self.max_tilt:
            self.pub.publish(msg)
            self._passed += 1
        else:
            self._dropped += 1

    def _report(self):
        total = self._passed + self._dropped
        if total:
            self.get_logger().info(
                f"scans passed {self._passed}/{total} "
                f"({100.0 * self._passed / total:.0f}%) in the last 5 s")
        self._passed = self._dropped = 0


def main(args=None):
    rclpy.init(args=args)
    node = ScanTiltGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
