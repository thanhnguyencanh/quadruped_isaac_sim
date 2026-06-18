"""Victim detector — Phase 2 perception (first cut).

A deterministic RGB-D victim detector: threshold a distinctive victim colour in the RGB
image, back-project the blob centroid through the camera intrinsics into the optical frame,
transform it into a fixed frame via tf2, and publish the detections as
spot_sar_msgs/VictimArray on /victims.

Inputs (from spot_perception_app.py):
  /camera/rgb/image_raw    sensor_msgs/Image       (rgb8)
  /camera/depth/image_raw  sensor_msgs/Image       (32FC1 metres)
  /camera/rgb/camera_info  sensor_msgs/CameraInfo  (intrinsics K; cached, not synced)
  TF: <target_frame> <- ... <- camera_optical_frame  (odom->base_link->camera_link->optical)

Output:
  /victims                 spot_sar_msgs/VictimArray

This is intentionally simple and learning-free for a first cut — a real perception model
replaces the HSV stage later. Detection is by saturated colour so it stays deterministic in
simulation. Run with use_sim_time so stamps align with /clock:

  ros2 run spot_sar_perception detector_node --ros-args -p use_sim_time:=true
"""
import os

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import cv2
import message_filters
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
import tf2_geometry_msgs  # noqa: F401  (registers PointStamped for do_transform_point)

from spot_sar_msgs.msg import Victim, VictimArray


class VictimDetector(Node):
    def __init__(self):
        super().__init__("victim_detector")

        # ---- params ----
        self.declare_parameter("rgb_topic", "/camera/rgb/image_raw")
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("info_topic", "/camera/rgb/camera_info")
        self.declare_parameter("victims_topic", "/victims")
        # map appears only once SLAM (Phase 3) runs; until then odom is the fixed frame.
        self.declare_parameter("target_frame", "odom")
        self.declare_parameter("optical_frame", "camera_optical_frame")
        # HSV band for the orange victim marker (OpenCV H is 0-179).
        self.declare_parameter("hsv_lo", [5, 120, 80])
        self.declare_parameter("hsv_hi", [25, 255, 255])
        self.declare_parameter("min_area", 700)      # px; reject specks / horizon fragments
        self.declare_parameter("max_depth", 10.0)    # m; reject far/garbage depth
        self.declare_parameter("sync_slop", 0.05)    # s; rgb/depth approx-sync tolerance

        g = self.get_parameter
        self.target_frame = g("target_frame").value
        self.optical_frame = g("optical_frame").value
        self.hsv_lo = np.array(g("hsv_lo").value, dtype=np.uint8)
        self.hsv_hi = np.array(g("hsv_hi").value, dtype=np.uint8)
        self.min_area = int(g("min_area").value)
        self.max_depth = float(g("max_depth").value)

        self.bridge = CvBridge()
        self.K = None  # 3x3 intrinsics, cached from CameraInfo
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(VictimArray, g("victims_topic").value, 10)
        self.create_subscription(CameraInfo, g("info_topic").value, self._on_info, 10)

        rgb_sub = message_filters.Subscriber(self, Image, g("rgb_topic").value, qos_profile=qos_profile_sensor_data)
        depth_sub = message_filters.Subscriber(self, Image, g("depth_topic").value, qos_profile=qos_profile_sensor_data)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=10, slop=float(g("sync_slop").value)
        )
        self.sync.registerCallback(self._on_rgbd)

        self._warned_no_k = False
        self._warned_no_tf = False
        # one-shot debug: set DETECTOR_DEBUG_DIR to dump the first RGB frame + HSV mask
        self._debug_dir = os.environ.get("DETECTOR_DEBUG_DIR", "")
        self._saved_debug = False
        self.get_logger().info(
            f"victim_detector up: target_frame={self.target_frame}, "
            f"HSV {self.hsv_lo.tolist()}..{self.hsv_hi.tolist()}, min_area={self.min_area}"
        )

    def _on_info(self, msg: CameraInfo):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)

    def _depth_at(self, depth: np.ndarray, u: int, v: int) -> float:
        """Median of a small valid neighbourhood (guards depth holes at blob centroids)."""
        h, w = depth.shape[:2]
        v0, v1 = max(0, v - 2), min(h, v + 3)
        u0, u1 = max(0, u - 2), min(w, u + 3)
        patch = depth[v0:v1, u0:u1].astype(np.float32).reshape(-1)
        patch = patch[np.isfinite(patch)]
        patch = patch[(patch > 0.0) & (patch < self.max_depth)]
        return float(np.median(patch)) if patch.size else float("nan")

    def _on_rgbd(self, rgb_msg: Image, depth_msg: Image):
        out = VictimArray()
        out.header.stamp = rgb_msg.header.stamp
        out.header.frame_id = self.target_frame

        if self.K is None:
            if not self._warned_no_k:
                self.get_logger().warn("waiting for CameraInfo (no intrinsics yet)…")
                self._warned_no_k = True
            self.pub.publish(out)  # publish empty for downstream liveness
            return

        bgr = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        if depth_msg.encoding == "16UC1":  # mm -> m, if a uint16 depth ever shows up
            depth = depth.astype(np.float32) / 1000.0

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lo, self.hsv_hi)
        # open with a small kernel to drop specks; close with a large kernel so a single
        # marker that grid lines / edges fragment is merged back into one blob.
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((15, 15), np.uint8))

        if self._debug_dir and not self._saved_debug:
            try:
                cv2.imwrite(os.path.join(self._debug_dir, "victim_cam_rgb.png"), bgr)
                cv2.imwrite(os.path.join(self._debug_dir, "victim_cam_mask.png"), mask)
                self._saved_debug = True
                self.get_logger().info(f"saved debug frames -> {self._debug_dir}")
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f"debug save failed: {e}")

        n, _labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
        fx, fy = self.K[0, 0], self.K[1, 1]
        cx0, cy0 = self.K[0, 2], self.K[1, 2]

        for lab in range(1, n):  # 0 is background
            area = int(stats[lab, cv2.CC_STAT_AREA])
            if area < self.min_area:
                continue
            u, v = centroids[lab]
            ui, vi = int(round(u)), int(round(v))
            z = self._depth_at(depth, ui, vi)
            if not np.isfinite(z):
                continue
            # back-project to the optical frame (z-fwd, x-right, y-down)
            x = (u - cx0) * z / fx
            y = (v - cy0) * z / fy
            pt = self._to_target(x, y, z, rgb_msg.header.stamp)
            if pt is None:
                continue

            vic = Victim()
            vic.header.stamp = rgb_msg.header.stamp
            vic.header.frame_id = self.target_frame
            vic.id = -1  # unassociated; the world model assigns/merges IDs later
            vic.pose.position.x, vic.pose.position.y, vic.pose.position.z = pt
            vic.pose.orientation.w = 1.0
            vic.confidence = float(min(1.0, area / 5000.0))
            vic.source = "rgbd"
            out.victims.append(vic)

        self.pub.publish(out)
        if out.victims:
            self.get_logger().info(
                f"detected {len(out.victims)} victim(s); first @ "
                f"({out.victims[0].pose.position.x:.2f}, {out.victims[0].pose.position.y:.2f}, "
                f"{out.victims[0].pose.position.z:.2f}) [{self.target_frame}]",
                throttle_duration_sec=2.0,
            )

    def _to_target(self, x, y, z, stamp):
        ps = PointStamped()
        ps.header.frame_id = self.optical_frame
        ps.header.stamp = stamp
        ps.point.x, ps.point.y, ps.point.z = float(x), float(y), float(z)
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame, self.optical_frame, stamp,
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
            out = tf2_geometry_msgs.do_transform_point(ps, tf)
            return (out.point.x, out.point.y, out.point.z)
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            if not self._warned_no_tf:
                self.get_logger().warn(f"TF {self.target_frame}<-{self.optical_frame} not ready: {e}")
                self._warned_no_tf = True
            return None


def main(args=None):
    rclpy.init(args=args)
    node = VictimDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
