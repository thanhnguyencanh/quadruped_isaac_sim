"""YOLO victim detector — Phase 2 perception (learned model).

Detects HUMAN victims with a pretrained YOLOv8 (COCO `person` class) instead of the HSV colour
filter, then localizes each person exactly like the HSV detector: back-project the box's
lower-centre pixel through the depth image, transform into the planner frame via tf2, and publish
spot_sar_msgs/VictimArray on /victims (same contract). The annotated `/camera/rgb/detections`
overlay + `/victims/markers` are reused unchanged.

SUBCLASSES the HSV `VictimDetector`: it reuses the rgb+depth sync, the CameraInfo cache, the tf
buffer, the publishers, `_depth_at`, `_to_target`, `_publish_overlay`, `_publish_markers` — only
the per-frame detection stage (HSV blobs -> YOLO person boxes) is overridden. The HSV node is left
untouched, so `detector:=hsv|yolo` selects between them with no shared risk.

Runs under ~/yolo_venv (ultralytics + CPU torch; numpy pinned to the system 1.26 ABI so cv_bridge
coexists). Launch passes an absolute `model:=~/yolo_venv/yolov8n.pt` so it never re-downloads:

  ~/yolo_venv/bin/python <install>/lib/spot_sar_perception/yolo_detector_node \
      --ros-args -p use_sim_time:=true -p model:=~/yolo_venv/yolov8n.pt -p device:=cpu
"""
import numpy as np
import rclpy

from spot_sar_msgs.msg import Victim, VictimArray
from spot_sar_perception.detector_node import VictimDetector


class YoloVictimDetector(VictimDetector):
    def __init__(self):
        super().__init__()  # reuses params, K cache, tf, /victims + overlay + marker pubs, rgbd sync
        self.declare_parameter("model", "yolov8n.pt")
        self.declare_parameter("device", "cpu")        # "cuda" optional (contends for Isaac's 8 GB VRAM)
        self.declare_parameter("conf", 0.4)
        self.declare_parameter("person_class", 0)      # COCO 'person'
        self.declare_parameter("infer_every_n", 5)     # process every Nth synced frame (~5 Hz)
        g = self.get_parameter
        self.yolo_conf = float(g("conf").value)
        self.person_class = int(g("person_class").value)
        self.infer_every_n = max(1, int(g("infer_every_n").value))
        self.device = g("device").value
        self._frame_i = 0
        from ultralytics import YOLO  # import here so a missing venv fails loudly at startup
        self.model = YOLO(g("model").value)
        self.get_logger().info(
            f"yolo_victim_detector up: model={g('model').value} device={self.device} "
            f"conf={self.yolo_conf} infer_every_n={self.infer_every_n} target_frame={self.target_frame}")

    def _on_rgbd(self, rgb_msg, depth_msg):
        out = VictimArray()
        out.header.stamp = rgb_msg.header.stamp
        out.header.frame_id = self.target_frame
        self._frame_i += 1
        if self.K is None or (self._frame_i % self.infer_every_n) != 0:
            self.pub.publish(out)  # publish empty on skipped frames for downstream liveness
            return

        bgr = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        if depth_msg.encoding == "16UC1":
            depth = depth.astype(np.float32) / 1000.0

        res = self.model.predict(bgr, conf=self.yolo_conf, classes=[self.person_class],
                                 device=self.device, verbose=False)[0]
        fx, fy = self.K[0, 0], self.K[1, 1]
        cx0, cy0 = self.K[0, 2], self.K[1, 2]

        dets = []  # (bbox_LTWH, (ui,vi), z, conf, pt_target) — for the reused viz
        for box in res.boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            u = (x1 + x2) / 2.0
            v = y1 + 0.65 * (y2 - y1)
            ui, vi = int(round(u)), int(round(v))
            # Depth = 25th percentile over the TORSO region (central 50% width, 30-80%
            # height) — NOT a single-point patch: at 8-9 m a person is only a few px wide,
            # the centre pixel often lands on BACKGROUND between the legs/beside the torso,
            # and the median of a tiny patch returns the wall depth — measured 10 phantom
            # "victims" back-projected onto the walls BEHIND real people. The person is
            # always CLOSER than the background, so a low percentile locks onto the body.
            h_img, w_img = depth.shape[:2]
            uu0 = int(max(0, x1 + 0.25 * (x2 - x1)))
            uu1 = int(min(w_img, x1 + 0.75 * (x2 - x1) + 1))
            vv0 = int(max(0, y1 + 0.30 * (y2 - y1)))
            vv1 = int(min(h_img, y1 + 0.80 * (y2 - y1) + 1))
            patch = depth[vv0:vv1, uu0:uu1].astype(np.float32).reshape(-1)
            patch = patch[np.isfinite(patch) & (patch > 0.0) & (patch < self.max_depth)]
            if patch.size < 8:
                continue  # box too small/far to localize reliably
            z = float(np.percentile(patch, 25))
            if z > 8.0:
                continue  # beyond reliable victim-localization range (and beyond any wall hit
                #           that could masquerade as a person: pure-background YOLO false
                #           positives project onto walls at 9-10 m)
            x = (u - cx0) * z / fx
            y = (v - cy0) * z / fy
            pt = self._to_target(x, y, z, rgb_msg.header.stamp)
            if pt is None:
                continue

            conf = float(box.conf[0])
            vic = Victim()
            vic.header.stamp = rgb_msg.header.stamp
            vic.header.frame_id = self.target_frame
            vic.id = -1  # unassociated; the world model assigns/merges ids
            vic.pose.position.x, vic.pose.position.y, vic.pose.position.z = pt
            vic.pose.orientation.w = 1.0
            vic.confidence = conf
            vic.source = "yolo"
            out.victims.append(vic)
            dets.append(((int(x1), int(y1), int(x2 - x1), int(y2 - y1)), (ui, vi), float(z), conf, pt))

        self.pub.publish(out)
        self._publish_overlay(bgr, dets, rgb_msg.header)
        self._publish_markers(dets, rgb_msg.header)
        if out.victims:
            self.get_logger().info(
                f"YOLO detected {len(out.victims)} person(s); first @ "
                f"({out.victims[0].pose.position.x:.2f}, {out.victims[0].pose.position.y:.2f}) "
                f"[{self.target_frame}]", throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = YoloVictimDetector()
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
