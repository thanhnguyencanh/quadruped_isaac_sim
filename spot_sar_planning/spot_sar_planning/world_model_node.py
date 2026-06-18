"""World model / symbol grounding — Phase 3 (first cut).

Bridges the metric world to a small SYMBOLIC model the PDDL layer can reason over. It:
  * accumulates unique victims from /victims (dedup by proximity, stable ids),
  * discretizes space into a coarse cell grid -> symbolic `locations` ("L_<ix>_<iy>"),
  * tracks which cells the robot has visited (`explored`) and the robot's current cell,
  * derives 4-neighbour `connectivity` between locations,
  * assigns each victim to its cell,
and publishes spot_sar_msgs/WorldModel on /world_model for the executive (Phase 6) to turn
into a PDDL problem. Pure helpers (cell_id / cell_center / cell_indices / merge_victim) are
module-level so the grounding is unit-testable without a running sim.

  ros2 run spot_sar_planning world_model_node --ros-args -p use_sim_time:=true
"""
import math

import rclpy
from rclpy.node import Node

import tf2_geometry_msgs  # noqa: F401  (registers PointStamped for do_transform_point)
from geometry_msgs.msg import Point, PointStamped
from tf2_ros import Buffer, ConnectivityException, ExtrapolationException, LookupException, TransformListener

from spot_sar_msgs.msg import Victim, WorldModel, VictimArray


def cell_id(x: float, y: float, cell: float) -> str:
    return f"L_{int(math.floor(x / cell))}_{int(math.floor(y / cell))}"


def cell_indices(cid: str):
    _, ix, iy = cid.split("_")
    return int(ix), int(iy)


def cell_center(cid: str, cell: float):
    ix, iy = cell_indices(cid)
    return ((ix + 0.5) * cell, (iy + 0.5) * cell)


def merge_victim(known: list, x: float, y: float, z: float, conf: float, src: str,
                 merge_dist: float, next_id: int):
    """Merge a detection into `known` (list of dicts) by proximity. Returns the new next_id."""
    for kv in known:
        if math.hypot(kv["x"] - x, kv["y"] - y) < merge_dist:
            kv["x"] = 0.7 * kv["x"] + 0.3 * x
            kv["y"] = 0.7 * kv["y"] + 0.3 * y
            kv["z"] = 0.7 * kv["z"] + 0.3 * z
            kv["confidence"] = max(kv["confidence"], conf)
            return next_id
    known.append({"id": next_id, "x": x, "y": y, "z": z,
                  "confidence": conf, "source": src, "reported": False})
    return next_id + 1


class WorldModelNode(Node):
    def __init__(self):
        super().__init__("world_model")
        self.declare_parameter("fixed_frame", "odom")  # "map" once SLAM is trusted
        self.declare_parameter("cell_size", 2.0)
        self.declare_parameter("victim_merge_dist", 0.6)
        self.declare_parameter("publish_period", 1.0)
        self.fixed_frame = self.get_parameter("fixed_frame").value
        self.cell = float(self.get_parameter("cell_size").value)
        self.merge_dist = float(self.get_parameter("victim_merge_dist").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.victims = []
        self.next_id = 0
        self.visited = set()
        self.robot_cell = ""

        self.create_subscription(VictimArray, "/victims", self._on_victims, 10)
        self.pub = self.create_publisher(WorldModel, "/world_model", 10)
        self.create_timer(float(self.get_parameter("publish_period").value), self._tick)
        self.get_logger().info(
            f"world_model up: fixed_frame={self.fixed_frame}, cell_size={self.cell} m"
        )

    def _to_fixed(self, pt: Point, src_frame: str):
        if not src_frame or src_frame == self.fixed_frame:
            return (pt.x, pt.y, pt.z)
        ps = PointStamped()
        ps.header.frame_id = src_frame
        ps.point = pt
        try:
            tf = self.tf_buffer.lookup_transform(self.fixed_frame, src_frame, rclpy.time.Time())
            out = tf2_geometry_msgs.do_transform_point(ps, tf)
            return (out.point.x, out.point.y, out.point.z)
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

    def _on_victims(self, msg: VictimArray):
        for v in msg.victims:
            src = msg.header.frame_id or v.header.frame_id
            p = self._to_fixed(v.pose.position, src)
            if p is None:
                continue
            self.next_id = merge_victim(self.victims, p[0], p[1], p[2],
                                        float(v.confidence), v.source, self.merge_dist, self.next_id)

    def _robot_xy(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.fixed_frame, "base_link", rclpy.time.Time())
            return (tf.transform.translation.x, tf.transform.translation.y)
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

    def _tick(self):
        wm = WorldModel()
        wm.header.stamp = self.get_clock().now().to_msg()
        wm.header.frame_id = self.fixed_frame

        rxy = self._robot_xy()
        if rxy is not None:
            self.robot_cell = cell_id(rxy[0], rxy[1], self.cell)
            self.visited.add(self.robot_cell)

        loc_set = set(self.visited)
        for v in self.victims:
            loc_set.add(cell_id(v["x"], v["y"], self.cell))
        locations = sorted(loc_set)

        wm.locations = locations
        wm.centroids = [Point(x=cell_center(c, self.cell)[0], y=cell_center(c, self.cell)[1], z=0.0)
                        for c in locations]
        wm.explored = sorted(self.visited)

        locset = set(locations)
        a_list, b_list = [], []
        for cid in locations:
            ix, iy = cell_indices(cid)
            for nx, ny in ((ix + 1, iy), (ix, iy + 1)):
                ncid = f"L_{nx}_{ny}"
                if ncid in locset:
                    a_list.append(cid)
                    b_list.append(ncid)
        wm.connected_a = a_list
        wm.connected_b = b_list
        wm.robot_location = self.robot_cell

        wm.victims, wm.victim_location, wm.victim_reported = [], [], []
        for v in self.victims:
            vic = Victim()
            vic.header.frame_id = self.fixed_frame
            vic.header.stamp = wm.header.stamp
            vic.id = int(v["id"])
            vic.pose.position.x, vic.pose.position.y, vic.pose.position.z = v["x"], v["y"], v["z"]
            vic.pose.orientation.w = 1.0
            vic.confidence = float(v["confidence"])
            vic.source = v["source"]
            wm.victims.append(vic)
            wm.victim_location.append(cell_id(v["x"], v["y"], self.cell))
            wm.victim_reported.append(bool(v["reported"]))

        self.pub.publish(wm)
        self.get_logger().info(
            f"world_model: {len(locations)} locations, {len(self.victims)} victims, "
            f"robot@{self.robot_cell or '?'}",
            throttle_duration_sec=5.0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = WorldModelNode()
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
