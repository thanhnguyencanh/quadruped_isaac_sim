"""Building world model / symbol grounding for the TWO-FLOOR building demo (parallel to
floor_world_model_node). Grounds the static two-floor plan (`sar_building.py`) into the room graph
the building PDDL domain reasons over:
  * locations      = all rooms incl. BOTH stair landings (f1_stair, f2_stair),
  * robot_location  = room_of(robot xy, floor) with HYSTERESIS. WHICH FLOOR comes from the latched
                      /floor_state (authoritative — the two landings share (x,y) so xy alone cannot
                      disambiguate them; this is the one place the x-offset trick needs a tie-break),
  * doors           = ALL portals: the two closeable floor-1 doors (open-state from /door_states) +
                      the always-open passages,
  * victims         = the a-priori victim rooms (one per floor) so the planner has goals on both
                      floors and must use the stairs to reach floor 2.

The STAIR topology is NOT put on the wire — the planner/executive import sar_building.STAIRS
directly (keeps spot_sar_msgs unchanged). Publishes spot_sar_msgs/WorldModel on /world_model for the
executive in its `building` profile.

  ros2 run spot_sar_planning building_world_model_node --ros-args -p use_sim_time:=true
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from geometry_msgs.msg import Point
from std_msgs.msg import String
from tf2_ros import Buffer, ConnectivityException, ExtrapolationException, LookupException, TransformListener

from spot_sar_msgs.msg import Victim, WorldModel
from spot_sar_planning import sar_building as B


class BuildingWorldModelNode(Node):
    def __init__(self):
        super().__init__("building_world_model")
        self.declare_parameter("fixed_frame", "map")   # "odom" before SLAM is trusted
        self.declare_parameter("publish_period", 1.0)
        self.fixed_frame = self.get_parameter("fixed_frame").value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.robot_room = B.SPAWN_ROOM          # hysteresis seed
        self.floor = "f1"                        # authoritative floor from /floor_state
        self.open_doors = set()                  # closeable door ids reported open by the sim

        latched = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/door_states", self._on_door_state, latched)
        self.create_subscription(String, "/floor_state", self._on_floor_state, latched)
        self.pub = self.create_publisher(WorldModel, "/world_model", 10)
        self.create_timer(float(self.get_parameter("publish_period").value), self._tick)
        self.get_logger().info(
            f"building_world_model up: fixed_frame={self.fixed_frame}, "
            f"rooms={[r.id for r in B.ROOMS]}, stairs={[s.id for s in B.STAIRS]}")

    def _on_door_state(self, msg: String):
        parts = msg.data.split()
        if not parts:
            return
        did = parts[0].strip()
        state = parts[1].strip().lower() if len(parts) > 1 else "open"
        if state.startswith("open"):
            self.open_doors.add(did)
        else:
            self.open_doors.discard(did)

    def _on_floor_state(self, msg: String):
        f = msg.data.strip()
        if f in ("f1", "f2"):
            self.floor = f

    def _robot_room(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.fixed_frame, "base_link", rclpy.time.Time())
            # floor from /floor_state disambiguates the two stacked landings; hysteresis holds the
            # last valid room while crossing a divider/doorway (room_of returns "").
            r = B.room_of(tf.transform.translation.x, tf.transform.translation.y, floor=self.floor)
            if r:
                self.robot_room = r
        except (LookupException, ConnectivityException, ExtrapolationException):
            pass
        return self.robot_room

    def _tick(self):
        wm = WorldModel()
        wm.header.stamp = self.get_clock().now().to_msg()
        wm.header.frame_id = self.fixed_frame

        wm.locations = [r.id for r in B.ROOMS]
        # nav_centroid = the clear standing point per room (landings are offset west of the stairs)
        wm.centroids = [Point(x=float(B.nav_centroid(r.id)[0]), y=float(B.nav_centroid(r.id)[1]),
                              z=float(B.FLOOR_Z[r.floor])) for r in B.ROOMS]
        wm.explored = [r.id for r in B.ROOMS]     # static building: rooms are known up front
        wm.robot_location = self._robot_room()

        # connectivity (room pairs through portals) — kept for any generic consumer
        wm.connected_a = [p.room_a for p in B.PORTALS]
        wm.connected_b = [p.room_b for p in B.PORTALS]

        # portals published as doors: passages always open; closeable floor-1 doors from /door_states
        wm.doors = [p.id for p in B.PORTALS]
        wm.door_room_a = [p.room_a for p in B.PORTALS]
        wm.door_room_b = [p.room_b for p in B.PORTALS]
        wm.door_open = [bool(p.always_open or p.id in self.open_doors) for p in B.PORTALS]

        # victims: a-priori known rooms to search (the planner's goals), one per floor
        wm.victims, wm.victim_location, wm.victim_reported = [], [], []
        for i, (x, y, z, room) in enumerate(B.VICTIMS):
            vic = Victim()
            vic.header.frame_id = self.fixed_frame
            vic.header.stamp = wm.header.stamp
            vic.id = i
            vic.pose.position.x, vic.pose.position.y, vic.pose.position.z = float(x), float(y), float(z)
            vic.pose.orientation.w = 1.0
            vic.confidence = 1.0
            vic.source = "building_plan"
            wm.victims.append(vic)
            wm.victim_location.append(room)
            wm.victim_reported.append(False)

        self.pub.publish(wm)
        self.get_logger().info(
            f"building_world_model: robot@{wm.robot_location} (floor={self.floor}), "
            f"open_doors={sorted(self.open_doors) or '[]'}",
            throttle_duration_sec=5.0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = BuildingWorldModelNode()
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
