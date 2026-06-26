"""Floor world model / symbol grounding for the multi-room DOOR demo (parallel to world_model_node).

Where the grid world model discretizes free space into cells, this grounds the STATIC floor plan
(`sar_floor.py`) into a ROOM graph the doors PDDL domain reasons over:
  * locations           = room ids (room_a, room_b, room_c),
  * robot_location       = room_of(robot xy from TF), with HYSTERESIS (hold the last room while the
                           robot crosses a doorway, where room_of() returns ""),
  * victims              = the a-priori victim locations from the floor plan (the scenario knows
                           which rooms to search), so the planner has a goal and must open doors to
                           reach them; the live detector + overlay still run for visualization,
  * doors                = topology from sar_floor + open-state accumulated from /door_states.

Publishes spot_sar_msgs/WorldModel on /world_model (door fields populated) for the executive in its
`doors` profile.  ros2 run spot_sar_planning floor_world_model_node --ros-args -p use_sim_time:=true
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from geometry_msgs.msg import Point
from std_msgs.msg import String
from tf2_ros import Buffer, ConnectivityException, ExtrapolationException, LookupException, TransformListener

from spot_sar_msgs.msg import Victim, WorldModel
from spot_sar_planning import sar_floor as F


class FloorWorldModelNode(Node):
    def __init__(self):
        super().__init__("floor_world_model")
        self.declare_parameter("fixed_frame", "map")  # "odom" before SLAM is trusted
        self.declare_parameter("publish_period", 1.0)
        self.fixed_frame = self.get_parameter("fixed_frame").value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.robot_room = F.SPAWN_ROOM          # hysteresis seed
        self.open_doors = set()                  # door ids reported open by the sim

        latched = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/door_states", self._on_door_state, latched)
        self.pub = self.create_publisher(WorldModel, "/world_model", 10)
        self.create_timer(float(self.get_parameter("publish_period").value), self._tick)
        self.get_logger().info(
            f"floor_world_model up: fixed_frame={self.fixed_frame}, "
            f"rooms={[r.id for r in F.ROOMS]}, doors={[d.id for d in F.DOORS]}"
        )

    def _on_door_state(self, msg: String):
        # "<id>" or "<id> open" -> open; "<id> close[d]" -> closed
        parts = msg.data.split()
        if not parts:
            return
        did = parts[0].strip()
        state = parts[1].strip().lower() if len(parts) > 1 else "open"
        if state.startswith("open"):
            self.open_doors.add(did)
        else:
            self.open_doors.discard(did)

    def _robot_room(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.fixed_frame, "base_link", rclpy.time.Time())
            r = F.room_of(tf.transform.translation.x, tf.transform.translation.y)
            if r:                                # only update on a confident in-room reading
                self.robot_room = r
        except (LookupException, ConnectivityException, ExtrapolationException):
            pass
        return self.robot_room

    def _tick(self):
        wm = WorldModel()
        wm.header.stamp = self.get_clock().now().to_msg()
        wm.header.frame_id = self.fixed_frame

        rooms = [r.id for r in F.ROOMS]
        wm.locations = rooms
        wm.centroids = [Point(x=r.center[0], y=r.center[1], z=0.0) for r in F.ROOMS]
        wm.explored = list(rooms)                # static floor: rooms are known up front
        wm.robot_location = self._robot_room()

        # connectivity (room pairs through doors) — kept for any generic consumer
        wm.connected_a = [d.room_a for d in F.DOORS]
        wm.connected_b = [d.room_b for d in F.DOORS]

        # doors: topology from the floor plan + live open-state
        wm.doors = [d.id for d in F.DOORS]
        wm.door_room_a = [d.room_a for d in F.DOORS]
        wm.door_room_b = [d.room_b for d in F.DOORS]
        wm.door_open = [d.id in self.open_doors for d in F.DOORS]

        # victims: a-priori known rooms to search (the planner's goals)
        wm.victims, wm.victim_location, wm.victim_reported = [], [], []
        for i, (x, y, z, room) in enumerate(F.VICTIMS):
            vic = Victim()
            vic.header.frame_id = self.fixed_frame
            vic.header.stamp = wm.header.stamp
            vic.id = i
            vic.pose.position.x, vic.pose.position.y, vic.pose.position.z = float(x), float(y), float(z)
            vic.pose.orientation.w = 1.0
            vic.confidence = 1.0
            vic.source = "floor_plan"
            wm.victims.append(vic)
            wm.victim_location.append(room)
            wm.victim_reported.append(False)

        self.pub.publish(wm)
        self.get_logger().info(
            f"floor_world_model: robot@{wm.robot_location}, "
            f"open_doors={sorted(self.open_doors) or '[]'}",
            throttle_duration_sec=5.0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = FloorWorldModelNode()
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
