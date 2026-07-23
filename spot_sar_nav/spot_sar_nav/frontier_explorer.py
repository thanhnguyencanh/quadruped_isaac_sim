"""Frontier exploration — Phase 4.

Autonomously explores unknown space: finds frontiers (free cells bordering unknown space) on
the slam_toolbox /map, picks the best one, and drives there via the Nav2 NavigateToPose action.
Repeats until no frontiers remain (coverage done). Used as the "explore" skill and as a
standalone coverage behavior.

Pure helper `find_frontiers(grid, w, h, ...)` is module-level for unit testing without ROS.

  ros2 run spot_sar_nav frontier_explorer --ros-args -p use_sim_time:=true
"""
import math

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, ConnectivityException, ExtrapolationException, LookupException, TransformListener


def find_frontiers(data, width, height, resolution, origin_x, origin_y,
                   free_thresh=25, min_cluster=8):
    """Return frontier cluster centroids as a list of (x, y) world points.

    data: flat occupancy list (-1 unknown, 0..100 occupancy). A frontier cell is FREE
    (0..free_thresh) with at least one UNKNOWN (-1) 4-neighbour. Adjacent frontier cells are
    clustered (BFS); clusters smaller than min_cluster are dropped.
    """
    g = np.asarray(data, dtype=np.int16).reshape(height, width)
    free = (g >= 0) & (g <= free_thresh)
    unknown = g < 0
    # a free cell is a frontier if any 4-neighbour is unknown
    fr = np.zeros_like(free)
    fr[1:, :] |= free[1:, :] & unknown[:-1, :]
    fr[:-1, :] |= free[:-1, :] & unknown[1:, :]
    fr[:, 1:] |= free[:, 1:] & unknown[:, :-1]
    fr[:, :-1] |= free[:, :-1] & unknown[:, 1:]

    # cluster frontier cells with iterative BFS (8-connected)
    seen = np.zeros_like(fr)
    centroids = []
    ys, xs = np.where(fr)
    fr_set = set(zip(ys.tolist(), xs.tolist()))
    for sy, sx in zip(ys.tolist(), xs.tolist()):
        if seen[sy, sx]:
            continue
        stack = [(sy, sx)]
        seen[sy, sx] = 1
        cells = []
        while stack:
            cy, cx = stack.pop()
            cells.append((cy, cx))
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    ny, nx = cy + dy, cx + dx
                    if (ny, nx) in fr_set and not seen[ny, nx]:
                        seen[ny, nx] = 1
                        stack.append((ny, nx))
        if len(cells) >= min_cluster:
            my = sum(c[0] for c in cells) / len(cells)
            mx = sum(c[1] for c in cells) / len(cells)
            wx = origin_x + (mx + 0.5) * resolution
            wy = origin_y + (my + 0.5) * resolution
            centroids.append((wx, wy, len(cells)))
    return centroids


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__("frontier_explorer")
        # Frontiers come from the GLOBAL COSTMAP, not slam's /map: misses are inf (REP-117),
        # which karto ignores entirely (finite miss markers poison its scan matcher — see
        # slam_toolbox.yaml), so /map only carves free space along hit rays and would starve
        # frontier detection. The costmap raytrace-clears inf beams (inf_is_valid: true) and
        # is published as an OccupancyGrid too (-1 unknown / 0..100 cost) — same math applies.
        # It lives in the odom frame, so goals skip the map->odom transform entirely.
        self.declare_parameter("map_topic", "/global_costmap/costmap")
        self.declare_parameter("min_cluster", 8)
        self.declare_parameter("free_thresh", 25)
        self.declare_parameter("planner_frame", "odom")
        self.declare_parameter("robot_base_frame", "base_link")
        self.declare_parameter("replan_period", 2.0)
        # Frontiers closer than this to the robot are UNCLEAREABLE: they sit inside the depth
        # camera's minimum-range blind zone (range_min 0.3 + the body footprint), so standing
        # "on" them never marks them known — chasing one livelocks exploration in place.
        self.declare_parameter("min_frontier_dist", 1.0)
        # Some frontiers are UNPLANNABLE artifacts (raytrace aliasing can leak free cells
        # through a grazing-angle wall, putting a frontier outside the building). Without a
        # blacklist the explorer re-sends them forever: measured 472 goals / 875 aborts in
        # 240 s, overloading the planner ("Costmap timed out waiting for update").
        self.declare_parameter("blacklist_radius", 0.7)
        self.declare_parameter("blacklist_ttl", 180.0)   # map growth may legitimize an area later
        self.declare_parameter("fail_cooldown", 3.0)     # pause after a failed goal (no churn)

        self.frame = self.get_parameter("planner_frame").value
        self.base = self.get_parameter("robot_base_frame").value
        self.min_cluster = int(self.get_parameter("min_cluster").value)
        self.free_thresh = int(self.get_parameter("free_thresh").value)

        self.grid = None
        self.busy = False
        self._blacklist = []  # (x, y, stamp_sec) of goals that did not succeed
        self._cooldown_until = 0.0
        self._last_goal = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.create_subscription(OccupancyGrid, self.get_parameter("map_topic").value, self._on_map, 1)
        self.create_timer(float(self.get_parameter("replan_period").value), self._tick)
        self.get_logger().info("frontier_explorer up; waiting for /map + nav2…")

    def _on_map(self, msg: OccupancyGrid):
        self.grid = msg

    def _robot_xy(self):
        # Try the planner frame first, then odom (same chain minus slam's map->odom leg —
        # near-identical in sim, and infinitely better than the map-origin fallback the
        # caller would otherwise use). Log failures: a silent None here once livelocked
        # exploration on an uncleareable blind-zone frontier.
        for frame in (self.frame, "odom"):
            try:
                t = self.tf_buffer.lookup_transform(frame, self.base, rclpy.time.Time())
                return (t.transform.translation.x, t.transform.translation.y)
            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                self.get_logger().warn(f"robot pose lookup {frame}->{self.base} failed: {e}",
                                       throttle_duration_sec=10.0)
        return None

    def _tick(self):
        if self.busy or self.grid is None:
            return
        info = self.grid.info
        cents = find_frontiers(self.grid.data, info.width, info.height, info.resolution,
                               info.origin.position.x, info.origin.position.y,
                               self.free_thresh, self.min_cluster)
        if not cents:
            self.get_logger().info("no frontiers left — exploration complete.", throttle_duration_sec=10.0)
            return
        rxy = self._robot_xy()
        if rxy is None:
            return  # no robot pose -> can't rank frontiers or filter the blind zone; wait a tick
        # drop blind-zone frontiers (see min_frontier_dist) — they can never be cleared
        min_d = float(self.get_parameter("min_frontier_dist").value)
        cents = [c for c in cents if math.hypot(c[0] - rxy[0], c[1] - rxy[1]) > min_d]
        if not cents:
            self.get_logger().info("only blind-zone frontiers remain — exploration complete.",
                                   throttle_duration_sec=10.0)
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now < self._cooldown_until:
            return
        ttl = float(self.get_parameter("blacklist_ttl").value)
        self._blacklist = [b for b in self._blacklist if now - b[2] < ttl]
        br = float(self.get_parameter("blacklist_radius").value)
        cents = [c for c in cents
                 if all(math.hypot(c[0] - bx, c[1] - by) > br for bx, by, _t in self._blacklist)]
        if not cents:
            self.get_logger().info("all remaining frontiers blacklisted — waiting for map growth",
                                   throttle_duration_sec=10.0)
            return
        # pick the frontier maximizing size / distance (nearest, biggest)
        best = max(cents, key=lambda c: c[2] / (1.0 + math.hypot(c[0] - rxy[0], c[1] - rxy[1])))
        self._go(best[0], best[1])

    def _go(self, x, y):
        if not self.nav.server_is_ready():
            self.get_logger().warn("navigate_to_pose server not ready yet", throttle_duration_sec=5.0)
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.frame
        # stamp ZERO = "latest available transform": bt_navigator transforms this map-framed goal
        # into its odom global frame without racing slam's map->odom stamp (a now() stamp fails
        # with "Failed to transform a goal pose" whenever the transform lags a scan-match).
        goal.pose.header.stamp = rclpy.time.Time().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.w = 1.0
        self.busy = True
        self._last_goal = (float(x), float(y))
        self.get_logger().info(f"exploring frontier @ ({x:.2f}, {y:.2f})")
        self.nav.send_goal_async(goal).add_done_callback(self._on_goal_resp)

    def _on_goal_resp(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self._on_fail()
            return
        gh.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, fut):
        if fut.result().status == GoalStatus.STATUS_SUCCEEDED:
            self.busy = False
        else:
            self._on_fail()

    def _on_fail(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._last_goal is not None:
            self._blacklist.append((self._last_goal[0], self._last_goal[1], now))
            self.get_logger().info(
                f"goal @ ({self._last_goal[0]:.2f}, {self._last_goal[1]:.2f}) failed — "
                f"blacklisted ({len(self._blacklist)} active)")
        self._cooldown_until = now + float(self.get_parameter("fail_cooldown").value)
        self.busy = False


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
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
