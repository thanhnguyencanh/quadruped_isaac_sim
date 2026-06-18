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
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("min_cluster", 8)
        self.declare_parameter("free_thresh", 25)
        self.declare_parameter("planner_frame", "map")
        self.declare_parameter("robot_base_frame", "base_link")
        self.declare_parameter("replan_period", 2.0)

        self.frame = self.get_parameter("planner_frame").value
        self.base = self.get_parameter("robot_base_frame").value
        self.min_cluster = int(self.get_parameter("min_cluster").value)
        self.free_thresh = int(self.get_parameter("free_thresh").value)

        self.grid = None
        self.busy = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.create_subscription(OccupancyGrid, self.get_parameter("map_topic").value, self._on_map, 1)
        self.create_timer(float(self.get_parameter("replan_period").value), self._tick)
        self.get_logger().info("frontier_explorer up; waiting for /map + nav2…")

    def _on_map(self, msg: OccupancyGrid):
        self.grid = msg

    def _robot_xy(self):
        try:
            t = self.tf_buffer.lookup_transform(self.frame, self.base, rclpy.time.Time())
            return (t.transform.translation.x, t.transform.translation.y)
        except (LookupException, ConnectivityException, ExtrapolationException):
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
        rxy = self._robot_xy() or (info.origin.position.x, info.origin.position.y)
        # pick the frontier maximizing size / distance (nearest, biggest)
        best = max(cents, key=lambda c: c[2] / (1.0 + math.hypot(c[0] - rxy[0], c[1] - rxy[1])))
        self._go(best[0], best[1])

    def _go(self, x, y):
        if not self.nav.server_is_ready():
            self.get_logger().warn("navigate_to_pose server not ready yet", throttle_duration_sec=5.0)
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.w = 1.0
        self.busy = True
        self.get_logger().info(f"exploring frontier @ ({x:.2f}, {y:.2f})")
        self.nav.send_goal_async(goal).add_done_callback(self._on_goal_resp)

    def _on_goal_resp(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self.busy = False
            return
        gh.get_result_async().add_done_callback(lambda _f: setattr(self, "busy", False))


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
