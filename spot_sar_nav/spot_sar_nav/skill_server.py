"""Skill server — Phase 4: dispatchable SAR skills as a ROS 2 action.

Hosts the /skill action (spot_sar_msgs/action/Skill); the Task Executive (Phase 6) sends one
Skill goal per PDDL action and branches on success/failure to replan. Skills:

  go_to_location  -> Nav2 NavigateToPose to the target location's centroid (from /world_model)
  explore         -> drive to the biggest global-costmap frontier (NavigateToPose)
  observe         -> dwell and report how many victims are currently seen on /victims
  report          -> mark a victim reported (logged; the executive tracks reported state)

Runs under a MultiThreadedExecutor with a ReentrantCallbackGroup so the action server can call
the Nav2 action client and block on its result without deadlocking.

  ros2 run spot_sar_nav skill_server --ros-args -p use_sim_time:=true
"""
import math
import time

import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from tf2_ros import (Buffer, ConnectivityException, ExtrapolationException, LookupException,
                     TransformListener)

from spot_sar_msgs.action import Skill
from spot_sar_msgs.msg import VictimArray, WorldModel
from spot_sar_nav.frontier_explorer import find_frontiers


class SkillServer(Node):
    def __init__(self):
        super().__init__("skill_server")
        # odom, not map: world-model/victim positions are grounded in odom, nav runs in odom,
        # and the explore skill's frontiers come from the odom-framed global costmap.
        self.declare_parameter("planner_frame", "odom")
        self.declare_parameter("observe_dwell", 3.0)
        # explore skill: same blind-zone rule as frontier_explorer — the costmap's 5 m clearing
        # disc yields a frontier RING around the robot whose cluster centroid lands ON the
        # robot; without this filter explore "reaches" that goal instantly (0.1 s no-op
        # treadmill: measured 287 instant successes / zero motion in one mission).
        self.declare_parameter("min_frontier_dist", 1.0)
        self.frame = self.get_parameter("planner_frame").value
        self.observe_dwell = float(self.get_parameter("observe_dwell").value)

        self.cb = ReentrantCallbackGroup()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.wm = None
        self.grid = None
        self.n_victims = 0

        self._opened_doors = set()  # door ids reported open on /door_states (floor demo)
        self._floor = "f1"          # current floor from /floor_state (two-floor building demo)
        _latched = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(WorldModel, "/world_model", self._on_wm, 10, callback_group=self.cb)
        # global costmap, not /map: with REP-117 inf misses karto only carves free space along
        # hit rays, so /map starves frontier detection; the costmap raytrace-clears inf beams
        # (inf_is_valid) and is the "explored free space" source (same as frontier_explorer).
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap", self._on_map, 1,
                                 callback_group=self.cb)
        self.create_subscription(VictimArray, "/victims", self._on_vic, 10, callback_group=self.cb)
        self.create_subscription(String, "/door_states", self._on_door_state, _latched,
                                 callback_group=self.cb)
        self.create_subscription(String, "/floor_state", self._on_floor_state, _latched,
                                 callback_group=self.cb)
        self.door_cmd = self.create_publisher(String, "/door_cmd", 10)
        self.stairs_cmd = self.create_publisher(String, "/stairs_cmd", 10)
        # best-effort costmap clears after a floor teleport (drop stale marks at the landing)
        self._clear_global = self.create_client(ClearEntireCostmap,
                                                 "/global_costmap/clear_entirely_global_costmap")
        self._clear_local = self.create_client(ClearEntireCostmap,
                                                "/local_costmap/clear_entirely_local_costmap")
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose", callback_group=self.cb)
        self.srv = ActionServer(self, Skill, "skill", execute_callback=self._execute,
                                callback_group=self.cb)
        self.get_logger().info(
            "skill_server up: /skill {go_to_location|explore|observe|report|open_door|climb_stairs}")

    def _on_wm(self, m):
        self.wm = m

    def _on_map(self, m):
        self.grid = m

    def _on_vic(self, m):
        self.n_victims = len(m.victims)

    def _on_door_state(self, m):
        # "<id>" or "<id> open" -> open; "<id> close[d]" -> closed
        parts = m.data.split()
        if not parts:
            return
        did = parts[0].strip()
        state = parts[1].strip().lower() if len(parts) > 1 else "open"
        if state.startswith("open"):
            self._opened_doors.add(did)
        else:
            self._opened_doors.discard(did)

    def _on_floor_state(self, m):
        f = m.data.strip()
        if f in ("f1", "f2"):
            self._floor = f

    def _clear_costmaps(self):
        """Best-effort: drop stale marks (the robot's pre-teleport trace / the decorative stairs) at
        the landing after a floor change. Non-fatal if Nav2 costmaps aren't up."""
        for cli in (self._clear_global, self._clear_local):
            try:
                if cli.wait_for_service(timeout_sec=1.0):
                    cli.call_async(ClearEntireCostmap.Request())
            except Exception:  # noqa: BLE001
                pass

    def _climb_stairs(self, arg, timeout=25.0):
        """Change floors via the stairwell: publish '<stair_id> up|down' on /stairs_cmd and BLOCK
        until /floor_state confirms the target floor. Same busy-poll pattern as _open_door / _nav_to
        (safe under ReentrantCallbackGroup): the executor keeps servicing /floor_state while we wait.
        The in-app StairsNode performs the pure-z teleport (Spot's flat policy cannot climb)."""
        parts = arg.split()
        if not parts:
            return False, "empty climb_stairs target"
        direction = parts[1].strip().lower() if len(parts) > 1 else "up"
        target = "f2" if direction == "up" else "f1"
        if self._floor == target:
            return True, f"already on {target}"
        self.stairs_cmd.publish(String(data=arg))
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < timeout:
            if self._floor == target:
                self._clear_costmaps()
                return True, f"reached {target}"
            self.stairs_cmd.publish(String(data=arg))  # re-assert against subscription races
            time.sleep(0.1)
        return False, f"did not reach {target} within {timeout:.0f}s"

    def _open_door(self, door_id, timeout=20.0):
        """Publish the open command and BLOCK until /door_states confirms the door is open.

        Same busy-poll pattern as _nav_to (safe under ReentrantCallbackGroup): the executor keeps
        servicing /door_states while we wait. Idempotent: re-asserting an already-open door is fine.
        """
        door_id = door_id.strip()
        if door_id in self._opened_doors:
            return True, f"door {door_id} already open"
        self.door_cmd.publish(String(data=door_id))
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < timeout:
            if door_id in self._opened_doors:
                return True, f"door {door_id} opened"
            self.door_cmd.publish(String(data=door_id))  # re-assert against subscription races
            time.sleep(0.1)
        return False, f"door {door_id} did not open within {timeout:.0f}s"

    def _robot_xy(self):
        try:
            t = self.tf_buffer.lookup_transform("odom", "base_link", rclpy.time.Time())
            return (t.transform.translation.x, t.transform.translation.y)
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

    def _centroid(self, loc):
        if self.wm is None:
            return None
        for l, c in zip(self.wm.locations, self.wm.centroids):
            if l == loc:
                return (c.x, c.y)
        return None

    def _nav_to(self, x, y):
        """Blocking NavigateToPose (safe under MultiThreadedExecutor + ReentrantCallbackGroup)."""
        if not self.nav.wait_for_server(timeout_sec=8.0):
            return False, "nav2 navigate_to_pose unavailable"
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.frame
        # stamp ZERO = "latest transform" — see frontier_explorer._go: a now() stamp makes
        # bt_navigator's map->odom goal transform race slam's stamp and abort the goal.
        goal.pose.header.stamp = rclpy.time.Time().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.w = 1.0
        sf = self.nav.send_goal_async(goal)
        while rclpy.ok() and not sf.done():
            time.sleep(0.05)
        gh = sf.result()
        if gh is None or not gh.accepted:
            return False, "nav goal rejected"
        rf = gh.get_result_async()
        while rclpy.ok() and not rf.done():
            time.sleep(0.05)
        status = rf.result().status
        ok = status == GoalStatus.STATUS_SUCCEEDED
        return ok, ("reached" if ok else f"nav ended status {status}")

    def _execute(self, goal_handle):
        g = goal_handle.request
        skill, target = g.skill, g.target
        self.get_logger().info(f"skill: {skill}({target})")
        res = Skill.Result()

        if skill == "go_to_location":
            xy = self._centroid(target)
            if xy is None:
                res.success, res.message = False, f"unknown location {target}"
            else:
                res.success, res.message = self._nav_to(*xy)

        elif skill == "explore":
            if self.grid is None:
                res.success, res.message = False, "no costmap yet"
            else:
                info = self.grid.info
                cents = find_frontiers(self.grid.data, info.width, info.height, info.resolution,
                                       info.origin.position.x, info.origin.position.y)
                rxy = self._robot_xy()
                min_d = float(self.get_parameter("min_frontier_dist").value)
                if rxy is not None:
                    cents = [c for c in cents
                             if math.hypot(c[0] - rxy[0], c[1] - rxy[1]) > min_d]
                if not cents:
                    res.success, res.message = False, "no frontiers (area explored)"
                else:
                    hint = self._centroid(target) if target else None
                    if hint is not None:
                        # goal-directed sensing: the executive passes the location cell of a
                        # known-but-unreachable victim; head for the frontier CLOSEST to it
                        best = min(cents, key=lambda c: math.hypot(c[0] - hint[0], c[1] - hint[1]))
                    elif rxy is None:
                        best = max(cents, key=lambda c: c[2])
                    else:  # biggest-and-nearest, same ranking as frontier_explorer
                        best = max(cents, key=lambda c: c[2]
                                   / (1.0 + math.hypot(c[0] - rxy[0], c[1] - rxy[1])))
                    ok, msg = self._nav_to(best[0], best[1])
                    res.success, res.message = ok, f"explore frontier: {msg}"

        elif skill == "observe":
            t0 = time.time()
            while rclpy.ok() and time.time() - t0 < self.observe_dwell:
                time.sleep(0.1)
            res.success, res.message = True, f"observed {self.n_victims} victim(s)"

        elif skill == "report":
            self.get_logger().warn(f"*** REPORTED victim {target} ***")
            res.success, res.message = True, f"reported {target}"

        elif skill == "open_door":
            res.success, res.message = self._open_door(target)

        elif skill == "climb_stairs":
            res.success, res.message = self._climb_stairs(target)

        else:
            res.success, res.message = False, f"unknown skill '{skill}'"

        if res.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        self.get_logger().info(f"skill {skill} -> success={res.success} ({res.message})")
        return res


def main(args=None):
    rclpy.init(args=args)
    node = SkillServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
