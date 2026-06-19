"""Skill server — Phase 4: dispatchable SAR skills as a ROS 2 action.

Hosts the /skill action (spot_sar_msgs/action/Skill); the Task Executive (Phase 6) sends one
Skill goal per PDDL action and branches on success/failure to replan. Skills:

  go_to_location  -> Nav2 NavigateToPose to the target location's centroid (from /world_model)
  explore         -> drive to the nearest/biggest /map frontier (NavigateToPose)
  observe         -> dwell and report how many victims are currently seen on /victims
  report          -> mark a victim reported (logged; the executive tracks reported state)

Runs under a MultiThreadedExecutor with a ReentrantCallbackGroup so the action server can call
the Nav2 action client and block on its result without deadlocking.

  ros2 run spot_sar_nav skill_server --ros-args -p use_sim_time:=true -p planner_frame:=map
"""
import time

import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid

from spot_sar_msgs.action import Skill
from spot_sar_msgs.msg import VictimArray, WorldModel
from spot_sar_nav.frontier_explorer import find_frontiers


class SkillServer(Node):
    def __init__(self):
        super().__init__("skill_server")
        self.declare_parameter("planner_frame", "map")
        self.declare_parameter("observe_dwell", 3.0)
        self.frame = self.get_parameter("planner_frame").value
        self.observe_dwell = float(self.get_parameter("observe_dwell").value)

        self.cb = ReentrantCallbackGroup()
        self.wm = None
        self.grid = None
        self.n_victims = 0

        self.create_subscription(WorldModel, "/world_model", self._on_wm, 10, callback_group=self.cb)
        self.create_subscription(OccupancyGrid, "/map", self._on_map, 1, callback_group=self.cb)
        self.create_subscription(VictimArray, "/victims", self._on_vic, 10, callback_group=self.cb)
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose", callback_group=self.cb)
        self.srv = ActionServer(self, Skill, "skill", execute_callback=self._execute,
                                callback_group=self.cb)
        self.get_logger().info("skill_server up: /skill {go_to_location|explore|observe|report}")

    def _on_wm(self, m):
        self.wm = m

    def _on_map(self, m):
        self.grid = m

    def _on_vic(self, m):
        self.n_victims = len(m.victims)

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
        goal.pose.header.stamp = self.get_clock().now().to_msg()
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
                res.success, res.message = False, "no /map yet"
            else:
                info = self.grid.info
                cents = find_frontiers(self.grid.data, info.width, info.height, info.resolution,
                                       info.origin.position.x, info.origin.position.y)
                if not cents:
                    res.success, res.message = False, "no frontiers (area explored)"
                else:
                    best = max(cents, key=lambda c: c[2])
                    ok, msg = self._nav_to(best[0], best[1])
                    res.success, res.message = ok, f"explore frontier: {msg}"

        elif skill == "observe":
            t0 = time.time()
            while rclpy.ok() and time.time() - t0 < self.observe_dwell:
                time.sleep(0.1)
            res.success, res.message = True, f"observed {self.n_victims} victim(s)"

        elif skill == "report":
            self.get_logger().info(f"*** REPORTED victim {target} ***")
            res.success, res.message = True, f"reported {target}"

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
