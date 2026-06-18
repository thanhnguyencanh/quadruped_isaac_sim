"""Task Executive — Phase 6: the SENSE -> GROUND -> PLAN -> ACT -> MONITOR -> REPLAN loop.

Each cycle:
  * SENSE/GROUND: read the latest symbolic WorldModel on /world_model (from world_model_node).
  * PLAN: build a PDDL problem (planner.problem_pddl_from_worldmodel) over the UNREPORTED
    victims and solve it with Fast Downward (planner.solve).
  * ACT: dispatch the FIRST plan action as a skill:
      - move / explore -> Nav2 NavigateToPose to the target location's centroid
      - detect          -> no-op (perception runs continuously; the victim is already known)
      - report          -> mark the victim reported (and log it)
  * MONITOR/REPLAN: when the skill finishes, the next cycle re-reads the world model and replans
    — so newly explored locations / newly detected victims are handled automatically.

Runs in the planning venv (needs unified_planning) with ROS + the workspace sourced:
  source /opt/ros/jazzy/setup.bash && source ~/unige_ws/install/setup.bash
  source ~/sar_planning_venv/bin/activate
  ros2 run spot_sar_executive task_executive --ros-args -p use_sim_time:=true

Use -p dry_run:=true to log decisions WITHOUT sending Nav2 goals (planning-loop test without Nav2).
parse_action() is module-level for unit testing.
"""
import os
import tempfile

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from nav2_msgs.action import NavigateToPose
from spot_sar_msgs.msg import WorldModel

from spot_sar_planning.planner import default_pddl_dir, problem_pddl_from_worldmodel, solve


def _san(loc: str) -> str:
    return loc.replace("-", "n")


def parse_action(action: str):
    """'move(l_0_0, l_1_0)' -> ('move', ['l_0_0', 'l_1_0'])."""
    name = action[: action.index("(")].strip()
    inner = action[action.index("(") + 1 : action.rindex(")")]
    args = [a.strip() for a in inner.split(",") if a.strip()]
    return name, args


class TaskExecutive(Node):
    def __init__(self):
        super().__init__("task_executive")
        self.declare_parameter("planner_frame", "map")
        self.declare_parameter("cycle_period", 3.0)
        self.declare_parameter("dry_run", False)
        self.frame = self.get_parameter("planner_frame").value
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.domain = os.path.join(default_pddl_dir(), "domain.pddl")

        self.wm = None
        self.reported = set()
        self.busy = False
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.create_subscription(WorldModel, "/world_model", self._on_wm, 10)
        self.create_timer(float(self.get_parameter("cycle_period").value), self._cycle)
        self.get_logger().info(f"task_executive up (dry_run={self.dry_run}); domain={self.domain}")

    def _on_wm(self, msg: WorldModel):
        self.wm = msg

    def _centroid(self, loc_id):
        for l, c in zip(self.wm.locations, self.wm.centroids):
            if l == loc_id:
                return (c.x, c.y)
        return None

    def _cycle(self):
        if self.busy or self.wm is None:
            return
        loc_by_id = {v.id: self.wm.victim_location[i] for i, v in enumerate(self.wm.victims)}
        vids = [v.id for v in self.wm.victims if v.id not in self.reported]
        if not vids:
            self.get_logger().info("mission: all known victims reported (or none yet).",
                                   throttle_duration_sec=10.0)
            return
        vlocs = [loc_by_id[i] for i in vids]
        edges = list(zip(self.wm.connected_a, self.wm.connected_b))
        prob = problem_pddl_from_worldmodel(self.wm.locations, edges, self.wm.robot_location,
                                            vids, vlocs, self.wm.explored)
        pf = os.path.join(tempfile.gettempdir(), "sar_problem.pddl")
        with open(pf, "w") as fh:
            fh.write(prob)
        try:
            plan = solve(self.domain, pf)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"planner error: {e}")
            return
        if not plan:
            self.get_logger().info("no plan (victims unreachable in current map) — keep exploring.",
                                   throttle_duration_sec=10.0)
            return
        self.get_logger().info(f"PLAN ({len(plan)} actions); next: {plan[0]}")
        self._dispatch(plan[0])

    def _dispatch(self, action):
        name, args = parse_action(action)
        loc_map = {_san(l).lower(): l for l in self.wm.locations}
        if name in ("move", "explore"):
            target = loc_map.get(args[-1], args[-1])
            xy = self._centroid(target)
            if xy is None:
                self.get_logger().warn(f"no centroid for {target}; skipping")
                return
            self._navigate(xy, label=f"{name}->{target}")
        elif name == "detect":
            self.get_logger().info(f"detect: {args[0]} (perception continuous) — replanning")
        elif name == "report":
            vid = int(args[0].lstrip("vV"))
            self.reported.add(vid)
            self.get_logger().info(f"*** REPORTED victim {vid} at {loc_map.get(args[-1], args[-1])} ***")
        else:
            self.get_logger().warn(f"unknown action {name}")

    def _navigate(self, xy, label):
        if self.dry_run:
            self.get_logger().info(f"[dry_run] {label}: would NavigateToPose ({xy[0]:.2f}, {xy[1]:.2f})")
            return
        if not self.nav.server_is_ready():
            self.get_logger().warn("navigate_to_pose not ready (is Nav2 up?)", throttle_duration_sec=5.0)
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(xy[0])
        goal.pose.pose.position.y = float(xy[1])
        goal.pose.pose.orientation.w = 1.0
        self.busy = True
        self.get_logger().info(f"{label}: NavigateToPose ({xy[0]:.2f}, {xy[1]:.2f})")
        self.nav.send_goal_async(goal).add_done_callback(self._on_goal_resp)

    def _on_goal_resp(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self.busy = False
            return
        gh.get_result_async().add_done_callback(lambda _f: setattr(self, "busy", False))


def main(args=None):
    rclpy.init(args=args)
    node = TaskExecutive()
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
