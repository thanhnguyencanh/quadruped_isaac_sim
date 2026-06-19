"""Task Executive — Phase 6: the SENSE -> GROUND -> PLAN -> ACT -> MONITOR -> REPLAN loop.

Each cycle:
  * SENSE/GROUND : read the latest symbolic WorldModel on /world_model (from world_model_node).
  * PLAN         : build a PDDL problem (planner.problem_pddl_from_worldmodel) over the UNREPORTED,
                   non-blocked victims and solve it with Fast Downward (planner.solve).
  * ACT          : dispatch the FIRST plan action as a /skill action (spot_sar_msgs/action/Skill):
                       move    -> go_to_location(<dest>)   (Nav2 NavigateToPose, via skill_server)
                       explore -> observe                  (dwell + sense the current cell)
                       detect  -> observe
                       report  -> report(<victim>)
  * MONITOR      : on the skill RESULT (success/failure): a successful `report` marks the victim
                   reported; a skill that fails repeatedly blocks that victim (unreachable) so the
                   mission can still terminate.
  * REPLAN       : the next cycle re-reads /world_model and re-solves — newly explored locations /
                   newly detected victims are handled automatically (partial observability).

Runs in the planning venv (unified_planning) under a MultiThreadedExecutor (so the Skill action
client can run concurrently with the planning timer):
  source /opt/ros/jazzy/setup.bash && source ~/unige_ws/install/setup.bash
  source ~/sar_planning_venv/bin/activate
  python <install>/lib/spot_sar_executive/task_executive --ros-args -p use_sim_time:=true

-p dry_run:=true logs decisions WITHOUT sending skill goals (planning-loop test without the stack).
parse_action() is module-level for unit testing.
"""
import os
import tempfile

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from spot_sar_msgs.action import Skill
from spot_sar_msgs.msg import WorldModel

from spot_sar_planning.planner import default_pddl_dir, problem_pddl_from_worldmodel, solve

# PDDL action name -> (skill name, which arg is the target: 'loc-last' | 'victim-first' | None)
ACTION_TO_SKILL = {
    "move": ("go_to_location", "loc-last"),
    "explore": ("observe", None),
    "detect": ("observe", None),
    "report": ("report", "victim-first"),
}
MAX_ACTION_FAILS = 3


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
        self.reported = set()      # victim ids already reported
        self.blocked = set()       # victim ids we gave up on (unreachable)
        self.busy = False
        self.last_action = None
        self.fail_count = 0

        self.cb = ReentrantCallbackGroup()
        self.skill = ActionClient(self, Skill, "skill", callback_group=self.cb)
        self.create_subscription(WorldModel, "/world_model", self._on_wm, 10, callback_group=self.cb)
        self.create_timer(float(self.get_parameter("cycle_period").value), self._cycle, callback_group=self.cb)
        self.get_logger().info(f"task_executive up (dry_run={self.dry_run}); domain={self.domain}")

    def _on_wm(self, msg: WorldModel):
        self.wm = msg

    def _centroid(self, loc_id):
        for l, c in zip(self.wm.locations, self.wm.centroids):
            if l == loc_id:
                return (c.x, c.y)
        return None

    # ---------------- PLAN ----------------
    def _cycle(self):
        if self.busy or self.wm is None:
            return
        loc_by_id = {v.id: self.wm.victim_location[i] for i, v in enumerate(self.wm.victims)}
        vids = [v.id for v in self.wm.victims if v.id not in self.reported and v.id not in self.blocked]
        if not vids:
            self.get_logger().info("MISSION: all known victims reported (or none yet).",
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
            # no plan for the remaining victims in the current (partial) map -> keep sensing.
            self.get_logger().info("no plan (victims unreachable in current map) - keep exploring.",
                                   throttle_duration_sec=10.0)
            return
        self.get_logger().info(f"PLAN ({len(plan)} actions); next: {plan[0]}")
        self._dispatch(plan[0])

    # ---------------- ACT ----------------
    def _dispatch(self, action):
        name, args = parse_action(action)
        loc_map = {_san(l).lower(): l for l in self.wm.locations}
        mapping = ACTION_TO_SKILL.get(name)
        if mapping is None:
            self.get_logger().warn(f"no skill mapping for action {name}")
            return
        skill_name, targ_kind = mapping
        target = ""
        if targ_kind == "loc-last":
            target = loc_map.get(args[-1], args[-1])
            if self._centroid(target) is None:
                self.get_logger().warn(f"no centroid for {target}; skipping")
                return
        elif targ_kind == "victim-first":
            target = args[0].lstrip("vV")

        self.last_dispatched = (name, args)
        if self.dry_run:
            self.get_logger().info(f"[dry_run] would dispatch skill {skill_name}({target}) for {action}")
            return
        if not self.skill.server_is_ready():
            self.get_logger().warn("/skill server not ready (is skill_server up?)", throttle_duration_sec=5.0)
            return
        goal = Skill.Goal()
        goal.skill = skill_name
        goal.target = target
        self.busy = True
        self.get_logger().info(f"ACT: skill {skill_name}({target})  [from {action}]")
        self.skill.send_goal_async(goal).add_done_callback(
            lambda fut, a=action: self._on_goal_resp(fut, a))

    def _on_goal_resp(self, fut, action):
        gh = fut.result()
        if gh is None or not gh.accepted:
            self.get_logger().warn(f"skill goal rejected for {action}")
            self.busy = False
            return
        gh.get_result_async().add_done_callback(lambda f, a=action: self._on_result(f, a))

    # ---------------- MONITOR ----------------
    def _on_result(self, fut, action):
        result = fut.result().result
        name, args = parse_action(action)
        ok = bool(result.success)
        self.get_logger().info(f"MONITOR: {action} -> success={ok} ({result.message})")
        if ok:
            self.fail_count = 0
            if name == "report":
                self.reported.add(int(args[0].lstrip("vV")))
                self.get_logger().info(f"*** victim {args[0]} REPORTED — mission progress ***")
        else:
            # same action failing repeatedly -> block its victim so the mission can terminate.
            if self.last_action == action:
                self.fail_count += 1
            else:
                self.fail_count = 1
            self.last_action = action
            if self.fail_count >= MAX_ACTION_FAILS:
                vid = self._victim_in_action(args)
                if vid is not None:
                    self.blocked.add(vid)
                    self.get_logger().warn(f"giving up on victim {vid} after {self.fail_count} failed '{name}'")
                self.fail_count = 0
        self.busy = False  # -> REPLAN next cycle

    def _victim_in_action(self, args):
        for a in args:
            if a and a[0] in "vV" and a[1:].isdigit():
                return int(a[1:])
        return None


def main(args=None):
    rclpy.init(args=args)
    node = TaskExecutive()
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
