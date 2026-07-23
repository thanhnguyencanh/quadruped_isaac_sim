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
import math
import os
import tempfile

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from spot_sar_msgs.action import Skill
from spot_sar_msgs.msg import WorldModel

from spot_sar_planning import sar_building
from spot_sar_planning.planner import (default_pddl_dir, problem_pddl_from_worldmodel,
                                        problem_pddl_from_worldmodel_building,
                                        problem_pddl_from_worldmodel_doors, solve)

# PDDL action name -> (skill name, which arg is the target: 'loc-last' | 'victim-first' | None)
# GRID profile (the original cell-grid mission, domain.pddl):
ACTION_TO_SKILL = {
    "move": ("go_to_location", "loc-last"),
    "explore": ("observe", None),
    "detect": ("observe", None),
    "report": ("report", "victim-first"),
}
# DOORS profile (room-graph floor demo, domain_doors.pddl). `move` is move(from,to,door) so the
# nav destination is the 2nd-from-last arg; open-door's target is the door id (first arg).
ACTION_TO_SKILL_DOORS = {
    "move": ("go_to_location", "loc-second-last"),
    "open-door": ("open_door", "door-first"),
    "explore": ("observe", None),
    "detect": ("observe", None),
    "report": ("report", "victim-first"),
}
# BUILDING profile (two-floor building, domain_building.pddl): doors gate intra-floor moves + a
# stair changes floors. use-stairs(?s ?from ?to) -> climb_stairs("<stair_id> up|down"); direction is
# computed from the destination landing's floor ('stair-dir').
ACTION_TO_SKILL_BUILDING = {
    **ACTION_TO_SKILL_DOORS,
    "use-stairs": ("climb_stairs", "stair-dir"),
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
        self.declare_parameter("planner_frame", "odom")  # nav + world model live in odom
        self.declare_parameter("cycle_period", 3.0)
        self.declare_parameter("dry_run", False)
        # "grid" (cell mission) | "doors" (floor demo) | "building" (two-floor stairs demo)
        self.declare_parameter("domain_profile", "grid")
        self.frame = self.get_parameter("planner_frame").value
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.profile = self.get_parameter("domain_profile").value
        if self.profile == "building":
            self.domain = os.path.join(default_pddl_dir(), "domain_building.pddl")
            self.action_to_skill = ACTION_TO_SKILL_BUILDING
        elif self.profile == "doors":
            self.domain = os.path.join(default_pddl_dir(), "domain_doors.pddl")
            self.action_to_skill = ACTION_TO_SKILL_DOORS
        else:
            self.domain = os.path.join(default_pddl_dir(), "domain.pddl")
            self.action_to_skill = ACTION_TO_SKILL

        self.wm = None
        self.found = set()         # victim ids whose `detect` succeeded (executive-tracked state)
        self.reported = set()      # victim ids already reported
        self.blocked = set()       # victim ids we gave up on (unreachable)
        self.busy = False
        self.last_action = None
        self.fail_count = 0
        self.explore_done = False  # explore skill reported "no frontiers" -> area fully seen
        self._goto_fail = {}       # victim-cell -> failed walk-toward attempts (fallback gating)
        self.current_action = "idle"  # what is in flight right now (for the [STATUS] line)

        self.cb = ReentrantCallbackGroup()
        self.skill = ActionClient(self, Skill, "skill", callback_group=self.cb)
        self.create_subscription(WorldModel, "/world_model", self._on_wm, 10, callback_group=self.cb)
        self.create_timer(float(self.get_parameter("cycle_period").value), self._cycle, callback_group=self.cb)
        # one-line mission dashboard for the launch terminal — the nav2/slam stream buries
        # individual events, so the CURRENT state must be re-announced periodically.
        self.create_timer(8.0, self._status, callback_group=self.cb)
        self.get_logger().info(
            f"task_executive up (dry_run={self.dry_run}, profile={self.profile}); domain={self.domain}")

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
            # No unreported victims KNOWN is not mission-done — victims may simply not have
            # been SEEN yet (they spawn beyond the first camera view). Keep the SENSE loop
            # moving: dispatch the explore skill (one costmap-frontier goal, blocks until nav
            # finishes) until it reports the area fully explored. Idling here deadlocked the
            # whole mission: robot never moves -> camera never sees -> world model never grows.
            if self.explore_done:
                self.get_logger().warn(
                    f"MISSION COMPLETE: area fully explored, {len(self.reported)} victim(s) reported.",
                    throttle_duration_sec=30.0)
                return
            self._dispatch_sensing("explore")
            return
        # Plan ONLY over victims whose cells are symbolically REACHABLE (BFS from the
        # robot's cell over connected edges). The goal is a conjunction: one victim in a
        # disconnected cell makes the WHOLE problem unsolvable — observed: the robot stood
        # ON v0 while v1's edgeless cell blocked every plan. Unreachable victims drive
        # SENSING (walk-toward / explore) below until their cells connect.
        adj = {}
        for a, b in zip(self.wm.connected_a, self.wm.connected_b):
            adj.setdefault(a, set()).add(b)
            adj.setdefault(b, set()).add(a)
        reach = set()
        stack = [self.wm.robot_location] if self.wm.robot_location else []
        while stack:
            n = stack.pop()
            if n in reach:
                continue
            reach.add(n)
            stack.extend(adj.get(n, ()))
        unreachable = [i for i in vids if loc_by_id[i] not in reach]
        vids = [i for i in vids if loc_by_id[i] in reach]
        if not vids:
            self._sense_toward(unreachable, loc_by_id)
            return
        vlocs = [loc_by_id[i] for i in vids]
        found_here = [v for v in vids if v in self.found]
        if self.profile == "building":
            doors = list(zip(self.wm.doors, self.wm.door_room_a, self.wm.door_room_b))
            open_ids = [d for d, o in zip(self.wm.doors, self.wm.door_open) if o]
            stairs = [(s.id, s.room_a, s.room_b) for s in sar_building.STAIRS]  # static topology
            prob = problem_pddl_from_worldmodel_building(
                list(self.wm.locations), doors, self.wm.robot_location, vids, vlocs, stairs,
                open_door_ids=open_ids, explored=list(self.wm.explored), found_ids=found_here)
        elif self.profile == "doors":
            doors = list(zip(self.wm.doors, self.wm.door_room_a, self.wm.door_room_b))
            open_ids = [d for d, o in zip(self.wm.doors, self.wm.door_open) if o]
            prob = problem_pddl_from_worldmodel_doors(
                list(self.wm.locations), doors, self.wm.robot_location, vids, vlocs,
                open_door_ids=open_ids, explored=list(self.wm.explored), found_ids=found_here)
        else:
            edges = list(zip(self.wm.connected_a, self.wm.connected_b))
            prob = problem_pddl_from_worldmodel(self.wm.locations, edges, self.wm.robot_location,
                                                vids, vlocs, self.wm.explored, found_ids=found_here)
        pf = os.path.join(tempfile.gettempdir(), "sar_problem.pddl")
        with open(pf, "w") as fh:
            fh.write(prob)
        try:
            plan = solve(self.domain, pf)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"planner error: {e}")
            return
        if not plan:
            # No symbolic path to any remaining victim in the CURRENT visited-cell graph —
            # the victim's cell only connects once the robot has walked near it. Exploration
            # is what grows that graph, so DISPATCH it (this branch used to log "keep
            # exploring" while nobody actually explored: victim known -> no plan -> idle).
            # Reachable victims but still no plan: domain-level infeasibility (should be
            # rare). Fall back to sensing toward them rather than idling.
            self._sense_toward(vids, loc_by_id)
            return
        self.get_logger().warn(f"PLAN ({len(plan)} actions); next: {plan[0]}")
        self._dispatch(plan[0])

    # ---------------- STATUS (terminal dashboard) ----------------
    def _status(self):
        if self.wm is None:
            self.get_logger().info("[STATUS] waiting for /world_model …")
            return
        known = len(self.wm.victims)
        unreported = [v.id for v in self.wm.victims
                      if v.id not in self.reported and v.id not in self.blocked]
        if self.explore_done and not unreported:
            phase = "COMPLETE"
        elif unreported:
            phase = "RESCUE"
        else:
            phase = "EXPLORING"
        self.get_logger().warn(  # WARN = yellow console line: easy to spot (user request)
            f"[STATUS] phase={phase}  action={self.current_action}  "
            f"victims known={known} found={len(self.found)} reported={len(self.reported)}"
            + (f" blocked={len(self.blocked)}" if self.blocked else "")
            + f"  |  locations={len(self.wm.locations)} robot@{self.wm.robot_location or '?'}")

    def _sense_toward(self, victim_ids, loc_by_id):
        """SENSING: connect a known victim's cell to the visited graph. Walk straight toward
        the nearest one (Nav2 plans through costmap-known space; en-route cells get visited);
        after 2 failed walks fall back to victim-hinted frontier exploration; once the area
        is fully explored, block the remaining victims so the mission can terminate.
        Frontier exploration ALONE can never terminate this state: the lidar clears the
        victim's area from afar while the symbolic graph grows only from physical visits."""
        hint, best_d = "", None
        rob = self._centroid(self.wm.robot_location) if self.wm.robot_location else None
        for i in victim_ids:
            c = self._centroid(loc_by_id[i])
            d = math.hypot(c[0] - rob[0], c[1] - rob[1]) if (c and rob) else 0.0
            if best_d is None or d < best_d:
                best_d, hint = d, loc_by_id[i]
        if hint and self._goto_fail.get(hint, 0) < 2:
            self.get_logger().info(
                f"no plan yet — walking toward victim cell {hint} to connect the graph.",
                throttle_duration_sec=10.0)
            self._dispatch_sensing("go_to_location", hint)
        elif not self.explore_done:
            self.get_logger().info(
                f"no plan yet — exploring toward {hint or 'frontier'}.",
                throttle_duration_sec=10.0)
            self._dispatch_sensing("explore", hint)
        else:
            self.blocked.update(victim_ids)
            self.get_logger().warn(
                f"no plan and area fully explored — blocking victim(s) {victim_ids} as unreachable.")

    # ---------------- ACT (sensing: explore / walk-toward-victim) ----------------
    def _dispatch_sensing(self, skill_name: str, target: str = ""):
        if self.dry_run:
            self.get_logger().info(f"[dry_run] would dispatch sensing skill {skill_name}({target})",
                                   throttle_duration_sec=10.0)
            return
        if not self.skill.server_is_ready():
            self.get_logger().warn("/skill server not ready (is skill_server up?)",
                                   throttle_duration_sec=5.0)
            return
        goal = Skill.Goal()
        goal.skill = skill_name
        goal.target = target
        self.busy = True
        self.current_action = f"{skill_name}({target})"
        self.get_logger().info(f"ACT: skill {skill_name}({target})  [sensing]")
        self.skill.send_goal_async(goal).add_done_callback(
            lambda fut: self._on_sensing_resp(fut, skill_name, target))

    def _on_sensing_resp(self, fut, skill_name, target):
        gh = fut.result()
        if gh is None or not gh.accepted:
            self.get_logger().warn(f"sensing skill {skill_name} rejected")
            self.busy = False
            return
        gh.get_result_async().add_done_callback(
            lambda f: self._on_sensing_result(f, skill_name, target))

    def _on_sensing_result(self, fut, skill_name, target):
        result = fut.result().result
        self.busy = False
        self.current_action = "idle"
        self.get_logger().info(
            f"MONITOR: {skill_name}({target}) -> success={result.success} ({result.message})")
        if skill_name == "explore" and not result.success and "no frontiers" in result.message:
            self.explore_done = True
        if skill_name == "go_to_location" and target:
            if result.success:
                self._goto_fail.pop(target, None)
            else:
                self._goto_fail[target] = self._goto_fail.get(target, 0) + 1

    # ---------------- ACT ----------------
    def _dispatch(self, action):
        name, args = parse_action(action)
        loc_map = {_san(l).lower(): l for l in self.wm.locations}
        mapping = self.action_to_skill.get(name)
        if mapping is None:
            self.get_logger().warn(f"no skill mapping for action {name}")
            return
        skill_name, targ_kind = mapping
        target = ""
        if targ_kind in ("loc-last", "loc-second-last"):
            arg = args[-1] if targ_kind == "loc-last" else args[-2]
            target = loc_map.get(arg, arg)
            if self._centroid(target) is None:
                self.get_logger().warn(f"no centroid for {target}; skipping")
                return
        elif targ_kind == "victim-first":
            target = args[0].lstrip("vV")
        elif targ_kind == "door-first":
            target = args[0]  # door id, published verbatim to /door_cmd by the open_door skill
        elif targ_kind == "stair-dir":
            # use-stairs(?s ?from ?to): climb_stairs target = "<stair_id> up|down" (up if ?to on f2)
            stair_id, _to = args[0], args[-1]
            direction = "up" if sar_building.floor_of(_to) == "f2" else "down"
            target = f"{stair_id} {direction}"

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
        self.current_action = f"{skill_name}({target})"
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
        self.current_action = "idle"
        result = fut.result().result
        name, args = parse_action(action)
        ok = bool(result.success)
        self.get_logger().info(f"MONITOR: {action} -> success={ok} ({result.message})")
        if ok:
            self.fail_count = 0
            if name == "detect":
                # persist the detect effect so the planner advances to `report` (no detect loop)
                vid = self._victim_in_action(args)
                if vid is not None:
                    self.found.add(vid)
            if name == "report":
                self.reported.add(int(args[0].lstrip("vV")))
                self.get_logger().warn(f"*** victim {args[0]} REPORTED — mission progress ***")
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
