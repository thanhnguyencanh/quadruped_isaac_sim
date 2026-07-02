"""PDDL planning for Spot SAR (Phase 5).

Two pieces:
  * solve(domain, problem)            — run a PDDL domain+problem through unified-planning
    (Fast Downward / ENHSP) and return the action sequence.
  * problem_pddl_from_worldmodel(...) — turn the symbolic WorldModel (locations, connectivity,
    victims, robot location) into a problem.pddl whose goal is "report every found victim".

unified_planning lives in the planning venv (~/sar_planning_venv, built with
--system-site-packages so rclpy still imports). Run standalone with that venv:

    source ~/sar_planning_venv/bin/activate
    python -m spot_sar_planning.planner            # solves pddl/problem_example.pddl

The executive (Phase 6) calls problem_pddl_from_worldmodel() each cycle and re-solves —
closing the SENSE -> GROUND -> PLAN -> ACT -> MONITOR -> REPLAN loop.
"""
import os


def default_pddl_dir() -> str:
    """Locate the pddl/ dir: installed share dir (preferred) or the source tree."""
    try:
        from ament_index_python.packages import get_package_share_directory
        cand = os.path.join(get_package_share_directory("spot_sar_planning"), "pddl")
        if os.path.isdir(cand):
            return cand
    except Exception:  # noqa: BLE001  (ament not available / package not installed)
        pass
    here = os.path.dirname(os.path.abspath(__file__))
    cand = os.path.join(here, "..", "pddl")            # source checkout
    return os.path.abspath(cand)


def solve(domain_path: str, problem_path: str):
    """Return the plan as a list of action strings (empty list if unsolvable)."""
    from unified_planning.io import PDDLReader
    from unified_planning.shortcuts import OneshotPlanner

    problem = PDDLReader().parse_problem(domain_path, problem_path)
    with OneshotPlanner(problem_kind=problem.kind) as planner:
        result = planner.solve(problem)
    plan = result.plan
    if plan is None:
        return []
    return [str(a) for a in plan.actions]


def problem_pddl_from_worldmodel(locations, edges, robot_location, victim_ids,
                                 victim_locations, explored, found_ids=(), problem_name="sar") -> str:
    """Build a problem.pddl string from the grounded world model.

    locations          : list of symbolic location ids (e.g. "L_1_0")
    edges              : list of (a, b) adjacency pairs (undirected; emitted both ways)
    robot_location     : current robot location id ("" if unknown)
    victim_ids         : list of int victim ids
    victim_locations   : list of location ids, parallel to victim_ids
    explored           : list of explored location ids
    found_ids          : victim ids already detected (executive-tracked) -> (found v) in init, so
                         the plan advances past `detect` to `report` instead of looping on detect.
    Goal: report every victim.
    """
    objs_loc = " ".join(_san(l) for l in locations) or "L_none"
    objs_vic = " ".join(f"v{i}" for i in victim_ids)
    init = []
    if robot_location:
        init.append(f"(at {_san(robot_location)})")
    for a, b in edges:
        init.append(f"(connected {_san(a)} {_san(b)})")
        init.append(f"(connected {_san(b)} {_san(a)})")
    for l in explored:
        init.append(f"(explored {_san(l)})")
    for vid, vloc in zip(victim_ids, victim_locations):
        init.append(f"(victim-at v{vid} {_san(vloc)})")
    for vid in found_ids:
        if vid in victim_ids:
            init.append(f"(found v{vid})")
    goal = " ".join(f"(reported v{vid})" for vid in victim_ids) or "(= 0 0)"

    obj_line = f"    {objs_loc} - location"
    if objs_vic:
        obj_line += f"\n    {objs_vic} - victim"
    return (
        f"(define (problem {problem_name})\n"
        f"  (:domain spot-sar)\n"
        f"  (:objects\n{obj_line})\n"
        f"  (:init\n    " + "\n    ".join(init) + ")\n"
        f"  (:goal (and {goal})))\n"
    )


def problem_pddl_from_worldmodel_doors(rooms, doors, robot_location, victim_ids, victim_rooms,
                                       open_door_ids=(), explored=None, found_ids=(),
                                       problem_name="sar_floor") -> str:
    """Build a problem.pddl for the room-graph DOORS domain (domain_doors.pddl).

    rooms          : list of room ids (e.g. "room_a")
    doors          : list of (door_id, room_a, room_b) tuples
    robot_location : current room id of the robot ("" if unknown)
    victim_ids     : list of int victim ids
    victim_rooms   : list of room ids, parallel to victim_ids
    open_door_ids  : door ids currently OPEN (door-open in init); the rest get door-closed
    explored       : room ids flagged explored (default: all rooms — the static floor is known)
    found_ids      : victim ids already detected (executive-tracked) -> (found v) in init
    Goal: report every victim.

    CRITICAL: door-between is emitted in BOTH room orderings (verified: a one-ordering emit makes
    reverse traversal unsolvable).
    """
    if explored is None:
        explored = list(rooms)  # rooms are known up front on the static floor
    open_set = set(open_door_ids)
    objs_room = " ".join(_san(r) for r in rooms) or "R_none"
    objs_door = " ".join(_san(d[0]) for d in doors)
    objs_vic = " ".join(f"v{i}" for i in victim_ids)

    init = []
    if robot_location:
        init.append(f"(at {_san(robot_location)})")
    for did, a, b in doors:
        init.append(f"(door-between {_san(did)} {_san(a)} {_san(b)})")
        init.append(f"(door-between {_san(did)} {_san(b)} {_san(a)})")  # both orderings (mandatory)
        init.append(f"(door-open {_san(did)})" if did in open_set else f"(door-closed {_san(did)})")
    for r in explored:
        init.append(f"(explored {_san(r)})")
    for vid, vroom in zip(victim_ids, victim_rooms):
        init.append(f"(victim-at v{vid} {_san(vroom)})")
    for vid in found_ids:
        if vid in victim_ids:
            init.append(f"(found v{vid})")
    goal = " ".join(f"(reported v{vid})" for vid in victim_ids) or "(= 0 0)"

    obj_line = f"    {objs_room} - location"
    if objs_door:
        obj_line += f"\n    {objs_door} - door"
    if objs_vic:
        obj_line += f"\n    {objs_vic} - victim"
    return (
        f"(define (problem {problem_name})\n"
        f"  (:domain spot-sar-doors)\n"
        f"  (:objects\n{obj_line})\n"
        f"  (:init\n    " + "\n    ".join(init) + ")\n"
        f"  (:goal (and {goal})))\n"
    )


def problem_pddl_from_worldmodel_building(rooms, doors, robot_location, victim_ids, victim_rooms,
                                          stairs, open_door_ids=(), explored=None, found_ids=(),
                                          problem_name="sar_building") -> str:
    """Build a problem.pddl for the TWO-FLOOR building domain (domain_building.pddl).

    Superset of problem_pddl_from_worldmodel_doors: same room-graph + door gating PLUS a STAIR type
    linking the two floor landings. The stair is the only edge between the (x-disjoint) floor wings,
    so the planner is forced to use-stairs to reach the far floor.

    rooms          : list of room ids (e.g. "f1_a", "f2_c", "f1_stair", "f2_stair")
    doors          : list of (door_id, room_a, room_b) — includes always-open passages AND closeable doors
    robot_location : current room id of the robot ("" if unknown)
    victim_ids     : list of int victim ids
    victim_rooms   : list of room ids, parallel to victim_ids
    stairs         : list of (stair_id, landing_a, landing_b) — the inter-floor links
    open_door_ids  : door/passage ids currently OPEN (door-open in init); the rest get door-closed
    explored       : room ids flagged explored (default: all rooms — the static building is known)
    found_ids      : victim ids already detected (executive-tracked) -> (found v) in init

    CRITICAL: door-between AND stair-between are each emitted in BOTH orderings (a one-ordering emit
    makes reverse traversal unsolvable — the same lesson as the doors domain).
    """
    if explored is None:
        explored = list(rooms)
    open_set = set(open_door_ids)
    objs_room = " ".join(_san(r) for r in rooms) or "R_none"
    objs_door = " ".join(_san(d[0]) for d in doors)
    objs_stair = " ".join(_san(s[0]) for s in stairs)
    objs_vic = " ".join(f"v{i}" for i in victim_ids)

    init = []
    if robot_location:
        init.append(f"(at {_san(robot_location)})")
    for did, a, b in doors:
        init.append(f"(door-between {_san(did)} {_san(a)} {_san(b)})")
        init.append(f"(door-between {_san(did)} {_san(b)} {_san(a)})")  # both orderings (mandatory)
        init.append(f"(door-open {_san(did)})" if did in open_set else f"(door-closed {_san(did)})")
    for sid, a, b in stairs:
        init.append(f"(stair-between {_san(sid)} {_san(a)} {_san(b)})")
        init.append(f"(stair-between {_san(sid)} {_san(b)} {_san(a)})")  # both orderings (mandatory)
    for r in explored:
        init.append(f"(explored {_san(r)})")
    for vid, vroom in zip(victim_ids, victim_rooms):
        init.append(f"(victim-at v{vid} {_san(vroom)})")
    for vid in found_ids:
        if vid in victim_ids:
            init.append(f"(found v{vid})")
    goal = " ".join(f"(reported v{vid})" for vid in victim_ids) or "(= 0 0)"

    obj_line = f"    {objs_room} - location"
    if objs_door:
        obj_line += f"\n    {objs_door} - door"
    if objs_stair:
        obj_line += f"\n    {objs_stair} - stair"
    if objs_vic:
        obj_line += f"\n    {objs_vic} - victim"
    return (
        f"(define (problem {problem_name})\n"
        f"  (:domain spot-sar-building)\n"
        f"  (:objects\n{obj_line})\n"
        f"  (:init\n    " + "\n    ".join(init) + ")\n"
        f"  (:goal (and {goal})))\n"
    )


def _san(loc: str) -> str:
    """PDDL identifiers can't contain '-'; cells like L_-1_2 -> L_n1_2."""
    return loc.replace("-", "n")


def main():
    pddl = default_pddl_dir()
    domain = os.path.join(pddl, "domain.pddl")
    problem = os.path.join(pddl, "problem_example.pddl")
    print(f"[planner] domain={domain}")
    print(f"[planner] problem={problem}")
    plan = solve(domain, problem)
    if plan:
        print(f"[planner] PLAN ({len(plan)} actions):")
        for i, a in enumerate(plan):
            print(f"  {i + 1}. {a}")
    else:
        print("[planner] no plan found")


if __name__ == "__main__":
    main()
