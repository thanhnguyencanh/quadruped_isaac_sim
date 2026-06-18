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
    """Locate the pddl/ dir, in the source tree or the installed share dir."""
    here = os.path.dirname(os.path.abspath(__file__))
    for cand in (
        os.path.join(here, "..", "pddl"),                       # source checkout
        os.path.join(here, "..", "..", "..", "..", "share", "spot_sar_planning", "pddl"),
    ):
        if os.path.isdir(cand):
            return os.path.abspath(cand)
    return os.path.abspath(os.path.join(here, "..", "pddl"))


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
                                 victim_locations, explored, problem_name="sar") -> str:
    """Build a problem.pddl string from the grounded world model.

    locations          : list of symbolic location ids (e.g. "L_1_0")
    edges              : list of (a, b) adjacency pairs (undirected; emitted both ways)
    robot_location     : current robot location id ("" if unknown)
    victim_ids         : list of int victim ids
    victim_locations   : list of location ids, parallel to victim_ids
    explored           : list of explored location ids
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
