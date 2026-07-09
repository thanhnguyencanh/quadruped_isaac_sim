"""sar_building — SINGLE SOURCE OF TRUTH for the TWO-FLOOR SAR building (rooms + doors + stairs + victims).

Pure Python, NO pxr / isaacsim / rclpy imports, so every layer imports it cheaply and stays in
sync — the Isaac scene builder (Isaac's bundled python), the building grounding node + planner
(system Jazzy / planning venv). Mirrors sar_floor.py; if the geometry here changes, the sim, the
symbol grounding and the PDDL problem all change together.

ARCHITECTURE (why a two-floor building works on a 2D SLAM/Nav2 stack, per the design panel):
  * The two floors are laid out X-OFFSET so their occupancy footprints are DISJOINT in the single
    2D map (floor-2 rooms live far in +x). A vertically-stacked footprint would project floor-2
    walls onto floor-1 cells and corrupt the one planar /map — the offset avoids that entirely, so
    ONE slam_toolbox + ONE Nav2 costmap serve BOTH floors unchanged.
  * The stair LANDING column is the exception: f1_stair and f2_stair sit at the SAME (x,y)=(10.5,0),
    differing only in z (0.0 vs 3.0). The floor change is therefore a PURE-Z teleport
    (10.5,0,0.8)->(10.5,0,3.8): slam_toolbox's planar map->odom sees ~zero (x,y,yaw) discontinuity,
    so NO map swap / pause / re-seed is needed. This is the load-bearing trick.
  * Spot's flat-terrain policy cannot climb, so the level change is an in-app teleport-assist
    (StairsNode == DoorNode with a z-lerp). A decorative staircase is rendered at the landing purely
    for camera / YOLO / elevation-mapping realism; Spot never drives onto it.

Layout (side view along +x; f2 is offset +x AND lifted +3 m; the landing column is shared):

    z=3  ................................  f2_stair | f2_a | f2_b | f2_c  (VICTIM v1 in f2_c)
                                          (x in [9,15])[15,21][21,27][27,33]
    z=0  f1_a | f1_b | f1_c | f1_stair .................................
        [-9,-3][-3,3][3,9]  [9,15]
              [SPAWN f1_b]  (VICTIM v0 in f1_c)   ^stacked landing column x in [9,15]

The ONLY graph edge between the two wings is the stair (f1_stair <-> f2_stair); STRIPS is forced
to `use-stairs` to reach floor 2, exactly as it is forced to `open-door` before traversal.
"""
from collections import namedtuple

# ---- metric constants (metres) — shared with sar_floor conventions ----
WALL_H = 2.0          # wall / door height
WALL_T = 0.2          # wall + door-slab thickness
ROOM_HX = 3.0         # room half-extent in x  (rooms are 6 m wide)
ROOM_HY = 3.0         # room half-extent in y  (rooms are 6 m deep)
DOOR_W = 1.2          # door / passage opening width (along y)
DOOR_LIFT = 2.3       # how far a closeable slab rises when "open" (portcullis)
ROOM_MARGIN = 0.4     # room_of() shrinks each room so doorways belong to NEITHER room

FLOOR_Z = {"f1": 0.0, "f2": 3.0}   # slab-TOP height per floor (walls span z0..z0+WALL_H)
SPAWN_ROOM = "f1_b"                 # Spot spawns at (0,0,0.8) inside f1_b
SPAWN_XYZ = (0.0, 0.0, 0.8)

# The stair landings (f1_stair, f2_stair) are 6x6 rooms centred at x=12 (span [9,15]); they are
# vertically STACKED (same x,y, different floor). LANDING_XY is where Spot stands / teleports — offset
# WEST of the landing centre so it is clear of the decorative staircase (which fills x[12,15]) and so
# it faces the ascending steps. Both floor slabs cover (10.5,0), so the teleport lands on solid floor.
LANDING_CENTER_X = 12.0
LANDING_XY = (10.5, 0.0)
SPOT_STAND_DZ = 0.8                 # body height above the slab (matches the proven f1 spawn offset)

Room = namedtuple("Room", "id center half floor")   # center=(cx,cy), half=(hx,hy), floor="f1"|"f2"
# Portal: an inter-room link. always_open passages have NO physical slab (just a wall gap); closeable
# doors get a kinematic slab and start CLOSED (forcing an open-door action). pos=(cx,cy) on the divider.
Portal = namedtuple("Portal", "id room_a room_b pos always_open")
Stair = namedtuple("Stair", "id room_a room_b landing_xy")  # room_a on f1, room_b on f2

ROOMS = [
    # floor 1 (z=0): three rooms in a line + the stair landing at the +x end
    Room("f1_a", (-6.0, 0.0), (ROOM_HX, ROOM_HY), "f1"),   # x in [-9,-3]
    Room("f1_b", (0.0, 0.0), (ROOM_HX, ROOM_HY), "f1"),    # x in [-3, 3]  (SPAWN)
    Room("f1_c", (6.0, 0.0), (ROOM_HX, ROOM_HY), "f1"),    # x in [ 3, 9]  (VICTIM v0)
    Room("f1_stair", (LANDING_CENTER_X, 0.0), (ROOM_HX, ROOM_HY), "f1"),  # x in [9,15]  landing (z=0)
    # floor 2 (z=3): the stair landing (stacked at the SAME x,y) + three rooms marching off in +x
    Room("f2_stair", (LANDING_CENTER_X, 0.0), (ROOM_HX, ROOM_HY), "f2"),  # x in [9,15]  landing (z=3, STACKED)
    Room("f2_a", (18.0, 0.0), (ROOM_HX, ROOM_HY), "f2"),   # x in [15,21]
    Room("f2_b", (24.0, 0.0), (ROOM_HX, ROOM_HY), "f2"),   # x in [21,27]
    Room("f2_c", (30.0, 0.0), (ROOM_HX, ROOM_HY), "f2"),   # x in [27,33]  (VICTIM v1)
]

PORTALS = [
    # floor 1: two CLOSEABLE doors (exercise open-door) + one open passage to the landing
    Portal("door_ab", "f1_a", "f1_b", (-3.0, 0.0), False),
    Portal("door_bc", "f1_b", "f1_c", (3.0, 0.0), False),
    Portal("pass_c_s", "f1_c", "f1_stair", (9.0, 0.0), True),
    # floor 2: doorless — open passages only (the door pattern is already proven on floor 1)
    Portal("pass_s_a", "f2_stair", "f2_a", (15.0, 0.0), True),
    Portal("pass_a_b", "f2_a", "f2_b", (21.0, 0.0), True),
    Portal("pass_b_c", "f2_b", "f2_c", (27.0, 0.0), True),
]

STAIRS = [Stair("stair_main", "f1_stair", "f2_stair", LANDING_XY)]

# victims: (x, y, z, room). z is the marker centre height; for a human the FOOT sits on the room's slab.
VICTIMS = [
    (6.0, 1.5, 0.3, "f1_c"),     # floor 1 -> reach requires open-door(door_bc)
    (30.0, -1.5, 3.3, "f2_c"),   # floor 2 -> reach requires use-stairs(stair_main)
]

CLOSEABLE_DOORS = [p for p in PORTALS if not p.always_open]   # get physical slabs + start closed


# --------------------------------------------------------------------- symbolic helpers
def floor_of(room_id):
    """'f1' | 'f2' for a room id (or '' if unknown)."""
    for r in ROOMS:
        if r.id == room_id:
            return r.floor
    return ""


def room_of(x, y, floor=None):
    """Symbolic room containing (x, y), optionally restricted to a floor, or "" if in a wall/doorway.

    Rooms are shrunk by ROOM_MARGIN so dividers/doorways belong to neither room (the grounding node
    applies hysteresis while crossing). `floor` is REQUIRED to disambiguate the stacked landing
    column (f1_stair and f2_stair share (x,y)); elsewhere the x-offset makes (x,y) already unique.
    """
    for r in ROOMS:
        if floor is not None and r.floor != floor:
            continue
        if (abs(x - r.center[0]) <= r.half[0] - ROOM_MARGIN and
                abs(y - r.center[1]) <= r.half[1] - ROOM_MARGIN):
            return r.id
    return ""


def nav_centroid(room_id):
    """The clear standing point a skill drives to for a room (published as its /world_model centroid).
    For the stair landings this is LANDING_XY — offset WEST of the geometric centre so Spot stands
    clear of the decorative staircase (which fills the east of the landing) and faces the ascending
    steps; for every other room it is the geometric centre."""
    if room_id in ("f1_stair", "f2_stair"):
        return LANDING_XY
    for r in ROOMS:
        if r.id == room_id:
            return (r.center[0], r.center[1])
    return (0.0, 0.0)


def portals_between(a, b):
    return [p for p in PORTALS if {p.room_a, p.room_b} == {a, b}]


def stairs_between(a, b):
    return [s for s in STAIRS if {s.room_a, s.room_b} == {a, b}]


def open_portal_ids():
    """Portal ids that are ALWAYS open (passages). Closeable doors' live state comes from /door_states."""
    return [p.id for p in PORTALS if p.always_open]


def door_poses(door):
    """(closed_xyz, open_xyz) for a CLOSEABLE door slab on floor 1 (portcullis lift). Floor-1 anchored
    (z0=0) since only floor 1 has closeable doors."""
    cx, cy = door.pos
    z0 = FLOOR_Z[floor_of(door.room_a)]
    closed = (cx, cy, z0 + WALL_H / 2.0)
    opened = (cx, cy, z0 + WALL_H / 2.0 + DOOR_LIFT)
    return closed, opened


def door_slab_scale(door):
    return (WALL_T, DOOR_W, WALL_H)


def stair_transition_info():
    """What the in-app StairsNode needs: the shared landing (x,y) and the two stand heights."""
    lx, ly = LANDING_XY
    return {
        "stair_id": STAIRS[0].id,
        "landing_xy": (lx, ly),
        "z_bottom": FLOOR_Z["f1"] + SPOT_STAND_DZ,   # 0.8
        "z_top": FLOOR_Z["f2"] + SPOT_STAND_DZ,      # 3.8
        # trigger zone: the CLEAR west/centre of the landing (x in [9,12], y in [-2,2]) — inside the
        # landing [9,15] but west of the decorative staircase [12,15]. The teleport eases x,y to
        # LANDING_XY (10.5,0) regardless, which sits on BOTH floor slabs.
        "footprint": (9.0, LANDING_CENTER_X, ly - 2.0, ly + 2.0),
        "floor_bottom": "f1",
        "floor_top": "f2",
    }


# --------------------------------------------------------------------- geometry for the scene builder
def _floor_bounds(floor):
    rooms = [r for r in ROOMS if r.floor == floor]
    x_min = min(r.center[0] - r.half[0] for r in rooms)
    x_max = max(r.center[0] + r.half[0] for r in rooms)
    return x_min, x_max, -ROOM_HY, ROOM_HY


def wall_segments():
    """(name, center(x,y,z), size(sx,sy,sz)) for perimeter + divider walls of BOTH floors, each
    z-anchored to its floor. Interior dividers are split into two stubs flanking the DOOR_W gap at
    y=0 (open passages leave the gap empty; closeable doors get a slab on top from door_poses())."""
    segs = []
    stub_len = ROOM_HY - DOOR_W / 2.0            # 2.4
    stub_cy = DOOR_W / 2.0 + stub_len / 2.0      # 1.8
    for f in ("f1", "f2"):
        x_min, x_max, y_min, y_max = _floor_bounds(f)
        zc = FLOOR_Z[f] + WALL_H / 2.0
        length_x = (x_max - x_min) + WALL_T
        depth_y = (y_max - y_min)
        segs += [
            (f"{f}_wall_north", (0.5 * (x_min + x_max), y_max, zc), (length_x, WALL_T, WALL_H)),
            (f"{f}_wall_south", (0.5 * (x_min + x_max), y_min, zc), (length_x, WALL_T, WALL_H)),
            (f"{f}_wall_east", (x_max, 0.0, zc), (WALL_T, depth_y, WALL_H)),
            (f"{f}_wall_west", (x_min, 0.0, zc), (WALL_T, depth_y, WALL_H)),
        ]
    # interior dividers (one per portal): two stubs flanking the gap, anchored to the portal's floor
    for p in PORTALS:
        dx = p.pos[0]
        f = floor_of(p.room_a)
        zc = FLOOR_Z[f] + WALL_H / 2.0
        tag = p.id
        segs.append((f"div_{tag}_top", (dx, stub_cy, zc), (WALL_T, stub_len, WALL_H)))
        segs.append((f"div_{tag}_bot", (dx, -stub_cy, zc), (WALL_T, stub_len, WALL_H)))
    return segs


def floor_slab_segments():
    """The two structural floor plates (static colliders Spot stands on). f1 at z=0, f2 at z=3. Each
    plate is slightly thick and spans its floor's full room footprint."""
    out = []
    T = 0.2
    for f in ("f1", "f2"):
        x_min, x_max, y_min, y_max = _floor_bounds(f)
        cx = 0.5 * (x_min + x_max)
        top = FLOOR_Z[f]
        out.append((f"{f}_slab", (cx, 0.0, top - T / 2.0),
                    (x_max - x_min + WALL_T, y_max - y_min + WALL_T, T)))
    return out


def stair_step_boxes():
    """Decorative staircase at the floor-1 landing (x in [12,15]) rising toward the upper floor, purely
    for camera / YOLO / elevation-mapping realism. Solid cumulative blocks give a clean stair profile
    for the elevation map. Static colliders; Spot NEVER drives onto them (the teleport does the climb)."""
    steps = []
    n = 10
    run = 0.3
    rise = 0.30          # 10 * 0.30 = 3.0 => the top step meets the floor-2 slab (FLOOR_Z['f2']=3.0),
                         #                    so the staircase visually connects the two floors
    x0 = 12.0            # start just east of the landing centre, against the east wall (x=15)
    width_y = 1.2
    for i in range(n):
        h = (i + 1) * rise
        cx = x0 + run * (i + 0.5)
        steps.append((f"stair_step_{i}", (cx, 0.0, h / 2.0), (run, width_y, h)))
    return steps


# --------------------------------------------------------------------- self-test (Gate 1, no Isaac)
def _selftest():
    # 1) floor room x-bands must be DISJOINT except the shared landing column [9,15].
    f1x = [(r.center[0] - r.half[0], r.center[0] + r.half[0]) for r in ROOMS
           if r.floor == "f1" and r.id != "f1_stair"]
    f2x = [(r.center[0] - r.half[0], r.center[0] + r.half[0]) for r in ROOMS
           if r.floor == "f2" and r.id != "f2_stair"]
    f1_max = max(hi for _, hi in f1x)      # 9
    f2_min = min(lo for lo, _ in f2x)      # 15
    assert f1_max <= f2_min, f"floor wings overlap in x: f1 max {f1_max} > f2 min {f2_min}"
    # 2) the two landings are STACKED (same x,y, different floor).
    s1 = next(r for r in ROOMS if r.id == "f1_stair")
    s2 = next(r for r in ROOMS if r.id == "f2_stair")
    assert s1.center == s2.center, "stair landings must share (x,y) for the pure-z teleport"
    # 3) room_of must return a UNIQUE, correct room for every room centroid (with the floor key).
    for r in ROOMS:
        got = room_of(r.center[0], r.center[1], floor=r.floor)
        assert got == r.id, f"room_of({r.center},{r.floor}) = {got!r}, expected {r.id!r}"
    # 4) every victim sits in its declared room.
    for (x, y, z, room) in VICTIMS:
        assert room_of(x, y, floor=floor_of(room)) == room, f"victim at {(x, y)} not in {room}"
    print(f"[sar_building] OK: {len(ROOMS)} rooms, {len(PORTALS)} portals "
          f"({len(CLOSEABLE_DOORS)} closeable), {len(STAIRS)} stair(s), {len(VICTIMS)} victims; "
          f"f1 wing<= x{f1_max}, f2 wing>= x{f2_min}, landing stacked at {s1.center}.")


if __name__ == "__main__":
    _selftest()
