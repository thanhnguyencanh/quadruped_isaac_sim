"""sar_floor — the SINGLE SOURCE OF TRUTH for the multi-room SAR floor (rooms + doors + victims).

Pure Python, NO pxr / isaacsim / rclpy imports, so every layer can import it cheaply and stay in
sync — the Isaac scene builder (Isaac's bundled python), the floor grounding node + planner
(system Jazzy / planning venv). If the geometry here changes, the sim, the symbol grounding, and
the PDDL problem all change together — they can never drift.

Layout: 3 rooms in a line along +X, each 6 m x 6 m, separated by interior divider walls with a
1.2 m door gap at y=0. Spot spawns in the middle room (room_b). A victim in room_c forces opening
door_bc; a second victim in room_a exercises door_ab.

    y
    ^   +-----------+-----------+-----------+
    |   |  room_a   |  room_b   |  room_c   |
    |   |   v1 .     |   [SPOT]  |     . v0  |
    |   +====D ab====+====D bc===+===========+   (== walls, D = door gap)
    +----------------------------------------> x
       -9    -6   -3     0     3     6     9
"""
from collections import namedtuple

# ---- metric constants (metres) ----
WALL_H = 2.0          # wall / door height
WALL_T = 0.2          # wall + door-slab thickness
ROOM_HX = 3.0         # room half-extent in x  (room is 6 m wide)
ROOM_HY = 3.0         # room half-extent in y  (room is 6 m deep)
DOOR_W = 1.2          # door opening width (along y)
DOOR_LIFT = 2.3       # how far the slab rises when "open" (slab bottom clears the wall top)
ROOM_MARGIN = 0.4     # room_of() shrinks each room by this so doorways belong to NEITHER room

Room = namedtuple("Room", "id center half")     # center=(cx,cy), half=(hx,hy)
Door = namedtuple("Door", "id room_a room_b pos")  # pos=(cx,cy) = slab CLOSED centre on the divider

ROOMS = [
    Room("room_a", (-6.0, 0.0), (ROOM_HX, ROOM_HY)),   # x in [-9, -3]
    Room("room_b", (0.0, 0.0), (ROOM_HX, ROOM_HY)),    # x in [-3,  3]
    Room("room_c", (6.0, 0.0), (ROOM_HX, ROOM_HY)),    # x in [ 3,  9]
]

DOORS = [
    Door("door_ab", "room_a", "room_b", (-3.0, 0.0)),  # divider at x = -3
    Door("door_bc", "room_b", "room_c", (3.0, 0.0)),   # divider at x = +3
]

# victims: (x, y, z, room). z is the marker centre height.
VICTIMS = [
    (6.0, 1.5, 0.3, "room_c"),    # behind door_bc -> forces open-door(door_bc)
    (-6.0, -1.5, 0.3, "room_a"),  # behind door_ab -> forces open-door(door_ab)
]

SPAWN_ROOM = "room_b"   # Spot spawns at (0,0,0.8) = inside room_b


def room_of(x, y):
    """Symbolic room containing (x, y), or "" if in a wall / doorway (no-man's-land).

    Rooms are shrunk by ROOM_MARGIN so the divider line + doorway belong to neither room; the
    grounding node applies hysteresis (keep the last valid room) while the robot crosses.
    """
    for r in ROOMS:
        if (abs(x - r.center[0]) <= r.half[0] - ROOM_MARGIN and
                abs(y - r.center[1]) <= r.half[1] - ROOM_MARGIN):
            return r.id
    return ""


def doors_between(a, b):
    """All doors connecting rooms a and b (order-independent)."""
    return [d for d in DOORS if {d.room_a, d.room_b} == {a, b}]


def door_poses(door):
    """(closed_xyz, open_xyz) for a door slab. Open = lifted straight up like a portcullis so the
    slab clears the 0..WALL_H opening (and the laser plane) without intersecting the ground."""
    cx, cy = door.pos
    closed = (cx, cy, WALL_H / 2.0)
    opened = (cx, cy, WALL_H / 2.0 + DOOR_LIFT)
    return closed, opened


def door_slab_scale(door):
    """Full extents (sx, sy, sz) of the door slab: thin in x, spans the gap in y, full height."""
    return (WALL_T, DOOR_W, WALL_H)


def _outer_bounds():
    xs = [r.center[0] for r in ROOMS]
    x_min = min(xs) - ROOM_HX
    x_max = max(xs) + ROOM_HX
    return x_min, x_max, -ROOM_HY, ROOM_HY


def wall_segments():
    """List of (name, center(x,y,z), size(sx,sy,sz)) for perimeter + divider walls.

    Each interior divider is split into TWO stubs flanking the door gap (gap = DOOR_W at y=0).
    Box convention matches sar_scene._box: size = full extents of a unit Cube scaled.
    """
    x_min, x_max, y_min, y_max = _outer_bounds()
    z = WALL_H / 2.0
    length_x = (x_max - x_min) + WALL_T   # overlap the corners
    depth_y = (y_max - y_min)
    segs = [
        ("wall_north", (0.0, y_max, z), (length_x, WALL_T, WALL_H)),
        ("wall_south", (0.0, y_min, z), (length_x, WALL_T, WALL_H)),
        ("wall_east", (x_max, 0.0, z), (WALL_T, depth_y, WALL_H)),
        ("wall_west", (x_min, 0.0, z), (WALL_T, depth_y, WALL_H)),
    ]
    # interior dividers (one per door position): two stubs flanking the 1.2 m gap at y=0
    stub_len = (ROOM_HY - DOOR_W / 2.0)            # from gap edge (0.6) to wall (3.0) -> 2.4
    stub_cy = DOOR_W / 2.0 + stub_len / 2.0        # centre of each stub -> 1.8
    for d in DOORS:
        dx = d.pos[0]
        tag = d.id.replace("door_", "div_")
        segs.append((f"{tag}_top", (dx, stub_cy, z), (WALL_T, stub_len, WALL_H)))
        segs.append((f"{tag}_bot", (dx, -stub_cy, z), (WALL_T, stub_len, WALL_H)))
    return segs
