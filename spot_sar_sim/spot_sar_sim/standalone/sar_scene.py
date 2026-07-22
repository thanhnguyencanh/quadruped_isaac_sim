"""Shared SAR scene builder — Phase 2 (SAR Environment & Sensor Suite).

Builds the search-and-rescue environment used by both the live sim (spot_perception_app.py)
and the synthetic-data generator (replicator_sar_sdg.py): a walled room with SEVERAL victim
markers spread across it, each given a UsdPreviewSurface (saturated orange, the HSV detector's
target) AND a semantic label "victim" so Replicator's segmentation / bbox annotators pick them
up. Distractor objects (a non-victim coloured box) are labelled separately so a learned detector
can be trained to discriminate.

Imported by the standalone apps via sys.path insertion (they run by file path under Isaac's
python). pxr / isaacsim imports happen at call time so importing this module is cheap and only
touches USD once the SimulationApp exists.
"""

# room: 10 x 10 m, walls 2 m tall. Each entry: (name, center, scale[x,y,z]).
WALL_COLOR = (0.6, 0.6, 0.6)
WALLS = [
    ("wall_n", (5.0, 0.0, 1.0), (0.2, 10.0, 2.0)),
    ("wall_s", (-5.0, 0.0, 1.0), (0.2, 10.0, 2.0)),
    ("wall_e", (0.0, -5.0, 1.0), (10.0, 0.2, 2.0)),
    ("wall_w", (0.0, 5.0, 1.0), (10.0, 0.2, 2.0)),
]
VICTIM_COLOR = (1.0, 0.35, 0.0)
# victims spread across the room so the robot must navigate between them.
VICTIM_POS = [
    (3.0, 1.5, 0.3),
    (3.0, -2.5, 0.3),
    (-3.0, 2.5, 0.3),
]
# a non-victim distractor (blue), labelled so a learned detector can be trained to ignore it.
DISTRACTOR_COLOR = (0.1, 0.2, 0.9)
DISTRACTOR_POS = (-2.0, -3.0, 0.3)

# Isaac People characters used as human victims (relative to assets_root_path). Cycled across
# victim positions. These are realistic, textured, standing humans -> a pretrained YOLO 'person'
# detector finds them. (Resolve via assets_root_path + f"/Isaac/People/Characters/{n}/{n}.usd".)
HUMAN_CHARACTERS = ["F_Business_02", "M_Medical_01", "male_adult_construction_05_new"]


def _box(define_prim, UsdGeom, Gf, path, pos, scale, color, size=1.0):
    prim = define_prim(path, "Cube")
    UsdGeom.Cube(prim).GetSizeAttr().Set(float(size))
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    if scale is not None:
        xf.AddScaleOp().Set(Gf.Vec3f(*scale))
    UsdGeom.Gprim(prim).GetDisplayColorAttr().Set([Gf.Vec3f(*color)])
    return prim


def _emissive_material(UsdShade, Sdf, Gf, prim, color):
    stage = prim.GetStage()
    path = str(prim.GetPath())
    mat = UsdShade.Material.Define(stage, path + "/Mat")
    shd = UsdShade.Shader.Define(stage, path + "/Mat/Shader")
    shd.CreateIdAttr("UsdPreviewSurface")
    shd.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shd.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shd.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.6)
    mat.CreateSurfaceOutput().ConnectToSource(shd.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI(prim).Apply(prim)
    UsdShade.MaterialBindingAPI(prim).Bind(mat)


def _label(prim, semantic_label):
    """Attach a UsdSemantics class label so Replicator's bbox/segmentation annotators pick it up.

    Isaac 6.0: `add_labels(prim, labels, instance_name)` (applies UsdSemantics.LabelsAPI); it pulls
    in omni.replicator.core.functional, so only call this from the SDG (label_semantics=True).
    """
    try:
        from isaacsim.core.utils.semantics import add_labels
        add_labels(prim, [semantic_label], "class")
        return
    except Exception:  # noqa: BLE001
        pass
    try:
        import omni.replicator.core.functional as F
        F.modify.semantics(prim, {"class": [semantic_label]}, mode="replace")
    except Exception:  # noqa: BLE001
        pass


def _add_human(define_prim, UsdGeom, Gf, path, pos, yaw_deg, character_usd):
    """Reference an Isaac People character (SkelRoot, Z-up) at a victim position, standing on the
    floor. A static reference renders the skeleton's BIND POSE (standing) — no animation graph
    needed. `pos` is the FOOT position; pos[2] is the floor's slab-top height (0.0 on floor 1,
    3.0 on the upper floor of the two-floor building)."""
    prim = define_prim(path, "Xform")
    prim.GetReferences().AddReference(character_usd)
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(float(pos[0]), float(pos[1]), float(pos[2])))
    xf.AddRotateZOp().Set(float(yaw_deg))  # face a chosen direction
    return prim


def _victims_or_humans(define_prim, UsdGeom, UsdShade, Sdf, Gf, positions, humans, assets_root_path,
                       label_semantics, foot_zs=None):
    """Place victims as Isaac People humans (if humans + assets_root_path) or orange emissive boxes.
    `positions` is a list of (x, y, z) with z the ABSOLUTE marker-centre height. `foot_zs` (parallel,
    optional) gives each floor's slab-top height for standing HUMANS (default 0.0 = single floor).
    Returns the list of reported victim positions."""
    from pxr import UsdPhysics

    use_humans = bool(humans and assets_root_path)
    if humans and not assets_root_path:
        print("[sar_scene] humans requested but no assets_root_path given; using orange markers",
              flush=True)
    out = []
    for i, (x, y, z) in enumerate(positions):
        if use_humans:
            foot_z = 0.0 if foot_zs is None else float(foot_zs[i])
            name = HUMAN_CHARACTERS[i % len(HUMAN_CHARACTERS)]
            usd = assets_root_path + f"/Isaac/People/Characters/{name}/{name}.usd"
            yaw = 180.0 if x > 0 else 0.0  # roughly face the room interior / patrolling robot
            p = _add_human(define_prim, UsdGeom, Gf, f"/World/Victim_{i}", (x, y, foot_z), yaw, usd)
            # INVISIBLE collision proxy: the skeletal character mesh has no physics, so without
            # this Spot walks straight THROUGH people. A person-sized static column makes the
            # victim solid (and visible to the horizontal lidar plane); invisible to rendering,
            # so the camera/YOLO see only the human mesh.
            proxy = _box(define_prim, UsdGeom, Gf, f"/World/Victim_{i}_collision",
                         (x, y, foot_z + 0.85), (0.4, 0.4, 1.7), VICTIM_COLOR)
            UsdGeom.Imageable(proxy).MakeInvisible()
            _add_static_collider(UsdPhysics, proxy)
            if label_semantics:
                _label(p, "victim")
            out.append((x, y, foot_z))  # report foot position (on the floor's slab)
        else:
            p = _box(define_prim, UsdGeom, Gf, f"/World/Victim_{i}", (x, y, z), None, VICTIM_COLOR, size=0.5)
            _add_static_collider(UsdPhysics, p)  # victims are solid in marker mode too
            _emissive_material(UsdShade, Sdf, Gf, p, VICTIM_COLOR)
            if label_semantics:
                _label(p, "victim")
            out.append((x, y, z))
    return out


def _add_lighting(define_prim, UsdLux, Gf):
    """Even, shadow-filling lighting so EVERY room is visible. The 2 m walls block a single key
    light and leave the side rooms dark, so we add a bright DomeLight (ambient fill from all
    directions — reaches the roofless rooms) plus an overhead DistantLight for a bit of shading.
    Idempotent-ish: re-defining the same prim paths just re-sets the attrs."""
    stage = define_prim("/World/SceneLights", "Xform").GetStage()
    dome = UsdLux.DomeLight.Define(stage, "/World/SceneLights/Dome")
    dome.CreateIntensityAttr(1200.0)
    dome.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 1.0))
    key = UsdLux.DistantLight.Define(stage, "/World/SceneLights/Key")
    key.CreateIntensityAttr(1500.0)   # shines along local -Z = straight down (lights every floor)
    key.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 0.97))


def build_sar_scene(label_semantics=False, humans=True, assets_root_path=None):
    """Create the walled room + victims + a distractor.

    humans + assets_root_path: place realistic Isaac People characters as victims (the YOLO target);
      if humans is False (or assets_root_path is missing — e.g. the Replicator SDG), fall back to the
      orange emissive markers (the HSV target).
    label_semantics: apply UsdSemantics class labels (for the Replicator SDG).
    Returns the list of victim (x, y, z) world positions.
    """
    from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdShade, UsdPhysics
    from isaacsim.core.experimental.utils.stage import define_prim

    _add_lighting(define_prim, UsdLux, Gf)
    # static colliders like the floor/building scenes: the walls must be PHYSICAL, both so the
    # lidar raycasts hit them and so Spot cannot walk through them (visual-only before).
    for name, pos, scale in WALLS:
        p = _box(define_prim, UsdGeom, Gf, f"/World/{name}", pos, scale, WALL_COLOR)
        _add_static_collider(UsdPhysics, p)

    victims = _victims_or_humans(define_prim, UsdGeom, UsdShade, Sdf, Gf, list(VICTIM_POS),
                                 humans, assets_root_path, label_semantics)

    d = _box(define_prim, UsdGeom, Gf, "/World/Distractor_0", DISTRACTOR_POS, None,
             DISTRACTOR_COLOR, size=0.5)
    _add_static_collider(UsdPhysics, d)
    _emissive_material(UsdShade, Sdf, Gf, d, DISTRACTOR_COLOR)
    if label_semantics:
        _label(d, "distractor")

    return victims


# ---------------------------------------------------------------- multi-room floor (door demo)
DOOR_COLOR = (0.45, 0.30, 0.15)  # wood-ish, visually distinct from the grey walls


def _add_static_collider(UsdPhysics, prim):
    """Make a Cube an immovable STATIC collider (walls). Analytic Cube needs no mesh approximation;
    no RigidBodyAPI => infinite mass / immovable, the cheapest collider for PhysX."""
    UsdPhysics.CollisionAPI.Apply(prim)


def _add_kinematic_collider(UsdPhysics, prim):
    """Make a Cube a KINEMATIC collider (door slab): it collides + occludes the depth/lidar, and we
    relocate it every frame via its xform op (kinematic teleport, not physics-driven)."""
    UsdPhysics.CollisionAPI.Apply(prim)
    rb = UsdPhysics.RigidBodyAPI.Apply(prim)
    rb.CreateKinematicEnabledAttr(True)


def build_floor_scene(label_semantics=False, humans=True, assets_root_path=None):
    """Multi-room floor (sar_floor.py): perimeter + divider walls with door gaps, a sliding door
    slab per door, and victim markers. Walls/slabs get colliders so SLAM/Nav2 see them.

    Requires `sar_floor` importable (the Isaac app inserts spot_sar_planning's source dir on
    sys.path before calling this — one source of truth shared with the grounding node + planner).
    Returns (victim_xyz_list, door_handles) where door_handles[door_id] = {prim, closed, open}.
    """
    from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdShade, UsdPhysics
    from isaacsim.core.experimental.utils.stage import define_prim
    import sar_floor as FLOOR

    _add_lighting(define_prim, UsdLux, Gf)  # fill light so every room is visible
    # 1. walls — static colliders
    for name, pos, size in FLOOR.wall_segments():
        p = _box(define_prim, UsdGeom, Gf, f"/World/{name}", pos, size, WALL_COLOR)
        _add_static_collider(UsdPhysics, p)

    # 2. door slabs — kinematic colliders we lift open each frame (translate op = ordered op [0])
    door_handles = {}
    for d in FLOOR.DOORS:
        closed, opened = FLOOR.door_poses(d)
        size = FLOOR.door_slab_scale(d)
        p = _box(define_prim, UsdGeom, Gf, f"/World/{d.id}", closed, size, DOOR_COLOR)
        _add_kinematic_collider(UsdPhysics, p)
        door_handles[d.id] = {"prim": p, "closed": closed, "open": opened}

    # 3. victims — Isaac People humans (YOLO target) or orange emissive markers (HSV target)
    positions = [(x, y, z) for (x, y, z, _room) in FLOOR.VICTIMS]
    victims = _victims_or_humans(define_prim, UsdGeom, UsdShade, Sdf, Gf, positions,
                                 humans, assets_root_path, label_semantics)

    return victims, door_handles


# ---------------------------------------------------------------- two-floor building (stairs demo)
SLAB_COLOR = (0.50, 0.50, 0.55)   # structural floor plates
STEP_COLOR = (0.40, 0.32, 0.22)   # decorative staircase


def _add_stair_fill_light(define_prim, UsdLux, UsdGeom, Gf, xy):
    """The floor-1 stair landing sits UNDER the floor-2 slab, so the DomeLight/DistantLight can't
    reach it. Add a local sphere light so the decorative stairs are visible to the camera / the
    elevation map."""
    stage = define_prim("/World/SceneLights", "Xform").GetStage()
    light = UsdLux.SphereLight.Define(stage, "/World/SceneLights/StairFill")
    light.CreateIntensityAttr(45000.0)
    light.CreateRadiusAttr(0.35)
    light.CreateColorAttr(Gf.Vec3f(1.0, 0.98, 0.95))
    UsdGeom.Xformable(light.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(float(xy[0]), float(xy[1]), 1.7))


def build_two_floor_scene(label_semantics=False, humans=True, assets_root_path=None):
    """TWO-FLOOR building (sar_building.py): two x-offset floor wings sharing a vertically-stacked
    stair landing column, joined only by a stair the robot crosses via an in-app pure-z teleport.
    Builds: two floor plates + perimeter/divider walls (both floors, z-anchored) + a decorative
    staircase at the floor-1 landing + closeable door slabs on floor 1 + humans on BOTH floors.

    Requires `sar_building` importable (the Isaac app inserts spot_sar_planning's source dir on
    sys.path first). Returns (victim_xyz_list, door_handles, stair_info) where stair_info feeds the
    in-app StairsNode (shared landing xy + the two stand heights + trigger footprint)."""
    from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdShade, UsdPhysics
    from isaacsim.core.experimental.utils.stage import define_prim
    import sar_building as BLD

    _add_lighting(define_prim, UsdLux, Gf)                       # fill light for both open-top wings
    _add_stair_fill_light(define_prim, UsdLux, UsdGeom, Gf, BLD.LANDING_XY)

    # 1. structural floor plates (static colliders Spot stands on): f1 at z=0, f2 at z=3
    for name, pos, size in BLD.floor_slab_segments():
        p = _box(define_prim, UsdGeom, Gf, f"/World/{name}", pos, size, SLAB_COLOR)
        _add_static_collider(UsdPhysics, p)

    # 2. perimeter + divider walls of BOTH floors (static colliders)
    for name, pos, size in BLD.wall_segments():
        p = _box(define_prim, UsdGeom, Gf, f"/World/{name}", pos, size, WALL_COLOR)
        _add_static_collider(UsdPhysics, p)

    # 3. decorative staircase at the floor-1 landing (static colliders; Spot NEVER climbs it)
    for name, pos, size in BLD.stair_step_boxes():
        p = _box(define_prim, UsdGeom, Gf, f"/World/{name}", pos, size, STEP_COLOR)
        _add_static_collider(UsdPhysics, p)

    # 4. closeable door slabs on floor 1 — kinematic colliders lifted open by the door bus
    door_handles = {}
    for d in BLD.CLOSEABLE_DOORS:
        closed, opened = BLD.door_poses(d)
        size = BLD.door_slab_scale(d)
        p = _box(define_prim, UsdGeom, Gf, f"/World/{d.id}", closed, size, DOOR_COLOR)
        _add_kinematic_collider(UsdPhysics, p)
        door_handles[d.id] = {"prim": p, "closed": closed, "open": opened}

    # 5. victims — Isaac People humans on BOTH floors (feet on each floor's slab) or orange markers
    positions = [(x, y, z) for (x, y, z, _room) in BLD.VICTIMS]
    foot_zs = [BLD.FLOOR_Z[BLD.floor_of(room)] for (_x, _y, _z, room) in BLD.VICTIMS]
    victims = _victims_or_humans(define_prim, UsdGeom, UsdShade, Sdf, Gf, positions,
                                 humans, assets_root_path, label_semantics, foot_zs=foot_zs)

    return victims, door_handles, BLD.stair_transition_info()
