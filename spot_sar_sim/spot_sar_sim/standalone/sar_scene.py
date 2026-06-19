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


def build_sar_scene(label_semantics=False):
    """Create the walled room + victims (orange, labelled 'victim') + a distractor.

    label_semantics: apply UsdSemantics class labels (for the Replicator SDG). Leave False in the
    live sim (the perception app doesn't need semantics and labelling pulls in Replicator).
    Returns the list of victim (x, y, z) world positions.
    """
    from pxr import Gf, Sdf, UsdGeom, UsdShade
    from isaacsim.core.experimental.utils.stage import define_prim

    for name, pos, scale in WALLS:
        _box(define_prim, UsdGeom, Gf, f"/World/{name}", pos, scale, WALL_COLOR)

    for i, pos in enumerate(VICTIM_POS):
        p = _box(define_prim, UsdGeom, Gf, f"/World/Victim_{i}", pos, None, VICTIM_COLOR, size=0.5)
        _emissive_material(UsdShade, Sdf, Gf, p, VICTIM_COLOR)
        if label_semantics:
            _label(p, "victim")

    d = _box(define_prim, UsdGeom, Gf, "/World/Distractor_0", DISTRACTOR_POS, None,
             DISTRACTOR_COLOR, size=0.5)
    _emissive_material(UsdShade, Sdf, Gf, d, DISTRACTOR_COLOR)
    if label_semantics:
        _label(d, "distractor")

    return list(VICTIM_POS)
