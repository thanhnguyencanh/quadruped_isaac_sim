"""replicator_sar_sdg.py — Phase 2: synthetic labelled dataset for victim detection.

Uses NVIDIA Replicator to render the SAR scene from randomized viewpoints + lighting and write
a labelled dataset (RGB + 2D tight bounding boxes + semantic segmentation) via the BasicWriter.
The victims carry the "victim" semantic label (set in sar_scene), distractors "distractor", so a
learned detector (e.g. YOLO) can be trained to find victims and ignore distractors — the upgrade
path from the current HSV detector (same /victims topic contract).

Run (headless; first RTX run is slow):
  scripts/run_isaac.sh spot_sar_sim/spot_sar_sim/standalone/replicator_sar_sdg.py --frames 30
Output: ~/unige_ws/datasets/sar_victims/ (rgb/, bounding_box_2d_tight/, semantic_segmentation/).
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from isaacsim import SimulationApp

parser = argparse.ArgumentParser(description="Replicator SAR synthetic data generation")
parser.add_argument("--frames", type=int, default=30, help="number of labelled frames to write")
parser.add_argument("--out", default=os.path.expanduser("~/unige_ws/datasets/sar_victims"),
                    help="output dataset directory")
args, _ = parser.parse_known_args()

simulation_app = SimulationApp(
    {"headless": True, "width": 640, "height": 480, "renderer": "RayTracedLighting"}
)

import carb

LOCAL_ASSETS = os.environ.get("ISAAC_ASSETS", os.path.expanduser("~/isaacsim_assets"))
carb.settings.get_settings().set("/persistent/isaac/asset_root/default", LOCAL_ASSETS)
_OMNI_CACHE = os.environ.get("OMNI_CACHE_DIR", os.path.expanduser("~/.cache/isaacsim_omni_cache"))
os.makedirs(_OMNI_CACHE, exist_ok=True)
try:
    carb.tokens.get_tokens_interface().set_value("omni_cache", _OMNI_CACHE)
except Exception as e:  # noqa: BLE001
    print(f"[sdg] could not set omni_cache token: {e}", flush=True)

import omni.replicator.core as rep
from isaacsim.core.experimental.utils.stage import define_prim
from isaacsim.storage.native import get_assets_root_path
from sar_scene import build_sar_scene

# ---------------------------------------------------------------- scene
assets_root_path = get_assets_root_path()
define_prim("/World/Ground", "Xform").GetReferences().AddReference(
    assets_root_path + "/Isaac/Environments/Grid/default_environment.usd"
)
victims = build_sar_scene(label_semantics=True)
print(f"[sdg] SAR scene built: {len(victims)} victims (labelled 'victim') + 1 distractor", flush=True)
simulation_app.update()

# ---------------------------------------------------------------- Replicator graph
camera = rep.create.camera()
render_product = rep.create.render_product(camera, (640, 480))
dome = rep.create.light(light_type="Dome", intensity=800)

writer = rep.WriterRegistry.get("BasicWriter")
os.makedirs(args.out, exist_ok=True)
writer.initialize(
    output_dir=args.out,
    rgb=True,
    bounding_box_2d_tight=True,
    semantic_segmentation=True,
)
writer.attach([render_product])

# Domain randomization: orbit the camera over the room, always looking at the scene centre.
# (Lighting/material randomization is an easy extension — add a `with dome:` modify block.)
with rep.trigger.on_frame(num_frames=args.frames):
    with camera:
        rep.modify.pose(
            position=rep.distribution.uniform((-8.0, -8.0, 1.0), (8.0, 8.0, 2.8)),
            look_at=(0.0, 0.0, 0.3),
        )

print(f"[sdg] generating {args.frames} frames -> {args.out}", flush=True)
rep.orchestrator.run()
simulation_app.update()
print(f"[sdg] DONE: wrote labelled frames to {args.out}", flush=True)
simulation_app.close()
