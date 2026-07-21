"""spot_sar_sim — minimal scene viewer: just Isaac Sim + the SAR environment + Spot.

The leanest possible app for *looking* at the simulation. NO ROS 2 at all — no bridge
extension, no rclpy, no OmniGraph publish graphs, no camera render product. It only:

  1. boots Isaac Sim (GUI by default — the whole point is to see it),
  2. loads the SAR environment (grid ground + walled room + human victims; --no-humans for markers),
  3. spawns Spot with the flat-terrain policy and lets it stand,
  4. renders forever until you close the window (Spot holds a zero command = stands still).

Use this as the first step-by-step bring-up check: if the viewport shows the room + Spot,
the renderer/scene/robot are healthy, and you can add ROS (spot_cmd_vel_app.py) on top next.

LOAD-BEARING control ordering (do NOT reorder — same as the cmd_vel/perception apps):
  play() -> update() -> spot.initialize() -> update() -> register FORWARD-ONLY physics
  callback. Initializing the policy before the PhysX articulation view exists (or inside the
  callback) makes joint-target writes no-ops and Spot collapses to the floor.

Run:  scripts/run_isaac.sh spot_sar_sim/spot_sar_sim/standalone/spot_view_scene.py
      scripts/run_isaac.sh spot_sar_sim/spot_sar_sim/standalone/spot_view_scene.py --floor     # 3-room floor + doors
      scripts/run_isaac.sh spot_sar_sim/spot_sar_sim/standalone/spot_view_scene.py --headless
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sar_scene import build_sar_scene, build_floor_scene, build_two_floor_scene  # shared SAR environments

# sar_floor (the multi-room floor plan) lives in spot_sar_planning; add its dir for build_floor_scene()
_REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))
sys.path.insert(0, os.path.join(_REPO, "spot_sar_planning", "spot_sar_planning"))

from isaacsim import SimulationApp

parser = argparse.ArgumentParser(description="Minimal Isaac Sim viewer: SAR env + Spot (no ROS)")
parser.add_argument("--headless", action="store_true", help="run without the GUI window")
parser.add_argument("--floor", action="store_true",
                    help="show the multi-room floor (walls + doors) instead of the single room")
parser.add_argument("--building", action="store_true",
                    help="show the TWO-FLOOR building (x-offset wings + stacked stair landing + stairs)")
parser.add_argument("--no-humans", dest="humans", action="store_false",
                    help="use orange box victims instead of human figures (same flag as spot_perception_app.py)")
parser.add_argument("--device", choices=["cpu", "cuda"], default="cpu", help="physics/policy device")
parser.add_argument("--steps", type=int, default=0, help="auto-exit after N render frames (0 = run forever)")
args, _ = parser.parse_known_args()

# GUI on by default (this app exists to be looked at); --headless to drop the window.
simulation_app = SimulationApp(
    {"headless": args.headless, "width": 1280, "height": 720, "renderer": "RayTracedLighting"}
)

import carb

# Pin the real local asset root (the persistent config points at a stale /home/isaac path).
LOCAL_ASSETS = os.environ.get("ISAAC_ASSETS", os.path.expanduser("~/isaacsim_assets"))
carb.settings.get_settings().set("/persistent/isaac/asset_root/default", LOCAL_ASSETS)

# Redirect Kit's ${omni_cache} (stale Docker path) to a writable dir so derived/OGN caches persist.
_OMNI_CACHE = os.environ.get("OMNI_CACHE_DIR", os.path.expanduser("~/.cache/isaacsim_omni_cache"))
os.makedirs(_OMNI_CACHE, exist_ok=True)
try:
    carb.tokens.get_tokens_interface().set_value("omni_cache", _OMNI_CACHE)
    print(f"[view] omni_cache -> {_OMNI_CACHE}", flush=True)
except Exception as e:  # noqa: BLE001
    print(f"[view] could not set omni_cache token: {e}", flush=True)

import numpy as np
import omni.timeline
from isaacsim.core.deprecation_manager import import_module
from isaacsim.core.experimental.utils.stage import define_prim
from isaacsim.core.rendering_manager import RenderingManager
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.simulation_manager.impl.isaac_events import IsaacEvents
from isaacsim.robot.policy.examples.robots import SpotFlatTerrainPolicy
from isaacsim.storage.native import get_assets_root_path

torch = import_module("torch")
import warp as wp

SPOT_PRIM = "/World/Spot"
PHYSICS_HZ = 200.0

# ---------------------------------------------------------------- scene + robot
assets_root_path = get_assets_root_path()
print(f"[view] assets_root_path = {assets_root_path}", flush=True)

define_prim("/World/Ground", "Xform").GetReferences().AddReference(
    assets_root_path + "/Isaac/Environments/Grid/default_environment.usd"
)
define_prim("/World/PhysicsScene", "PhysicsScene")

RenderingManager.set_dt(8.0 / PHYSICS_HZ)  # ~25 Hz render (caps GPU load on 8 GB VRAM)
SimulationManager.set_physics_sim_device(args.device)
SimulationManager.set_physics_dt(1.0 / PHYSICS_HZ)

spot = SpotFlatTerrainPolicy(prim_path=SPOT_PRIM, position=[0.0, 0.0, 0.8])
base_command = torch.zeros(3, device=args.device)  # [vx, vy, wz] — held at zero = stand still

# ---- environment: single room | multi-room floor (doors) | two-floor building (stairs) ----
# Same victim style as spot_perception_app.py: Isaac People humans by default (needs the asset
# root, streamed from cloud if no local pack), orange markers with --no-humans.
if args.building:
    _victims, _doors, _stair = build_two_floor_scene(
        humans=args.humans, assets_root_path=assets_root_path)
    print(f"[view] BUILDING scene: {len(_victims)} victim(s) (humans={args.humans}); "
          f"doors={list(_doors)}; stair={_stair['stair_id']} landing={_stair['landing_xy']} "
          f"(static viewer — no ROS to open doors or run the stair teleport; use --building in "
          f"spot_perception_app.py for that)", flush=True)
elif args.floor:
    _victims, _doors = build_floor_scene(humans=args.humans, assets_root_path=assets_root_path)
    print(f"[view] FLOOR scene: {len(_victims)} victim(s) (humans={args.humans}); doors={list(_doors)} "
          f"(closed — no ROS in this viewer to open them; use --floor in spot_perception_app.py for that)",
          flush=True)
else:
    _victims = build_sar_scene(humans=args.humans, assets_root_path=assets_root_path)
    print(f"[view] SAR scene: {len(_victims)} victim(s) (humans={args.humans}) at {_victims}", flush=True)


def _base_xyz():
    try:
        pos, _ = spot.robot.get_world_poses()
        return wp.to_torch(pos).cpu().numpy().reshape(-1)[:3]
    except Exception:  # noqa: BLE001
        return np.array([float("nan")] * 3)


# ---------------------------------------------------------------- control init (ORDER IS LOAD-BEARING)
def on_physics_step(step_size, _context):
    spot.forward(step_size, base_command)  # forward-only; NEVER initialize() in here


timeline = omni.timeline.get_timeline_interface()
timeline.play()
simulation_app.update()  # _on_play -> initialize_physics() creates the PhysX articulation view
spot.initialize()  # commit position-control mode + PD gains to the live stepping view
simulation_app.update()  # bake one step of drive/gains state
SimulationManager.register_callback(on_physics_step, IsaacEvents.POST_PHYSICS_STEP)
simulation_app.update()

mode = "headless" if args.headless else "GUI"
print(f"[view] {mode}: SAR room + Spot loaded; Spot standing. Close the window (or Ctrl+C) to exit.", flush=True)

frame = 0
while simulation_app.is_running():
    simulation_app.update()
    if not SimulationManager.is_simulating():
        continue
    if frame % 200 == 0:
        print(f"[view] frame={frame} spot_base_xyz={np.round(_base_xyz(), 3)}", flush=True)
    frame += 1
    if args.steps and frame >= args.steps:
        print(f"[view] reached --steps {args.steps}, exiting.", flush=True)
        break

simulation_app.close()
print("[view] done.", flush=True)
