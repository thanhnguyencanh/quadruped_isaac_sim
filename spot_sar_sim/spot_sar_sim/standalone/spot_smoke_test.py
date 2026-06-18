"""FAST headless validation of Isaac Sim 6.0 + GPU + Spot locomotion (physics only).

Steps PhysX with ``world.step(render=False)`` on a bare ground plane, so it skips the
expensive first-frame RTX shader compilation entirely (~3 s of compute vs. many minutes
for the render path on a cold Blackwell shader cache). Validates: Isaac boots, the Spot
USD loads, the flat-terrain policy (``spot_policy.pt``) loads + runs, and Spot walks
forward in response to a velocity command.

Run with::

    scripts/run_isaac.sh spot_sar_sim/spot_sar_sim/standalone/spot_smoke_test.py

Expected: "[smoke] SUCCESS: Spot loaded its policy and walked forward." (≈2.7 m in 600 steps).
For a RENDERED check (slow first run; validates the RTX path + camera readiness) see
``spot_smoke_render.py``.
"""
import os
import time

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import carb

# Isaac's persistent asset root may point at a stale Docker path; pin the real local assets.
LOCAL_ASSETS = os.environ.get("ISAAC_ASSETS", os.path.expanduser("~/isaacsim_assets"))
carb.settings.get_settings().set("/persistent/isaac/asset_root/default", LOCAL_ASSETS)

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.deprecation_manager import import_module
from isaacsim.robot.policy.examples.robots import SpotFlatTerrainPolicy
from isaacsim.storage.native import get_assets_root_path

torch = import_module("torch")

print(f"[smoke] assets_root_path = {get_assets_root_path()}", flush=True)
print("[smoke] building World (physics-only, no render)...", flush=True)

world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 200.0, rendering_dt=8.0 / 200.0)
world.scene.add_default_ground_plane()

spot = SpotFlatTerrainPolicy(prim_path="/World/Spot", position=[0.0, 0.0, 0.8])
base_command = torch.zeros(3, device="cpu")

world.reset()
spot.initialize()
print("[smoke] Spot initialized; stepping physics...", flush=True)

t0 = time.time()
N = 600  # physics steps = 3.0 s of sim time
start_xy = None
for i in range(N):
    if i == 20:
        start_xy = np.array(spot.robot.get_world_poses()[0]).reshape(-1)[:2]
        print(f"[smoke] start xy = {start_xy}", flush=True)
    if i >= 20:
        base_command = torch.tensor([1.0, 0.0, 0.0], device="cpu")  # walk forward 1 m/s
    spot.forward(1.0 / 200.0, base_command)
    world.step(render=False)
    if i % 100 == 0:
        print(f"[smoke] step {i}/{N}  (t={time.time() - t0:.1f}s)", flush=True)

end_xy = np.array(spot.robot.get_world_poses()[0]).reshape(-1)[:2]
dist = float(np.linalg.norm(end_xy - start_xy)) if start_xy is not None else -1.0
print(f"[smoke] end xy   = {end_xy}", flush=True)
print(f"[smoke] forward displacement = {dist:.3f} m over {N} steps in {time.time() - t0:.1f}s wall", flush=True)
print(
    "[smoke] SUCCESS: Spot loaded its policy and walked forward."
    if dist > 0.3
    else "[smoke] WARNING: Spot moved less than expected; check policy/physics.",
    flush=True,
)

simulation_app.close()
print("[smoke] done.", flush=True)
