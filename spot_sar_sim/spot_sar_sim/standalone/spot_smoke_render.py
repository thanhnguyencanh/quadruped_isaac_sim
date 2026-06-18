"""RENDERED Spot smoke test — validates the RTX render path (needed before camera sensors).

Mirrors the shipped ``standalone_examples/.../spot_standalone.py`` API for Isaac Sim 6.0
(``SimulationManager`` + a physics callback, a torch ``base_command``, and
``SpotFlatTerrainPolicy(prim_path, position)`` with NO ``name=`` kwarg), but runs HEADLESS,
pins the local asset root, and exits after a fixed number of rendered steps.

WARNING: the FIRST run is slow (often 10–20 min) because RTX compiles the full shader set
for the environment on a cold Blackwell driver cache; subsequent runs are faster. For routine
locomotion validation use ``spot_smoke_test.py`` (physics only, ~3 s) instead.

Run with::

    scripts/run_isaac.sh spot_sar_sim/spot_sar_sim/standalone/spot_smoke_render.py
"""
import os

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import carb

LOCAL_ASSETS = os.environ.get("ISAAC_ASSETS", os.path.expanduser("~/isaacsim_assets"))
carb.settings.get_settings().set("/persistent/isaac/asset_root/default", LOCAL_ASSETS)

import omni.timeline
from isaacsim.core.deprecation_manager import import_module
from isaacsim.core.experimental.utils.stage import define_prim
from isaacsim.core.rendering_manager import RenderingManager
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.simulation_manager.impl.isaac_events import IsaacEvents
from isaacsim.robot.policy.examples.robots import SpotFlatTerrainPolicy
from isaacsim.storage.native import get_assets_root_path

torch = import_module("torch")

DEVICE = "cpu"
first_step = True
reset_needed = False


def on_physics_step(step_size, context) -> None:
    global first_step, reset_needed
    if first_step:
        spot.initialize()
        first_step = False
    elif reset_needed:
        reset_needed = False
        first_step = True
    else:
        spot.forward(step_size, base_command)


assets_root_path = get_assets_root_path()
print(f"[smoke] resolved assets_root_path = {assets_root_path}", flush=True)
if assets_root_path is None:
    assets_root_path = LOCAL_ASSETS

prim = define_prim("/World/Ground", "Xform")
prim.GetReferences().AddReference(assets_root_path + "/Isaac/Environments/Grid/default_environment.usd")
define_prim("/World/PhysicsScene", "PhysicsScene")

RenderingManager.set_dt(8.0 / 200.0)
SimulationManager.set_physics_sim_device(DEVICE)
SimulationManager.set_physics_dt(1.0 / 200.0)

spot = SpotFlatTerrainPolicy(prim_path="/World/Spot", position=[0, 0, 0.8])
base_command = torch.zeros(3, device=DEVICE)

SimulationManager.register_callback(on_physics_step, IsaacEvents.POST_PHYSICS_STEP)

omni.timeline.get_timeline_interface().play()
simulation_app.update()

i = 0
MAX_STEPS = 250
while simulation_app.is_running():
    simulation_app.update()
    if SimulationManager.is_simulating():
        if i >= 10:
            base_command = torch.tensor([1.0, 0.0, 0.0], device=DEVICE)
        i += 1
        if i >= MAX_STEPS:
            print(f"[smoke] end pose = {spot.robot.get_world_poses()[0]}", flush=True)
            print("[smoke] SUCCESS: rendered simulation ran and Spot was stepped.", flush=True)
            break

simulation_app.close()
print("[smoke] done.", flush=True)
