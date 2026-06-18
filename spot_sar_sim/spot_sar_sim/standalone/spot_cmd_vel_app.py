"""spot_sar_sim — Phase 1: drive Spot from ROS 2 /cmd_vel and stream sim state.

Boots Isaac Sim 6.0 headless, spawns Boston Dynamics Spot with the shipped flat-terrain
RL policy, and bridges it to ROS 2:
  * subscribes  /cmd_vel       (geometry_msgs/Twist)    -> [vx, vy, wz] policy command
  * publishes   /clock         (rosgraph_msgs/Clock)     sim time
  * publishes   /joint_states  (sensor_msgs/JointState)  Spot's 12 leg joints
  * publishes   /odom          (nav_msgs/Odometry)       base pose + body-frame twist
  * publishes   /tf            (tf2_msgs/TFMessage)      odom -> base_link

Two hard-won lessons (both verified against the installed Isaac Sim 6.0 source):

  CONTROL INIT ORDERING — the policy only actuates Spot if spot.initialize() (which sets
  position-control mode + PD gains on the live PhysX articulation view) runs AFTER the
  timeline is playing and the physics view exists, and OUTSIDE the physics callback. If
  initialize() is deferred into the first POST_PHYSICS_STEP callback, the gains are not yet
  live when forward() writes joint targets -> the writes are no-ops -> Spot collapses with
  no error. So: play() -> update() -> spot.initialize() -> update() -> register a
  FORWARD-ONLY callback. (Pattern from exts/.../policy/examples/tests/test_spot.py.)

  /cmd_vel VIA rclpy, NOT OmniGraph — an OmniGraph ROS2SubscribeTwist node's outputs cannot
  be reliably read back into Python via og.Controller.get in a standalone loop (they stay
  zero). So /cmd_vel uses an in-process rclpy subscriber (Isaac's rclpy, available after the
  bridge is enabled); the OmniGraph is publish-only.

Run:  scripts/run_isaac.sh spot_sar_sim/spot_sar_sim/standalone/spot_cmd_vel_app.py
Verify from a SYSTEM ROS 2 (Jazzy) shell with the SAME ROS_DOMAIN_ID:
      ros2 topic echo /clock --once
      ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.6}}'
"""
import argparse
import os

from isaacsim import SimulationApp

parser = argparse.ArgumentParser(description="Spot /cmd_vel ROS 2 bridge app")
parser.add_argument("--gui", action="store_true", help="show the GUI window (default headless)")
parser.add_argument("--device", choices=["cpu", "cuda"], default="cpu", help="physics/policy device")
parser.add_argument("--steps", type=int, default=0, help="auto-exit after N render frames (0 = run forever)")
args, _ = parser.parse_known_args()

simulation_app = SimulationApp({"headless": not args.gui, "width": 1280, "height": 720})

import carb

LOCAL_ASSETS = os.environ.get("ISAAC_ASSETS", os.path.expanduser("~/isaacsim_assets"))
carb.settings.get_settings().set("/persistent/isaac/asset_root/default", LOCAL_ASSETS)

# The install's ${omni_cache} token points at a stale Docker path (/home/isaac/...), so Kit's
# on-demand OGN node-cache write fails with EACCES and ROS 2 node registration intermittently
# aborts -> bridge startup hangs/crashes. Redirect the cache to a writable dir BEFORE enabling
# the bridge so node generation can persist.
_OMNI_CACHE = os.environ.get("OMNI_CACHE_DIR", os.path.expanduser("~/.cache/isaacsim_omni_cache"))
os.makedirs(_OMNI_CACHE, exist_ok=True)
try:
    carb.tokens.get_tokens_interface().set_value("omni_cache", _OMNI_CACHE)
    print(f"[bridge] omni_cache -> {_OMNI_CACHE}", flush=True)
except Exception as e:  # noqa: BLE001
    print(f"[bridge] could not set omni_cache token: {e}", flush=True)

# Bridge must be enabled before importing rclpy / using ROS 2 OmniGraph nodes.
from isaacsim.core.utils.extensions import enable_extension

enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

# rclpy is only importable/usable after the bridge extension is loaded.
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

import numpy as np
import omni.graph.core as og
import omni.timeline
import usdrt
from isaacsim.core.deprecation_manager import import_module
from isaacsim.core.experimental.utils.stage import define_prim
from isaacsim.core.rendering_manager import RenderingManager
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.simulation_manager.impl.isaac_events import IsaacEvents
from isaacsim.robot.policy.examples.robots import SpotFlatTerrainPolicy
from isaacsim.storage.native import get_assets_root_path

torch = import_module("torch")
import warp as wp

VX_LIM, VY_LIM, WZ_LIM = 1.5, 1.0, 1.0  # flat-terrain policy trained range
SPOT_PRIM = "/World/Spot"  # articulation root (for joint-state reads)
BASE_LINK_PRIM = "/World/Spot/body"  # Spot's base rigid body (odom chassis frame)
GRAPH = "/ActionGraph"
PHYSICS_HZ = 200.0


class CmdVelNode(Node):
    """In-process ROS 2 subscriber: latest /cmd_vel -> (vx, vy, wz)."""

    def __init__(self):
        super().__init__("spot_cmd_vel")
        self.vx = self.vy = self.wz = 0.0
        self.create_subscription(Twist, "/cmd_vel", self._cb, 10)

    def _cb(self, msg: Twist):
        self.vx, self.vy, self.wz = msg.linear.x, msg.linear.y, msg.angular.z


# ---------------------------------------------------------------- scene + robot
assets_root_path = get_assets_root_path()
print(f"[bridge] assets_root_path = {assets_root_path}", flush=True)

define_prim("/World/Ground", "Xform").GetReferences().AddReference(
    assets_root_path + "/Isaac/Environments/Grid/default_environment.usd"
)
define_prim("/World/PhysicsScene", "PhysicsScene")

RenderingManager.set_dt(8.0 / PHYSICS_HZ)
SimulationManager.set_physics_sim_device(args.device)
SimulationManager.set_physics_dt(1.0 / PHYSICS_HZ)

spot = SpotFlatTerrainPolicy(prim_path=SPOT_PRIM, position=[0.0, 0.0, 0.8])
base_command = torch.zeros(3, device=args.device)  # [vx, vy, wz]; mutated in place


def _base_xyz():
    try:
        pos, _ = spot.robot.get_world_poses()
        return wp.to_torch(pos).cpu().numpy().reshape(-1)[:3]
    except Exception:  # noqa: BLE001
        return np.array([float("nan")] * 3)


# Initialize rclpy BEFORE building the bridge graph (clock.py order): the ROS context must
# exist before the graph's ROS2Context node / publishers attach, or they don't appear on the wire.
rclpy.init()

# ---------------------------------------------------------------- ROS 2 bridge graph (PUBLISH-ONLY)
P = usdrt.Sdf.Path
keys = og.Controller.Keys
og.Controller.edit(
    {"graph_path": GRAPH, "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("OnTick", "omni.graph.action.OnPlaybackTick"),
            # NOTE: no ROS2Context node — when the app also runs in-process rclpy (for /cmd_vel),
            # a ROS2Context node creates a second DDS context that conflicts and makes the
            # publishers invisible on the wire. Like clock.py, the publishers use the default
            # context (domain from ROS_DOMAIN_ID env), which the in-process rclpy shares.
            ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("ReadJoints", "isaacsim.sensors.physics.IsaacReadJointState"),
            ("PubJoints", "isaacsim.ros2.bridge.ROS2PublishJointState"),
            ("PubClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ("ComputeOdom", "isaacsim.core.nodes.IsaacComputeOdometry"),
            ("PubOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
            ("TFOdom2Base", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
        ],
        keys.SET_VALUES: [
            ("ReadSimTime.inputs:resetOnStop", True),
            ("ReadJoints.inputs:prim", [P(SPOT_PRIM)]),
            ("PubJoints.inputs:topicName", "joint_states"),
            ("PubClock.inputs:topicName", "/clock"),
            ("ComputeOdom.inputs:chassisPrim", [P(BASE_LINK_PRIM)]),
            ("PubOdom.inputs:topicName", "odom"),
            ("PubOdom.inputs:odomFrameId", "odom"),
            ("PubOdom.inputs:chassisFrameId", "base_link"),
            ("TFOdom2Base.inputs:topicName", "/tf"),
            ("TFOdom2Base.inputs:parentFrameId", "odom"),
            ("TFOdom2Base.inputs:childFrameId", "base_link"),
        ],
        keys.CONNECT: [
            ("OnTick.outputs:tick", "ReadJoints.inputs:execIn"),
            ("ReadJoints.outputs:execOut", "PubJoints.inputs:execIn"),
            ("OnTick.outputs:tick", "PubClock.inputs:execIn"),
            ("OnTick.outputs:tick", "ComputeOdom.inputs:execIn"),
            ("ComputeOdom.outputs:execOut", "PubOdom.inputs:execIn"),
            ("OnTick.outputs:tick", "TFOdom2Base.inputs:execIn"),
            ("ReadSimTime.outputs:simulationTime", "PubClock.inputs:timeStamp"),
            ("ReadSimTime.outputs:simulationTime", "PubOdom.inputs:timeStamp"),
            ("ReadSimTime.outputs:simulationTime", "TFOdom2Base.inputs:timeStamp"),
            ("ReadJoints.outputs:jointNames", "PubJoints.inputs:jointNames"),
            ("ReadJoints.outputs:jointPositions", "PubJoints.inputs:jointPositions"),
            ("ReadJoints.outputs:jointVelocities", "PubJoints.inputs:jointVelocities"),
            ("ReadJoints.outputs:jointEfforts", "PubJoints.inputs:jointEfforts"),
            ("ReadJoints.outputs:jointDofTypes", "PubJoints.inputs:jointDofTypes"),
            ("ReadJoints.outputs:stageMetersPerUnit", "PubJoints.inputs:stageMetersPerUnit"),
            ("ReadJoints.outputs:sensorTime", "PubJoints.inputs:sensorTime"),
            ("ComputeOdom.outputs:position", "PubOdom.inputs:position"),
            ("ComputeOdom.outputs:orientation", "PubOdom.inputs:orientation"),
            ("ComputeOdom.outputs:linearVelocity", "PubOdom.inputs:linearVelocity"),
            ("ComputeOdom.outputs:angularVelocity", "PubOdom.inputs:angularVelocity"),
            ("ComputeOdom.outputs:position", "TFOdom2Base.inputs:translation"),
            ("ComputeOdom.outputs:orientation", "TFOdom2Base.inputs:rotation"),
        ],
    },
)
simulation_app.update()

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

cmd_node = CmdVelNode()  # rclpy already initialized before the graph

print("[bridge] publishing /clock /joint_states /odom /tf; subscribing /cmd_vel (rclpy)", flush=True)
frame = 0
while simulation_app.is_running():
    simulation_app.update()
    rclpy.spin_once(cmd_node, timeout_sec=0.0)
    if not SimulationManager.is_simulating():
        continue

    base_command[0] = float(np.clip(cmd_node.vx, -VX_LIM, VX_LIM))
    base_command[1] = float(np.clip(cmd_node.vy, -VY_LIM, VY_LIM))
    base_command[2] = float(np.clip(cmd_node.wz, -WZ_LIM, WZ_LIM))

    if frame % 100 == 0:
        bc = base_command.cpu().numpy() if hasattr(base_command, "cpu") else np.asarray(base_command)
        print(f"[bridge] frame={frame} cmd={np.round(bc, 3)} base_xyz={np.round(_base_xyz(), 3)}", flush=True)

    frame += 1
    if args.steps and frame >= args.steps:
        print(f"[bridge] reached --steps {args.steps}, exiting.", flush=True)
        break

cmd_node.destroy_node()
rclpy.shutdown()
simulation_app.close()
print("[bridge] done.", flush=True)
