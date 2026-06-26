"""spot_sar_sim — Phase 2: Spot with an RGB-D camera + ROS 2 perception stream.

Superset of the Phase 1 /cmd_vel app (spot_cmd_vel_app.py): boots Isaac Sim 6.0,
spawns Spot with the flat-terrain policy, drives it from /cmd_vel, and bridges the same
locomotion topics — PLUS a forward-facing RGB-D camera mounted on Spot's body:

  locomotion (unchanged from Phase 1):
    * subscribes  /cmd_vel                    (geometry_msgs/Twist)
    * publishes   /clock /joint_states /odom /tf
  perception (new in Phase 2):
    * publishes   /camera/rgb/image_raw       (sensor_msgs/Image,   rgb8)
    * publishes   /camera/depth/image_raw     (sensor_msgs/Image,   32FC1 metres)
    * publishes   /camera/rgb/camera_info     (sensor_msgs/CameraInfo)

A bright-orange "victim" marker cube is placed ~2.5 m ahead so the downstream
victim-detection node (spot_sar_perception/detector_node.py) has a real target.

WHY A SEPARATE APP (not an edit of spot_cmd_vel_app.py): the camera needs RENDERING ON
(a render product produces no image in the physics-only path), which changes the render
contract. Keeping the proven Phase 1 app untouched de-risks locomotion.

LOAD-BEARING PATTERNS preserved from Phase 1 (do not reorder):
  asset-root pin -> omni_cache redirect -> enable bridge -> import rclpy -> rclpy.init()
  -> build graphs -> play() -> update() -> spot.initialize() -> update() -> register
  FORWARD-ONLY physics callback. And: NO ROS2Context node in any graph (in-process rclpy
  owns the default DDS context; a ROS2Context node would create a conflicting one and the
  publishers would vanish from the wire).

OPTICAL FRAME: Isaac's USD camera looks down its local -Z, but ROS2CameraHelper publishes
pixels + intrinsics in the ROS optical convention (z-fwd, x-right, y-down). Images are
stamped frameId=camera_optical_frame; the base_link->camera_link->camera_optical_frame TF
legs are published as STATIC transforms from the launch file (deterministic, decoupled from
the USD prim name).

Run:  scripts/run_isaac.sh spot_sar_sim/spot_sar_sim/standalone/spot_perception_app.py
Verify (SYSTEM ROS 2 Jazzy shell, SAME ROS_DOMAIN_ID):
      ros2 topic hz /camera/rgb/image_raw
      ros2 topic echo /camera/rgb/camera_info --once
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sar_scene import build_sar_scene, build_floor_scene  # shared SAR environments

# sar_floor (the multi-room floor plan) lives in spot_sar_planning so the grounding node + planner
# share it; add its source dir so build_floor_scene() can import it under Isaac's python.
_REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))
sys.path.insert(0, os.path.join(_REPO, "spot_sar_planning", "spot_sar_planning"))

from isaacsim import SimulationApp

parser = argparse.ArgumentParser(description="Spot RGB-D camera + ROS 2 perception app")
parser.add_argument("--gui", action="store_true", help="show the GUI window (default headless)")
parser.add_argument("--floor", action="store_true",
                    help="build the multi-room floor (walls + openable doors) instead of the single room")
parser.add_argument("--no-humans", dest="humans", action="store_false",
                    help="use orange box victims (HSV detector) instead of human figures (YOLO)")
parser.add_argument("--device", choices=["cpu", "cuda"], default="cpu", help="physics/policy device")
parser.add_argument("--steps", type=int, default=0, help="auto-exit after N render frames (0 = run forever)")
args, _ = parser.parse_known_args()

# Rendering MUST run for the camera render product to produce images; headless only drops the
# GUI window, not the renderer. Keep the swapchain modest (8 GB VRAM tier).
simulation_app = SimulationApp(
    {"headless": not args.gui, "width": 640, "height": 480, "renderer": "RayTracedLighting"}
)

import carb

LOCAL_ASSETS = os.environ.get("ISAAC_ASSETS", os.path.expanduser("~/isaacsim_assets"))
carb.settings.get_settings().set("/persistent/isaac/asset_root/default", LOCAL_ASSETS)

# Redirect Kit's ${omni_cache} (stale Docker path) to a writable dir BEFORE enabling the bridge,
# else OGN node-cache writes EACCES and ROS 2 node registration intermittently aborts.
_OMNI_CACHE = os.environ.get("OMNI_CACHE_DIR", os.path.expanduser("~/.cache/isaacsim_omni_cache"))
os.makedirs(_OMNI_CACHE, exist_ok=True)
try:
    carb.tokens.get_tokens_interface().set_value("omni_cache", _OMNI_CACHE)
    print(f"[perception] omni_cache -> {_OMNI_CACHE}", flush=True)
except Exception as e:  # noqa: BLE001
    print(f"[perception] could not set omni_cache token: {e}", flush=True)

from isaacsim.core.utils.extensions import enable_extension

enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String

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
from pxr import Gf, UsdGeom

torch = import_module("torch")
import warp as wp

VX_LIM, VY_LIM, WZ_LIM = 1.5, 1.0, 1.0  # flat-terrain policy trained range
SPOT_PRIM = "/World/Spot"  # articulation root (for joint-state reads)
BASE_LINK_PRIM = "/World/Spot/body"  # Spot's base rigid body (odom chassis frame)
GRAPH = "/ActionGraph"  # locomotion publish-only graph (Phase 1)
CAM_GRAPH = "/CameraGraph"  # camera publish graph (Phase 2), own OnPlaybackTick

# camera mount + frames
CAMERA_PRIM = "/World/Spot/body/front_camera"  # child of body -> moves with the chassis
CAMERA_OPTICAL_FRAME = "camera_optical_frame"  # z-fwd,x-right,y-down; image + K live here
CAM_W, CAM_H = 640, 480
RGB_TOPIC = "/camera/rgb/image_raw"
DEPTH_TOPIC = "/camera/depth/image_raw"
CAM_INFO_TOPIC = "/camera/rgb/camera_info"

PHYSICS_HZ = 200.0


class CmdVelNode(Node):
    """In-process ROS 2 subscriber: latest /cmd_vel -> (vx, vy, wz)."""

    def __init__(self):
        super().__init__("spot_cmd_vel")
        self.vx = self.vy = self.wz = 0.0
        self.create_subscription(Twist, "/cmd_vel", self._cb, 10)

    def _cb(self, msg: Twist):
        self.vx, self.vy, self.wz = msg.linear.x, msg.linear.y, msg.angular.z


class DoorNode(Node):
    """In-process door bus (floor demo). Subscribes /door_cmd (std_msgs/String) and drives the
    named slab toward open or closed by mutating its cached translate op each frame, then publishes
    /door_states on a TRANSIENT_LOCAL (latched) publisher so late-joining subscribers still see it.

    /door_cmd payload:  "<door_id>"        -> open  (e.g. "door_bc"; back-compat)
                        "<door_id> open"   -> open
                        "<door_id> close"  -> close
    /door_states payload: "<door_id> open" | "<door_id> closed"  (published once the slab arrives)

    door_handles[id] = {prim, closed(xyz), open(xyz), op (translate XformOp), frac}. frac: 0=closed..1=open.
    """

    def __init__(self, door_handles):
        super().__init__("spot_doors")
        self.handles = door_handles
        self.targets = {d: 0.0 for d in door_handles}  # desired frac per door (0 closed .. 1 open)
        latched = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/door_cmd", self._on_cmd, 10)
        self.state_pub = self.create_publisher(String, "/door_states", latched)

    def _on_cmd(self, msg: String):
        parts = msg.data.split()
        if not parts:
            return
        did = parts[0].strip()
        action = parts[1].strip().lower() if len(parts) > 1 else "open"
        if did in self.handles:
            self.targets[did] = 0.0 if action == "close" else 1.0

    def step(self, rate):
        """Lerp each slab toward its target frac (both directions); announce when it arrives."""
        for did, h in self.handles.items():
            tgt = self.targets[did]
            f = h["frac"]
            if abs(f - tgt) < 1e-6:
                continue
            f = min(tgt, f + rate) if f < tgt else max(tgt, f - rate)
            h["frac"] = f
            cx, cy, cz = h["closed"]
            oz = h["open"][2]
            h["op"].Set(Gf.Vec3d(cx, cy, cz + (oz - cz) * f))
            if abs(f - tgt) < 1e-6:  # just arrived
                state = "open" if tgt >= 1.0 else "closed"
                self.state_pub.publish(String(data=f"{did} {state}"))
                self.get_logger().info(f"door '{did}' fully {state}")


# ---------------------------------------------------------------- scene + robot
assets_root_path = get_assets_root_path()
print(f"[perception] assets_root_path = {assets_root_path}", flush=True)

define_prim("/World/Ground", "Xform").GetReferences().AddReference(
    assets_root_path + "/Isaac/Environments/Grid/default_environment.usd"
)
define_prim("/World/PhysicsScene", "PhysicsScene")

RenderingManager.set_dt(8.0 / PHYSICS_HZ)  # ~25 Hz render (caps GPU load for 8 GB VRAM)
SimulationManager.set_physics_sim_device(args.device)
SimulationManager.set_physics_dt(1.0 / PHYSICS_HZ)

spot = SpotFlatTerrainPolicy(prim_path=SPOT_PRIM, position=[0.0, 0.0, 0.8])
base_command = torch.zeros(3, device=args.device)  # [vx, vy, wz]; mutated in place

# ---- SAR environment: single walled room OR the multi-room floor with openable doors ----
if args.floor:
    _victims, _door_handles = build_floor_scene(humans=args.humans, assets_root_path=assets_root_path)
    for _did, _h in _door_handles.items():
        # cache the slab's translate op (ordered op [0]) so DoorNode mutates it each frame
        _h["op"] = UsdGeom.Xformable(_h["prim"]).GetOrderedXformOps()[0]
        _h["frac"] = 0.0
    print(f"[perception] FLOOR scene: {len(_victims)} victim(s) (humans={args.humans}); "
          f"doors={list(_door_handles)}", flush=True)
else:
    _victims = build_sar_scene(humans=args.humans, assets_root_path=assets_root_path)
    _door_handles = {}
    print(f"[perception] SAR scene: {len(_victims)} victim(s) (humans={args.humans}) at {_victims}", flush=True)

# ---- forward-facing RGB-D camera, child of the body so it tracks the chassis ----
cam_prim = define_prim(CAMERA_PRIM, "Camera")
_cx = UsdGeom.Xformable(cam_prim)
_cx.ClearXformOpOrder()
_cx.AddTranslateOp().Set(Gf.Vec3d(0.35, 0.0, 0.10))  # 0.35 m forward, 0.10 m up on the body
# USD camera looks down its local -Z. Orient it so -Z -> body +X (look forward) and +Y -> body
# +Z (up): quaternion (w,x,y,z) = (0.5, 0.5, -0.5, -0.5). This matches the static
# camera_link->camera_optical_frame TF, so back-projected victim poses stay consistent.
_cx.AddOrientOp().Set(Gf.Quatf(0.5, 0.5, -0.5, -0.5))
_cam = UsdGeom.Camera(cam_prim)
_cam.GetFocalLengthAttr().Set(18.0)  # with 24 mm horiz aperture -> ~67 deg HFOV
_cam.GetHorizontalApertureAttr().Set(24.0)
_cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.05, 100.0))


def _base_xyz():
    try:
        pos, _ = spot.robot.get_world_poses()
        return wp.to_torch(pos).cpu().numpy().reshape(-1)[:3]
    except Exception:  # noqa: BLE001
        return np.array([float("nan")] * 3)


# rclpy BEFORE building graphs (publishers attach to the default context that rclpy owns).
rclpy.init()

# ---------------------------------------------------------------- locomotion graph (PUBLISH-ONLY)
P = usdrt.Sdf.Path
keys = og.Controller.Keys
og.Controller.edit(
    {"graph_path": GRAPH, "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("OnTick", "omni.graph.action.OnPlaybackTick"),
            # NO ROS2Context node (see module docstring).
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

# ---------------------------------------------------------------- camera graph (PUBLISH-ONLY)
# IsaacCreateRenderProduct -> ROS2CameraHelper (rgb) + ROS2CameraHelper (depth) +
# ROS2CameraInfoHelper. renderProductPath is an OUTPUT token fanned out to all helpers.
# No ROS2Context node: helpers default to context 0 (the shared default context rclpy owns).
og.Controller.edit(
    {"graph_path": CAM_GRAPH, "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("CamTick", "omni.graph.action.OnPlaybackTick"),
            ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
            ("PubRGB", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("PubDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("PubCamInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
        ],
        keys.SET_VALUES: [
            ("RenderProduct.inputs:cameraPrim", [P(CAMERA_PRIM)]),
            ("RenderProduct.inputs:width", CAM_W),
            ("RenderProduct.inputs:height", CAM_H),
            ("PubRGB.inputs:type", "rgb"),
            ("PubRGB.inputs:topicName", RGB_TOPIC),
            ("PubRGB.inputs:frameId", CAMERA_OPTICAL_FRAME),
            ("PubDepth.inputs:type", "depth"),
            ("PubDepth.inputs:topicName", DEPTH_TOPIC),
            ("PubDepth.inputs:frameId", CAMERA_OPTICAL_FRAME),
            ("PubCamInfo.inputs:topicName", CAM_INFO_TOPIC),
            ("PubCamInfo.inputs:frameId", CAMERA_OPTICAL_FRAME),
        ],
        keys.CONNECT: [
            ("CamTick.outputs:tick", "RenderProduct.inputs:execIn"),
            ("RenderProduct.outputs:execOut", "PubRGB.inputs:execIn"),
            ("RenderProduct.outputs:execOut", "PubDepth.inputs:execIn"),
            ("RenderProduct.outputs:execOut", "PubCamInfo.inputs:execIn"),
            ("RenderProduct.outputs:renderProductPath", "PubRGB.inputs:renderProductPath"),
            ("RenderProduct.outputs:renderProductPath", "PubDepth.inputs:renderProductPath"),
            ("RenderProduct.outputs:renderProductPath", "PubCamInfo.inputs:renderProductPath"),
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

cmd_node = CmdVelNode()  # rclpy already initialized before the graphs
door_node = DoorNode(_door_handles) if args.floor else None  # door bus (floor demo only)
DOOR_RATE = 0.02  # fraction of slab travel per loop tick (~1-2 s to fully open)

print(
    "[perception] publishing /clock /joint_states /odom /tf "
    "+ /camera/rgb/image_raw /camera/depth/image_raw /camera/rgb/camera_info; "
    "subscribing /cmd_vel" + ("; doors on /door_cmd -> /door_states" if door_node else "") + " (rclpy)",
    flush=True,
)
frame = 0
while simulation_app.is_running():
    simulation_app.update()
    rclpy.spin_once(cmd_node, timeout_sec=0.0)
    if door_node is not None:
        rclpy.spin_once(door_node, timeout_sec=0.0)
        door_node.step(DOOR_RATE)
    if not SimulationManager.is_simulating():
        continue

    base_command[0] = float(np.clip(cmd_node.vx, -VX_LIM, VX_LIM))
    base_command[1] = float(np.clip(cmd_node.vy, -VY_LIM, VY_LIM))
    base_command[2] = float(np.clip(cmd_node.wz, -WZ_LIM, WZ_LIM))

    if frame % 100 == 0:
        bc = base_command.cpu().numpy() if hasattr(base_command, "cpu") else np.asarray(base_command)
        print(f"[perception] frame={frame} cmd={np.round(bc, 3)} base_xyz={np.round(_base_xyz(), 3)}", flush=True)

    frame += 1
    if args.steps and frame >= args.steps:
        print(f"[perception] reached --steps {args.steps}, exiting.", flush=True)
        break

cmd_node.destroy_node()
if door_node is not None:
    door_node.destroy_node()
rclpy.shutdown()
simulation_app.close()
print("[perception] done.", flush=True)
