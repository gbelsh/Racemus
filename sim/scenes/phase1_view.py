"""Phase 1, Step 1 — base + PAN joint.

Build the bottom of the turret: a fixed base and one link that rotates left/right (pans) about
the vertical Z axis, driven like a servo. Then command the pan to +45 deg, step the physics, and
read the angle back to PROVE the joint actually moved. No tilt, no camera yet — one joint only.

Ground truth is written to sim/scenes/_output/step1_status.txt (Kit swallows print()).

Run: D:\\source\\Racemus\\.venv\\Scripts\\python.exe sim\\scenes\\phase1_step1_pan.py
"""

from isaacsim import SimulationApp

sim_app = SimulationApp({"headless": False})

import math
import os
import sys

import numpy as np
from pxr import UsdGeom, UsdPhysics, PhysxSchema, Gf, UsdLux

# Make our config importable: this file is in sim/scenes/, turret_config.py is in sim/.
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import turret_config as cfg

from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction

import omni.replicator.core as rep
import PIL.Image

out_dir = os.path.join(os.path.dirname(__file__), "_output")
os.makedirs(out_dir, exist_ok=True)
status_path = os.path.join(out_dir, "step1_status.txt")
open(status_path, "w", encoding="utf-8").close()


def log(msg):
    with open(status_path, "a", encoding="utf-8") as f:
        f.write(msg + "\n")


log("booted SimulationApp")

# --- World = Isaac's simulation manager (owns the stage, physics, stepping) ----------------
world = World(stage_units_in_meters=1.0)
stage = world.stage

# A PhysicsScene must exist for any physics to run (gravity, solver, etc.).
UsdPhysics.Scene.Define(stage, "/physicsScene")

# Sky dome: a DomeLight is an infinite sphere enclosing the scene that both lights it and
# forms the background. With no HDRI texture it emits a flat color — we tint it sky-blue so
# the rendered background reads as sky. (Realistic sky/cloud/sun variation is Phase 2.)
dome = UsdLux.DomeLight.Define(stage, "/World/SkyDome")
dome.CreateIntensityAttr(1000.0)
dome.CreateColorAttr(Gf.Vec3f(0.35, 0.55, 0.90))
log("built sky dome")

# --- Camera -------------------------------------------------------------------
camera = rep.create.camera(
    position=(2.5, -2.0, 1.5),  # Where the eye sits.
    look_at=(0.0, 0.20, 0.35),  # Point at which the camera aims (turrets middle)
)

render_product = rep.create.render_product(camera, (1280, 720))
rgb_annotator = rep.AnnotatorRegistry.get_annotator("rgb")
rgb_annotator.attach(render_product)


# --- The turret as an articulation ---------------------------------------------------------
# /World/Turret is the articulation ROOT: the wrapper that makes base+joint+link one mechanism.
turret = UsdGeom.Xform.Define(stage, "/World/Turret")
UsdPhysics.ArticulationRootAPI.Apply(turret.GetPrim())
PhysxSchema.PhysxArticulationAPI.Apply(turret.GetPrim())

# Base link — a short cylinder, rigid body, that we pin to the world so it never moves.
base = UsdGeom.Cylinder.Define(stage, "/World/Turret/base")
base.CreateRadiusAttr(0.15)
base.CreateHeightAttr(0.20)
base.CreateAxisAttr(UsdGeom.Tokens.z)
UsdGeom.Xformable(base).AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.10))
UsdPhysics.CollisionAPI.Apply(base.GetPrim())
UsdPhysics.RigidBodyAPI.Apply(base.GetPrim())

# Fixed joint world->base: body0 empty = "the world", so the base is bolted down.
fixed = UsdPhysics.FixedJoint.Define(stage, "/World/Turret/base_to_world")
fixed.CreateBody1Rel().SetTargets([base.GetPrim().GetPath()])

# Pan link — a bar offset in +Y so its rotation about Z is obvious. Rigid body.
pan = UsdGeom.Cube.Define(stage, "/World/Turret/pan")
pan.CreateSizeAttr(1.0)
# Scale the unit cube into a thin bar (x=0.1, y=0.5, z=0.1) and lift it above the base.
xf = UsdGeom.Xformable(pan)
xf.AddTranslateOp().Set(Gf.Vec3d(0, 0.20, 0.30))
xf.AddScaleOp().Set(Gf.Vec3f(0.10, 0.50, 0.10))
UsdPhysics.CollisionAPI.Apply(pan.GetPrim())
UsdPhysics.RigidBodyAPI.Apply(pan.GetPrim())

# Revolute joint base->pan, axis Z = the pan hinge.
revolute = UsdPhysics.RevoluteJoint.Define(stage, "/World/Turret/pan_joint")
revolute.CreateBody0Rel().SetTargets([base.GetPrim().GetPath()])
revolute.CreateBody1Rel().SetTargets([pan.GetPrim().GetPath()])
revolute.CreateAxisAttr("Z")
# The pivot point expressed in each body's LOCAL frame (they must coincide in world space).
# base center is at z=0.10, pivot at z=0.20  -> local (0,0,0.10)
revolute.CreateLocalPos0Attr(Gf.Vec3f(0, 0, 0.10))
# pan center is at (0,0.20,0.30), pivot at (0,0,0.20) -> local (0,-0.20,-0.10)
revolute.CreateLocalPos1Attr(Gf.Vec3f(0, -0.20, -0.10))
# Joint limits straight from your config (RevoluteJoint limits are in degrees).
revolute.CreateLowerLimitAttr(float(cfg.PAN_RANGE_DEG[0]))
revolute.CreateUpperLimitAttr(float(cfg.PAN_RANGE_DEG[1]))

# Drive = the "servo motor" on the pan joint. Position drive: give it a target angle, it applies
# torque to reach + hold it. Stiffness/damping are the PID-ish gains that make it firm.
drive = UsdPhysics.DriveAPI.Apply(revolute.GetPrim(), "angular")
drive.CreateTypeAttr("force")
drive.CreateStiffnessAttr(1e6)
drive.CreateDampingAttr(1e5)
drive.CreateTargetPositionAttr(0.0)

log("built base + pan joint + drive")

# --- Tilt link + joint + drive (mounted on the PAN link - the chain) ----------------------
tilt = UsdGeom.Cube.Define(stage, "/World/Turret/tilt") # make a cube prim
tilt.CreateSizeAttr(1.0)                                # set its size attr
xf_t = UsdGeom.Xformable(tilt)                          # make it movable
xf_t.AddTranslateOp().Set(Gf.Vec3d(0, 0.45, 0.50))      # place it (a post above the bar)
xf_t.AddScaleOp().Set(Gf.Vec3f(0.10, 0.10, 0.4))        # stretch to a thin post
UsdPhysics.CollisionAPI.Apply(tilt.GetPrim())
UsdPhysics.RigidBodyAPI.Apply(tilt.GetPrim())

tilt_joint = UsdPhysics.RevoluteJoint.Define(stage, "/World/Turret/tilt_joint")
tilt_joint.CreateBody0Rel().SetTargets([pan.GetPrim().GetPath()])   # Parent based on Pan  
tilt_joint.CreateBody1Rel().SetTargets([tilt.GetPrim().GetPath()])  # Pan Points at tilt  
tilt_joint.CreateAxisAttr("X")                                      # About the Y axis
tilt_joint.CreateLocalPos0Attr(Gf.Vec3f(0, 0.25, 0.0))              # Pivot in pan frame
tilt_joint.CreateLocalPos1Attr(Gf.Vec3f(0, 0.0, -0.20))             # Pivot for Tilts Frame
tilt_joint.CreateLowerLimitAttr(float(cfg.TILT_RANGE_DEG[0]))       # Lower Limit Tilt Range
tilt_joint.CreateUpperLimitAttr(float(cfg.TILT_RANGE_DEG[1]))       # Upper limit Tilt Range

tilt_drive = UsdPhysics.DriveAPI.Apply(tilt_joint.GetPrim(), "angular") # The Servo
tilt_drive.CreateTypeAttr("force")
tilt_drive.CreateStiffnessAttr(1e6)
tilt_drive.CreateDampingAttr(1e5)
tilt_drive.CreateTargetPositionAttr(0.0)


# --- Simulate ------------------------------------------------------------------------------
world.reset()  # initialize physics + the articulation

# Wrap the articulation so we can command/read joints from Python.
art = SingleArticulation("/World/Turret")
art.initialize()
log(f"articulation dof names: {art.dof_names}")

pan_target_deg = 30.0
tilt_target_deg = 45.0
targets = np.array([
    math.radians(pan_target_deg),
    math.radians(tilt_target_deg),
])

art.apply_action(ArticulationAction(joint_positions=targets))
log(f"commanded pan->{pan_target_deg}, tilt->{tilt_target_deg}")

for _ in range(180):
    world.step(render=False)

reached = art.get_joint_positions()
log(f"DEBUG reached: type={type(reached)}  value={reached}")   # <-- temporary probe
log(f"dof_names: {art.dof_names}")
log(f"reached pan={math.degrees(reached[0]):.2f}, "
    f"tilt={math.degrees(reached[1]):.2f}")

arr_rgbA = np.zeros((0,))
for _ in range(20):
    rep.orchestrator.step()

    arr_rgbA = np.asarray(rgb_annotator.get_data())
    if arr_rgbA.size > 0 and arr_rgbA.ndim == 3 and arr_rgbA.shape[2] >= 3:
        log(f"Frame Shape: {arr_rgbA.shape}")
        rgb_pill = PIL.Image.fromarray(arr_rgbA[:, :, :3])
        rgb_pill.save(os.path.join(out_dir, "phase1_view.png"))
        break
else:
    log(f"RGB CAPTURE FAILED: 20 steps")



log("done; closing app")

sim_app.close()
