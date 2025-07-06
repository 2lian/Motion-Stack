# ruff: noqa: E402 # Allow imports after creating SimulationApp

import logging
import os
import sys
from pathlib import Path

# Add the package root to sys.path
package_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.append(package_root)

is_headless = "--headless" in sys.argv
sim_config_path = next(
    (arg.split("=")[1] for arg in sys.argv if arg.startswith("--sim-config-path=")),
    None,
)

if sim_config_path is None:
    sim_config_path = "default.toml"

from pprint import pprint

from environments.config import SimConfig, load_config

logging.info(f"Loading sim config from {sim_config_path}")
config: SimConfig = load_config(sim_config_path)
pprint(config.model_dump())


os.environ["OMNI_KIT_ACCEPT_EULA"] = "YES"
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": is_headless})

from isaacsim.core.utils.extensions import enable_extension

enable_extension("isaacsim.ros2.bridge")
enable_extension("omni.graph.bundle.action")
if not is_headless:
    enable_extension("omni.graph.window.core")
    enable_extension("omni.graph.window.action")
    enable_extension("omni.kit.window.commands")

import omni.isaac.core.objects
import omni.kit.app
import omni.kit.commands
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.kit.viewport.utility.camera_state import ViewportCameraState
from pxr import Gf, Sdf, UsdLux, UsdGeom

def reference_usd(usd_file: str, prim_path: str):
    path = str(Path(__file__).parent / "usd" / usd_file)
    return add_reference_to_stage(usd_path=path, prim_path=prim_path)


from environments.ground_truth_tf import GroundTruthTF
from environments.isaac_utils import apply_transform_config
from environments.joint_controller import JointController
from environments.load_moonbot import load_moonbot
from environments.observer_camera import ObserverCamera
from environments.realsense_camera import RealsenseCamera
from environments.reference_usd import add_usd_reference_to_stage

world = World(stage_units_in_meters=1.0)
world.play()

# Start publishing the clock first so the ROS2 pocesses can start
reference_usd("clock.usda", "/Graphs")

robot_name_to_prim = {}
for robot in config.robots:
    robot_path = load_moonbot(world, robot)
    robot_name_to_prim[robot.name] = robot_path

    if not robot.visualization_mode:
        if robot.publish_ground_truth_tf:
            GroundTruthTF(robot_prim=robot_path).initialize()

        if not robot.without_controls:
            JointController(robot_prim=robot_path).initialize()

    if robot.realsense_camera:
        RealsenseCamera(robot_path, robot.realsense_camera).initialize()

if config.ground:
    ground = reference_usd("ground.usda", "/Ground")
    if config.ground.transform:
        apply_transform_config(ground, config.ground.transform)
    # We dont want the ground plane default sphere light
    sphere_light = world.stage.GetPrimAtPath("/Ground/defaultGroundPlane/SphereLight")
    if sphere_light.IsValid():
        sphere_light.GetAttribute("visibility").Set("invisible")

for observer_camera_config in config.observer_cameras:
    ObserverCamera(observer_camera_config).initialize()

for usd_reference in config.usd_references:
    add_usd_reference_to_stage(world, usd_reference)

camera_state = ViewportCameraState("/OmniverseKit_Persp")
camera_state.set_position_world(
    Gf.Vec3d(-0.3577958949555765, -1.1875695366976564, 0.632201840815314), True
)
camera_state.set_target_world(Gf.Vec3d(*config.camera.target), True)

omni.kit.commands.execute(
    "CreatePrimWithDefaultXform",
    prim_type="DomeLight",
    prim_path="/World/DomeLight",
    attributes={
        "inputs:colorTemperature": 6150,
        "inputs:intensity": 1,
        "inputs:exposure": 8,
        "inputs:texture:format": "latlong",
    },
    select_new_prim=False,
)
dome_light = world.stage.GetPrimAtPath("/World/DomeLight")
dome_light.CreateAttribute("visibleInPrimaryRay", Sdf.ValueTypeNames.Bool).Set(False)
# Create DistantLight with specified settings
distantLight = UsdLux.DistantLight.Define(world.stage, Sdf.Path("/DistantLight"))
distantLight.CreateColorTemperatureAttr(7250)
distantLight.CreateIntensityAttr(1.5)
distantLight.CreateExposureAttr(9.5)
distantLight.CreateAngleAttr(0.53)

# Set transform for distant light (translate and rotate)
xform = UsdGeom.Xformable(distantLight)
xform.ClearXformOpOrder()
# Translation
xform.AddTranslateOp().Set(Gf.Vec3d(0, 0, 305))
# Rotation (55, 0, 135 degrees)
xform.AddRotateXYZOp().Set(Gf.Vec3f(55, 0, 135))


def set_initial_joint_positions():
    from omni.isaac.dynamic_control import _dynamic_control

    dc = _dynamic_control.acquire_dynamic_control_interface()
    for robot_config in config.robots:
        if robot_config.initial_joint_positions:
            robot_prim_path = robot_name_to_prim.get(robot_config.name)
            art_handle = dc.get_articulation(robot_prim_path)
            if art_handle == 0:
                logging.warning(f"Could not find articulation for {robot_prim_path}")
                continue

            for joint_name, position in robot_config.initial_joint_positions.items():
                dof_ptr = dc.find_articulation_dof(art_handle, joint_name)
                if dof_ptr:
                    logging.info(
                        f"Setting initial position for {joint_name} in {robot_prim_path} to {position}"
                    )
                    dc.set_dof_position(dof_ptr, position)
                    dc.set_dof_position_target(
                        dof_ptr,
                        position,
                    )
                else:
                    logging.error(
                        f"Could not find DOF pointer for {joint_name} in {robot_prim_path}"
                    )


world.reset()
is_first_step = True


while simulation_app.is_running():
    world.step(render=True)

    if world.is_playing():
        if is_first_step:
            # Set initial joint positions only once at the start of the simulation
            # TODO: This should run after world the world reset. Probably possible with a callback.
            set_initial_joint_positions()
            is_first_step = False
    else:
        is_first_step = True
simulation_app.close()
