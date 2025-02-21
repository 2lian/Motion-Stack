# ruff: noqa: E402 # Allow imports after creating SimulationApp

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

print(f"Loading sim config from {sim_config_path}")
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
from pxr import Gf, Sdf, UsdLux


def reference_usd(usd_file: str, prim_path: str):
    path = str(Path(__file__).parent / "usd" / usd_file)
    return add_reference_to_stage(usd_path=path, prim_path=prim_path)

from environments.load_moonbot import load_moonbot
from environments.realsense_camera import RealsenseCamera
from environments.utils import apply_transform_config
from environments.ground_truth_tf import GroundTruthTF
from environments.joint_controller import JointController

world = World(stage_units_in_meters=1.0)
world.play()

# Start publishing the clock first so the ROS2 pocesses can start
reference_usd("clock.usda", "/Graphs")

for robot in config.robots:
    robot_path = load_moonbot(world, robot)

    if not robot.visualization_mode and not robot.without_controls:
        GroundTruthTF(robot_prim=robot_path).initialize()
        JointController(robot_prim=robot_path).initialize()

if config.ground:
    ground = reference_usd("ground.usda", "/Ground")
    if config.ground.transform:
        apply_transform_config(ground, config.ground.transform)
        

reference_usd("observer_camera.usda", "/ObserverCamera")

# rs_camera = RealsenseCamera()
# rs_camera.initialize()


camera_state = ViewportCameraState("/OmniverseKit_Persp")
camera_state.set_position_world(
    Gf.Vec3d(-0.3577958949555765, -1.1875695366976564, 0.632201840815314), True
)
camera_state.set_target_world(Gf.Vec3d(*config.camera.target), True)

distantLight = UsdLux.DistantLight.Define(world.stage, Sdf.Path("/DistantLight"))
distantLight.CreateIntensityAttr(500)

omni.kit.commands.execute(
    "CreatePrimWithDefaultXform",
    prim_type="DomeLight",
    prim_path="/World/DomeLight",
    attributes={"inputs:intensity": 1000, "inputs:texture:format": "latlong"},
    select_new_prim=True,
)

world.reset()

while simulation_app.is_running():
    world.step(render=True)

    if world.is_playing():
        pass
simulation_app.close()
