# ruff: noqa: E402 # Allow imports after creating SimulationApp

import os
from pathlib import Path
import sys


os.environ["OMNI_KIT_ACCEPT_EULA"] = "YES"
from isaacsim import SimulationApp

is_headless = "--headless" in sys.argv

simulation_app = SimulationApp({"headless": is_headless})

from omni.isaac.core.utils.extensions import enable_extension

enable_extension("omni.isaac.ros2_bridge")
enable_extension("omni.isaac.gain_tuner")
enable_extension("omni.isaac.ros2_bridge.robot_description")

import omni.isaac.core.objects
import omni.kit.app
import omni.kit.commands
from omni.isaac.core import World
from omni.kit.viewport.utility.camera_state import ViewportCameraState
from pxr import Gf, Sdf, UsdLux
from omni.isaac.core.utils.stage import add_reference_to_stage


def reference_usd(usd_file: str, prim_path: str):
    path = str(Path(__file__).parent / "usd" / usd_file)
    return add_reference_to_stage(usd_path=path, prim_path=prim_path)


# Add the package root to sys.path
package_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.append(package_root)

from environments.load_moonbot import load_moonbot
from environments.realsense_camera import RealsenseCamera

world = World(stage_units_in_meters=1.0)
world.play()

# Start publishing the clock first so the ROS2 pocesses can start
reference_usd("clock.usda", "/Graphs")
load_moonbot(world)
reference_usd("ground.usda", "/Ground")
reference_usd("joint_controller.usda", "/Graphs")
reference_usd("ground_truth_tf.usda", "/Graphs")
reference_usd("observer_camera.usda", "/ObserverCamera")

rs_camera = RealsenseCamera()
rs_camera.initialize()


camera_state = ViewportCameraState("/OmniverseKit_Persp")
camera_state.set_position_world(
    Gf.Vec3d(-0.3577958949555765, -1.1875695366976564, 0.632201840815314), True
)
camera_state.set_target_world(Gf.Vec3d(0.4, 0.4, 0.0), True)

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
