import logging
import time

import omni.kit.commands
import omni.usd
from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from pxr import UsdPhysics

from environments.robot_definition_reader import RobotDefinitionReader
from environments.utils import set_attr


def add_urdf_to_stage(urdf_description):
    _status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
    import_config.merge_fixed_joints = False
    import_config.fix_base = False
    import_config.make_default_prim = True
    import_config.create_physics_scene = True
    import_config.parse_mimic = False
    _result, robot_model = omni.kit.commands.execute(
        "URDFParseText", urdf_string=urdf_description, import_config=import_config
    )
    _status, path = omni.kit.commands.execute(
        "URDFImportRobot", urdf_robot=robot_model, import_config=import_config
    )

    return path


def load_moonbot(world: World):
    robot_definition_reader = RobotDefinitionReader()
    robot_definition_reader.start_get_robot_description("/robot_description_isaac")

    while not robot_definition_reader.urdf_abs:
        logging.warning("Waiting for robot description")
        time.sleep(0.1)
        world.step(render=True)

    moonbot_path = add_urdf_to_stage(robot_definition_reader.urdf_abs)

    base_link = world.stage.GetPrimAtPath(f"{moonbot_path}/base_link")
    base_link.RemoveAPI(UsdPhysics.ArticulationRootAPI)
    moonbon = world.stage.GetPrimAtPath(moonbot_path)
    UsdPhysics.ArticulationRootAPI.Apply(moonbon)

    world.reset()

    articulation = Articulation(moonbot_path)
    articulation.initialize()
    for joint_path in articulation._articulation_view._dof_paths[0]:
        set_attr(joint_path, "physxJoint:maxJointVelocity", 5.0)
        set_attr(joint_path, "physxJoint:armature", 200.0)
        set_attr(joint_path, "physxJoint:jointFriction", 0.2)
        set_attr(joint_path, "drive:angular:physics:damping", 1e6)
        set_attr(joint_path, "drive:angular:physics:stiffness", 1e10)
        set_attr(joint_path, "drive:angular:physics:maxForce", 1500.0)
        set_attr(joint_path, "drive:angular:physics:type", "acceleration")

    return moonbot_path
