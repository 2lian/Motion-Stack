import logging
import time

import omni.kit.commands
import omni.usd
from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from pxr import UsdPhysics

from environments.robot_definition_reader import RobotDefinitionReader, XacroReader
from environments.utils import set_attr
from environments.config import RobotConfig


def add_urdf_to_stage(urdf_description):
    _status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
    import_config.merge_fixed_joints = False
    import_config.fix_base = False
    import_config.make_default_prim = True
    import_config.create_physics_scene = True
    import_config.parse_mimic = True
    _result, robot_model = omni.kit.commands.execute(
        "URDFParseText", urdf_string=urdf_description, import_config=import_config
    )
    _status, path = omni.kit.commands.execute(
        "URDFImportRobot", urdf_robot=robot_model, import_config=import_config
    )

    return path


def load_moonbot(world: World, robot_config: RobotConfig):
    if robot_config.xacro_path:
        urdf = XacroReader(robot_config.xacro_path)
    elif robot_config.robot_description_topic:
        robot_definition_reader = RobotDefinitionReader()
        robot_definition_reader.start_get_robot_description(robot_config.robot_description_topic)

        while not robot_definition_reader.urdf_description:
            logging.warning("Waiting for robot description")
            time.sleep(0.1)
            world.step(render=True)

        urdf = robot_definition_reader

    moonbot_path = add_urdf_to_stage(urdf.urdf_description)
    urdf.urdf_extras.apply_to_robot_prim(moonbot_path)

    for child_prim in world.stage.GetPrimAtPath(moonbot_path).GetChildren():
        # Remove the UsdPhysics.ArticulationRootAPI if it exists
        child_prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)

    moonbon = world.stage.GetPrimAtPath(moonbot_path)
    UsdPhysics.ArticulationRootAPI.Apply(moonbon)


    world.reset()

    articulation = Articulation(moonbot_path)
    articulation.initialize()
    
    for joint_path in articulation._articulation_view._dof_paths[0]:
        # Get the joint prim
        joint_prim = world.stage.GetPrimAtPath(joint_path)

        # For now use the same values all angular drives (joints) and all linear drives (grippers)
        if joint_prim.HasAttribute("drive:linear:physics:type"):
            logging.info(f"Joint {joint_path} has a linear drive")
            set_attr(joint_path, "physxJoint:maxJointVelocity", 0.003)
            set_attr(joint_path, "physxJoint:armature", 200.0)
            set_attr(joint_path, "physxJoint:jointFriction", 0.2)
            set_attr(joint_path, "drive:linear:physics:damping", 1e6)
            set_attr(joint_path, "drive:linear:physics:stiffness", 1e10)
            set_attr(joint_path, "drive:linear:physics:maxForce", 150000.0)
            set_attr(joint_path, "drive:linear:physics:type", "acceleration")
        elif joint_prim.HasAttribute("drive:angular:physics:type"):
            logging.info(f"Joint {joint_path} has an angular drive")
            set_attr(joint_path, "physxJoint:maxJointVelocity", 5.0)
            set_attr(joint_path, "physxJoint:armature", 200.0)
            set_attr(joint_path, "physxJoint:jointFriction", 0.2)
            set_attr(joint_path, "drive:angular:physics:damping", 1e6)
            set_attr(joint_path, "drive:angular:physics:stiffness", 1e10)
            set_attr(joint_path, "drive:angular:physics:maxForce", 150000.0)
            set_attr(joint_path, "drive:angular:physics:type", "acceleration")
        else:
            logging.warning(f"Joint {joint_path} does not have a recognized drive type")

    return moonbot_path
