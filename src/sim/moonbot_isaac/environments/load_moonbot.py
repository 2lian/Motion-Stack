import logging
import time

import omni
import omni.kit.commands
import omni.usd
from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.dynamic_control import _dynamic_control
from pxr import Sdf, Usd, UsdPhysics

from environments.config import RobotConfig
from environments.isaac_utils import (
    apply_transform_config,
    set_attr,
    set_attr_cmd,
    toggle_active_prims,
)
from environments.prim_to_tf_linker import PrimToTfLinker
from environments.robot_definition_reader import RobotDefinitionReader, XacroReader


def _set_initial_joint_positions(robot_path: str, joint_positions: dict[str, float]):
    """Set the initial joint positions for the robot."""
    dc = _dynamic_control.acquire_dynamic_control_interface()
    art = dc.get_articulation(robot_path)
    if art == 0:
        # This can happen if the articulation is not yet ready.
        logging.warning(f"Could not get articulation for {robot_path}, will retry.")
        return False

    for joint_name, position in joint_positions.items():
        dof_ptr = dc.find_articulation_dof(art, joint_name)
        if dof_ptr != 0:
            dc.set_dof_position(dof_ptr, position)
            dc.set_dof_position_target(dof_ptr, position)
        else:
            logging.warning(f"Could not find DOF for joint {joint_name} in {robot_path}")

    logging.info(f"Successfully set initial joint positions for {robot_path}.")
    return True


def add_urdf_to_stage(urdf_description, robot_config: RobotConfig):
    _status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
    import_config.merge_fixed_joints = False
    import_config.fix_base = False
    import_config.self_collision = False#not robot_config.visualization_mode
    import_config.make_default_prim = True
    import_config.create_physics_scene = True
    import_config.parse_mimic = (
        not robot_config.visualization_mode and robot_config.parse_mimic_joints
    )
    _result, robot_model = omni.kit.commands.execute(
        "URDFParseText", urdf_string=urdf_description, import_config=import_config
    )
    _status, path = omni.kit.commands.execute(
        "URDFImportRobot", urdf_robot=robot_model, import_config=import_config
    )

    return path


def load_moonbot(world: World, robot_config: RobotConfig):
    if robot_config.xacro_path:
        urdf = XacroReader(robot_config)
    elif robot_config.robot_description_topic:
        robot_definition_reader = RobotDefinitionReader(robot_config)
        robot_definition_reader.start_get_robot_description()

        while not robot_definition_reader.urdf_description:
            logging.warning(
                f"Waiting for robot description on topic {robot_config.robot_description_topic}"
            )
            time.sleep(0.1)
            world.step(render=True)

        urdf = robot_definition_reader

    moonbot_path = add_urdf_to_stage(urdf.urdf_description, robot_config=robot_config)

    # Isaac puts the robot assets in the root level.
    # Here move them inside the robot prim to avoid conflict with other robots.
    for group in ["meshes", "visuals", "colliders"]:
        omni.kit.commands.execute(
            "MovePrim",
            path_from=Sdf.Path(f"/{group}"),
            path_to=Sdf.Path(f"{moonbot_path}/_{group}"),
            destructive=False,
            stage_or_context=omni.usd.get_context().get_stage(),
        )

    if robot_config.transform:
        apply_transform_config(moonbot_path, robot_config.transform)

    for child_prim in world.stage.GetPrimAtPath(moonbot_path).GetChildren():
        # Remove the UsdPhysics.ArticulationRootAPI if it exists
        child_prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)

    moonbon = world.stage.GetPrimAtPath(moonbot_path)

    if robot_config.visualization_mode:
        # Turn off gravity
        set_attr_cmd("/physicsScene", "physics:gravityMagnitude", 0.0)

        for child_prim in moonbon.GetChildren():
            child_prim: Usd.Prim = child_prim

            for child_child_prim in child_prim.GetChildren():
                child_child_prim: Usd.Prim = child_child_prim

                # Disable all collisions
                if child_child_prim.HasAttribute("physics:collisionEnabled"):
                    set_attr(child_child_prim, "physics:collisionEnabled", False)

                joint_info = urdf.urdf_extras.get_joint_info(child_child_prim.GetName())

                # Disable all joints so prims (links) can be freely moved
                if joint_info:
                    toggle_active_prims(child_child_prim.GetPath(), False)

        # Start applying the transforms from ROS2
        PrimToTfLinker(
            fixed_frame=robot_config.visualization_fixed_frame,
            robot_prim=moonbon,
        )

    else:
        UsdPhysics.ArticulationRootAPI.Apply(moonbon)
        world.reset()

        articulation = Articulation(moonbot_path)
        articulation.initialize()

        if robot_config.initial_joint_positions:
            callback_name = f"set_initial_positions_{robot_config.name}"
            has_been_set = False

            def set_positions(step_size):
                nonlocal has_been_set
                if not world.is_playing():
                    has_been_set = False
                    return

                if has_been_set:
                    return

                success = _set_initial_joint_positions(
                    moonbot_path, robot_config.initial_joint_positions
                )
                if success:
                    has_been_set = True

            world.add_physics_callback(callback_name, set_positions)

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
                logging.warning(
                    f"Joint {joint_path} does not have a recognized drive type"
                )

    urdf.urdf_extras.apply_to_robot_prim(moonbot_path)

    return moonbot_path
