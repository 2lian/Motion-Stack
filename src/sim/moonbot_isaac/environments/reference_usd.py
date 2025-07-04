import logging
from pathlib import Path

from environments.config import UsdReferenceConfig
from environments.isaac_utils import apply_transform_config
from environments.ros_utils import replace_package_urls_with_paths


def add_usd_reference_to_stage(world, usd_reference_config: UsdReferenceConfig):
    """
    Adds a USD reference to the stage based on the provided configuration.
    Returns:
        The reference to the added prim, or None if processing failed
    """
    # Import omniverse modules after omniverse is initialized
    import omni.kit.commands
    from omni.isaac.core.utils.stage import add_reference_to_stage, get_next_free_path
    from pxr import Sdf

    # Get the absolute path to the blend file
    usd_path = replace_package_urls_with_paths(usd_reference_config.path)
    logging.info(f"Using USD path: {usd_path}")

    # The usd_reference_config.name or the filename is used as the name of the prim
    prim_name = usd_reference_config.name
    if not prim_name:
        prim_name = Path(usd_path).stem
    path = get_next_free_path(prim_name, "/")
    logging.info(f"Adding prim with path: {path}")

    prim = add_reference_to_stage(usd_path, path)

    if usd_reference_config.rigid_body:
        omni.kit.commands.execute(
            "SetRigidBody",
            path=Sdf.Path(path),
            approximationShape=usd_reference_config.rigid_body.approximation_shape,
            kinematic=usd_reference_config.rigid_body.kinematic,
        )

    # Set prim attributes
    for prop_path, value in usd_reference_config.prim_properties.items():
        omni.kit.commands.execute(
            "ChangeProperty",
            prop_path=Sdf.Path(f"{path}/{prop_path}"),
            value=value,
            prev=value,
            usd_context_name=omni.usd.get_context().get_stage(),
        )

    if usd_reference_config.transform:
        apply_transform_config(path, usd_reference_config.transform)

    return prim
