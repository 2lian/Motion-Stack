import logging

import omni.kit.commands
import omni.usd
import pxr
from omni.isaac.core.utils.prims import get_prim_at_path
from pxr import Gf, Sdf, Usd, PhysxSchema

from environments.config import TransformConfig


def set_attr(prim, attr_name, value):
    if type(prim) is str:
        prim = get_prim_at_path(prim)

    phys_attr = prim.GetAttribute(attr_name)
    if phys_attr:
        success = phys_attr.Set(value)
        logging.info(
            f"Set {attr_name} of {prim.GetPath()} to {value}. Success: {success}"
        )
    else:
        logging.warning(f"Attribute {attr_name} not found in {prim.GetPath()}")


def set_attr_cmd(prim: str | Usd.Prim, attr_name: str, value):
    if type(prim) is Usd.Prim:
        prim = prim.GetPath()

    if attr_name == "physics:approximation" and value == "sdf":
        # NOTE: In theory, this shouldn't be necessary. In the editor, it only show the `ChangeProperty` commands and that works properly.
        # Without this, it physix won't recognize the SDF mesh collision properly.
        # TODO: Try removing this in Isaac Sim 4.5.0
        meshCollision = PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(
            get_prim_at_path(prim)
        )
        meshCollision.CreateSdfResolutionAttr().Set(256)

    omni.kit.commands.execute(
        "ChangeProperty",
        prop_path=Sdf.Path(f"{prim}.{attr_name}"),
        value=value,
        prev=None,
        usd_context_name=omni.usd.get_context().get_stage(),
    )


def apply_transform_config(prim, transform: TransformConfig):
    if type(prim) is str:
        prim = get_prim_at_path(prim)

    prop_names = prim.GetPropertyNames()
    xformable = pxr.UsdGeom.Xformable(prim)
    if "xformOp:translate" not in prop_names:
        xformable.AddXformOp(
            pxr.UsdGeom.XformOp.TypeTranslate, pxr.UsdGeom.XformOp.PrecisionDouble, ""
        )

    if "xformOp:orient" not in prop_names:
        xformable.AddXformOp(
            pxr.UsdGeom.XformOp.TypeOrient, pxr.UsdGeom.XformOp.PrecisionDouble, ""
        )

    set_attr_cmd(
        prim,
        "xformOp:translate",
        Gf.Vec3f(
            transform.translation[0],
            transform.translation[1],
            transform.translation[2],
        ),
    )
    set_attr_cmd(
        prim,
        "xformOp:orient",
        Gf.Quatd(
            transform.rotation[0],
            transform.rotation[1],
            transform.rotation[2],
            transform.rotation[3],
        ),
    )


def toggle_active_prims(prim_path, active: bool):
    omni.kit.commands.execute(
        "ToggleActivePrims",
        stage_or_context=omni.usd.get_context().get_stage(),
        prim_paths=[prim_path],
        active=False,
    )
