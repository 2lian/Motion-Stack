import logging

import pxr
from omni.isaac.core.utils.prims import get_prim_at_path
from pxr import Gf

from environments.config import TransformConfig

# import omni.kit.property.usd.prim_selection_payload.PrimSelectionPayload



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
        logging.warn(f"Attribute {attr_name} not found in {prim.GetPath()}")

def apply_transform_config(prim, transform: TransformConfig):
    if type(prim) is str:
        prim = get_prim_at_path(prim)

    prop_names = prim.GetPropertyNames()
    xformable = pxr.UsdGeom.Xformable(prim)
    xformable.ClearXformOpOrder()
    if "xformOp:translate" not in prop_names:
        xformable.AddXformOp(
            pxr.UsdGeom.XformOp.TypeTranslate, pxr.UsdGeom.XformOp.PrecisionDouble, ""
        )

    if "xformOp:orient" not in prop_names:
        xformable.AddXformOp(
            pxr.UsdGeom.XformOp.TypeOrient, pxr.UsdGeom.XformOp.PrecisionDouble, ""
        )
    
    set_attr(
        prim,
        "xformOp:translate",
        Gf.Vec3f(
            transform.translation[0],
            transform.translation[1],
            transform.translation[2],
        ),
    )
    set_attr(
        prim,
        "xformOp:orient",
        Gf.Quatd(
            transform.rotation[0],
            transform.rotation[1],
            transform.rotation[2],
            transform.rotation[3],
        ),
    )
