import logging

from omni.isaac.core.utils.prims import get_prim_at_path



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

