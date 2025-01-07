import importlib
import pkgutil

import pytest

import easy_robot_control  # Replace with your library's root name

def find_submodules(package):
    """Find all submodules of the given package."""
    return [
        module.name
        for module in pkgutil.walk_packages(package.__path__, package.__name__ + ".")
    ]


@pytest.mark.parametrize("module_name", find_submodules(easy_robot_control))
def test_imports(module_name):
    """Test if a module in the package can be imported without ImportError."""
    try:
        importlib.import_module(module_name)
    except ImportError as e:
        pytest.fail(f"Failed to import {module_name}: {e}")


def test_my_rtb_fix():
    try:
        from roboticstoolbox.tools.urdf.urdf import URDF
    except ImportError as e:
        pytest.fail(f"Failed to import roboticstoolbox.tools.urdf.urdf as URDF: {e}")
    # assert URDF.modified_by_elian == True, "This is True if my version of rtb is loaded"
    additions = [ "_recursive_axis_definition", "finalize_linking"]
    for m in additions:
        method = getattr(URDF, m, None)
        assert method is not None, "RTB not fixed by Elian is being used"
        assert callable(method), "RTB not fixed by Elian is being used"

