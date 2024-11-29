import pytest
import pkgutil
import importlib
import rviz_basic  # Replace with your library's root name

def find_submodules(package):
    """Find all submodules of the given package."""
    return [
        module.name
        for module in pkgutil.walk_packages(package.__path__, package.__name__ + ".")
    ]

@pytest.mark.parametrize("module_name", find_submodules(rviz_basic))
def test_imports(module_name):
    """Test if a module in the package can be imported without ImportError."""
    try:
        importlib.import_module(module_name)
    except ImportError as e:
        pytest.fail(f"Failed to import {module_name}: {e}")
