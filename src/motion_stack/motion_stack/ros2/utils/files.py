import os
from launch_ros.substitutions.find_package import get_package_share_directory

def get_src_folder(package_name: str) -> str:
    """Absolute path to workspace/src/package_name.

    Note:
        Meant for debugging. Avoid using this, you should build properly.

    Args:
        package_name: workspace/src/package_name

    Returns: Absolute path as str

    """
    package_share_directory = get_package_share_directory(package_name)
    workspace_root = os.path.abspath(os.path.join(package_share_directory, "../../../.."))
    package_src_directory = os.path.join(workspace_root, "src", package_name)
    return package_src_directory



