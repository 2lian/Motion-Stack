import json
import sys
from pathlib import Path

# Add Isaac Sim sdk to vscode extra paths
ISAAC_VERSION="4.2.0" # TODO: Add argument to change version

def version_ge(v1, v2):
    return tuple(map(int, (v1.split(".")))) >= tuple(map(int, (v2.split("."))))

def version_gt(v1, v2):
    return tuple(map(int, (v1.split(".")))) > tuple(map(int, (v2.split("."))))

# Find the sdk path (TODO: Add argument for custom path)
home_path = Path.home()
if version_ge(ISAAC_VERSION, "4.2.0") and not version_gt(ISAAC_VERSION, "2021.2.0"):
    if sys.platform == "win32":
        sdk_path = home_path / "AppData" / "Local" / "ov" / "pkg" / f"isaac-sim-{ISAAC_VERSION}"
    else:
        sdk_path = home_path / ".local" / "share" / "ov" / "pkg" / f"isaac-sim-{ISAAC_VERSION}"
elif version_ge(ISAAC_VERSION, "2021.2.1") and not version_ge(ISAAC_VERSION, "2023.1.2"):
    if sys.platform == "win32":
        sdk_path = home_path / "AppData" / "Local" / "ov" / "pkg" / f"isaac_sim-{ISAAC_VERSION}"
    else:
        sdk_path = home_path / ".local" / "share" / "ov" / "pkg" / f"isaac_sim-{ISAAC_VERSION}"
else:
    print(f"Unsupported Isaac Sim version: {ISAAC_VERSION}")
    sys.exit(0)

extra_paths = []

# for each folder in the sdk path
for folder in sdk_path.iterdir():
    if not folder.is_dir():
        continue

    print(f"Found folder: {folder}")

    # for each folder in the folder
    for subfolder in folder.iterdir():
        if not subfolder.is_dir():
            continue
        print(f"Found package: {subfolder}")
        extra_paths.append(str(subfolder))


vscode_config =  Path(__file__).parent.parent / ".vscode/settings.json"
vscode_config.parent.mkdir(parents=True, exist_ok=True)

# Read config (create if it doesn't exist)
if not vscode_config.exists():
    vscode_config.write_text("{}")

config = json.loads(vscode_config.read_text())

# Merge the extra paths into the config
for settings_key in ["python.analysis.extraPaths", "python.autoComplete.extraPaths"]:
    old_extra_paths = config.get(settings_key, [])
    config[settings_key] = list(set(old_extra_paths + extra_paths))

# Write the config back
vscode_config.write_text(json.dumps(config, indent=4))