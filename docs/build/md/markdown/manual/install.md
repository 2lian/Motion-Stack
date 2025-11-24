<a id="install-pixi-label"></a>

# Installation using Pixi

[![image](https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/prefix-dev/pixi/main/assets/badge/v0.json)](https://pixi.sh)

Pixi can install different versions of ROS2! Quickly! In a virtual environment! Please check out their fantastic work and documentation.

## 0. [Install Pixi](https://pixi.sh/latest/installation/)

```bash
curl -fsSL https://pixi.sh/install.sh | sh
```

## 1. Download the workspace

```bash
git clone https://github.com/2lian/Motion-Stack.git
cd Motion-Stack
```

## 2. Install dependencies and build

Pixi will install all dependencies **including ROS** into a *virtual environment*, this evironment is defined by our pixi.toml. The `run build` command then calls `colcon build` using the *venv*.

```bash
pixi install # (optional)
pixi run -e default build
```

## You are all set!

Using Pixi, you can follow this documentation as normal. Simply **type \`\`pixi run …\`\` before the commands you want to run**. Or easier, activate the pixi environment by running `pixi shell`.

You can launch the *Moonbot Zero*:

```bash
# Terminal 1
pixi run ros2 launch motion_stack moonbot_zero.launch.py
```

```bash
# Terminal 2
pixi run ros2 launch motion_stack rviz_simu.launch.py
```

And have fun with the [Operator TUI](operator_tui.md#tui).

```bash
# Terminal 3
pixi run bash operator.bash
```

## (Preview) Conda package

We are working on distributing binaries (`x86`, `aarch64`) through Conda packages thanks to Pixi and RoboStack. [The motion stack binaries are hosted here](https://prefix.dev/channels/motion-stack) and installation can be as simple as:

```bash
pixi init my_workspace -c https://prefix.dev/motion-stack -c https://prefix.dev/conda-forge -c https://prefix.dev/robostack-jazzy
pixi add ros-jazzy-motion-stack ros-jazzy-ms-operator ros-jazzy-motion-stack-tuto
```

<a id="install-source-label"></a>

# Installation from source

## 0. Install ROS2

Install ROS2:

- [Kilted (Ubuntu 24.04) installation guide.](https://docs.ros.org/en/kilted/Installation.html)
- [Jazzy (Ubuntu 24.04) installation guide.](https://docs.ros.org/en/jazzy/Installation.html)
- [Humble (Ubuntu 22.04) installation guide.](https://docs.ros.org/en/humble/Installation.html)
- [Foxy (Ubuntu 20.04)](https://docs.ros.org/en/foxy/Installation.html) : DEPRECATED [after this commit](https://github.com/2lian/Motion-Stack/commit/42516a5ea84041a191c9e25279a1f5a7e7e91580) in favor of Pixi and RoboStack.

#### NOTE
The core of the Motion-Stack is pure python, ROS2 is the (only) communication interface (for now).
If you are a developer wanting to use something else instead of ROS2 (pure async python for minimal overhead, Zenoh …), you could develop your own interface.

## 1. Download the workspace

This documentation now assumes your workspace and working directory is
 *~/Motion-Stack*. Alternatively you can copy the contents of ./src into
another workspace.

```bash
git clone https://github.com/2lian/Motion-Stack.git
cd Motion-Stack
```

## 2. Install ROS dependencies with rosdep

```bash
# source ros here
cd ~/Motion-Stack
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r
```

## 3. Install Python dependencies

### (Optional) Create and activate a venv

```bash
# source ros here
cd ~/Motion-Stack
python3 -m venv --system-site-packages ./venv
. ./venv/bin/activate
python3 -m pip install --upgrade pip wheel
```

### Install using pip

```bash
# source ros here
# source venv here if used
cd ~/Motion-Stack/src/motion_stack/
python3 -m pip install pip-tools
python3 -m pip-compile -o requirements.txt setup.py
python3 -m pip install -r requirements.txt --force-reinstall --upgrade
rm -rf *.egg-info/ requirements.txt
```

#### IMPORTANT
You might need to use: `python3 -m pip install -r requirements.txt --force-reinstall --upgrade`. It often works better.

## 4. Build the workspace

```bash
# source ros here
# source venv here if used
cd ~/Motion-Stack
python3 -m colcon build --symlink-install
# You can now source the workspace
source ./install/setup.bash
```

## You are all set!

You can launch the *Moonbot Zero*:

```bash
# Terminal 1
# source your workspace and venv here
ros2 launch motion_stack moonbot_zero.launch.py
```

```bash
# Terminal 2
# source your workspace and venv here
ros2 launch motion_stack rviz_simu.launch.py
```

And have fun with the [Operator TUI](operator_tui.md#tui).

```bash
# Terminal 3
# source your workspace and venv here
bash operator.bash
```
