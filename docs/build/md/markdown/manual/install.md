# Installation using Pixi

[![image](https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/prefix-dev/pixi/main/assets/badge/v0.json)](https://pixi.sh)

## 0. [Install Pixi](https://pixi.sh/latest/installation/)

```bash
curl -fsSL https://pixi.sh/install.sh | sh
```

## 1. Download the workspace

```bash
git clone https://github.com/2lian/Motion-Stack.git
cd Motion-Stack
```

#### NOTE
This documentation now assumes your workspace and working directory is `~/Motion-Stack`.

## 2. Install dependencies and build

Pixi will install all dependencies **including ROS** into a *venv*. The `run` command then calls `colcon build` using the *venv*.

```bash
pixi install # (optional)
pixi run build
```

#### IMPORTANT
For the TUI, you also need to download `ros2-keyboard` from Christopher
Mower (GPLv2 license).

```default
git clone https://github.com/cmower/ros2-keyboard ./src/ros2-keyboard
```

## You are all set!

You can launch the *Moonbot Zero*:

```bash
# Terminal 1
pixi run ros2 launch motion_stack moonbot_zero.launch.py

# Terminal 2
pixi run bash launch_simu_rviz.bash
```

# Installation from source

## ROS2

Install ROS2:

- [Jazzy (Ubuntu 24.04) installation guide.](https://docs.ros.org/en/jazzy/Installation.html)
- [Humble (Ubuntu 22.04) installation guide.](https://docs.ros.org/en/humble/Installation.html)
- [Foxy (Ubuntu 20.04) installation guide.](https://docs.ros.org/en/foxy/Installation.html)

#### NOTE
The core of the Motion-Stack is pure python, ROS2 is only the communication interface, hence necessary.
If you are a developper wanting to use something else instead of ROS2 (pure async python for minimal overhead, Zenoh …), you could develop your own interface.

## Build tools

For installation, building, and docs, [doit](https://pydoit.org) is used. It is a build tool in the vein of *Make* but arguably easier to use. *doit* is **NOT** necessary for this repo, but dealing with multiple robots, multiple ROS2 distros, such tools can help.

```bash
sudo apt install python3-pip python3-doit
```

## Download the workspace

```bash
git clone https://github.com/2lian/Motion-Stack.git
cd Motion-Stack
```

#### NOTE
This documentation assumes your workspace is  *~/Motion-Stack*

## Automated installation using *doit*

Install ROS2 and Python dependencies:

```bash
doit pydep rosdep
```

Build the workspace and Test python dependencies:

```bash
doit -n 8 build test_import
```

### Available doit CLI arguments and settings

You can pass variables to *doit*, such as `doit build syml=y`, thus changing some installation and build settings. Available config is given and can be changed in `doit_config.py`:

```python
OVERIDE_CONFIG = {
    #: if 'venv=y', a python virtual environment is used. This is an early feature
    #: necessary for `ros2 jazzy`, it has not been tested for foxy/humble.
    "venv": None,
    #: if 'syml=y', colcon `--use-symlink` is used, making re-building mostly unnecessary.
    "syml": None,
    #: if 'pipforce=y', often fixes python dependencies
    #: --force-reinstall --update all of the python packages to a compatible version,
    #: regardless of other installed pip dependencies.
    "pipforce": None,
    #: if 'low_mem=y', fixes pip issue on low memory systems (<1GB)
    "low_mem": None,
    #: 'pip_args=<your args>', inserts arguments to pip install.
    #: This only affects pip install of the motion stack dependencies. So, not the pip install of other utilities such as pip-tools, wheels, venv ... You can for example use pip_args="--ignore-installed" to skip already installed packages, useful if some are pinned by ros2 and causes install issues.
    "pip_args": None,
    #: 'colcon_args=<your args>', inserts arguments to colcon build commands.
    "colcon_args": None,
    #: if 'dev=y', install dependencies for devloppers (docs, tests ...).
    #: This is enabled by default
    "dev": None,
}
```

### List all available doit commands with: `doit list`

```console
build         Colcon builds packages. Uses symlink if cli_arg has 'syml=y'
ci_badge      Copies fail/success.rst badge depending on last test result
gitdep        Install/updates github dependencies
html          Builds the documentation as html in docs/build/html
md            Post processes the .md docs for github integration
md_doc        Builds the documentation as markdown in /home/elian/Motion-Stack/docs/build/md
pipcompile    Compiles python requirements
pydep         Install python dependencies. If cli_arg has 'pipforce=y' pydep command will –force-reinstall –update.
python_venv   Creates python venv if cli_arg has 'venv=y'
rosdep        Install ROS dependencies
test          Runs all test, using colcon test
test_import   Fast sanity check -- Tests all python file executability
```

<a id="install-venv"></a>

## Regarding Python dependencies and virtual environments

#### IMPORTANT
If facing pip dependencies issues, try `doit pydep pipforce=y`. This command will **pip –force-reinstall –update** all of your python package to a compatible version, regardless of other installed pip dependencies.

#### NOTE
You can find the venv inside ~/Motion-Stack/venv/ after executing `doit pydep`. To install additional python dependencies in this venv, activate it with `source ~/Motion-Stack/venv/bin/activate` before using `pip install ...`

`doit clean` will delete this venv.

ROS2 [Jazzy with Ubuntu 24.04 requires a python virtual environment](https://docs.ros.org/en/jazzy/How-To-Guides/Using-Python-Packages.html#installing-via-a-virtual-environment), this is quite tricky to use.

> - The venv is only necessary when running Motion-Stack code. If you are using the motion stack through ROS2 messages (as opposed to the python API) and not building it (by working in you own workspace) you do not need to worry about it.
> - When developping with the Motion-Stack you must not only source the workspace, but first also source the venv using `. venv/bin/activate`.
> - To build: source the venv, then use colcon through `python3 -m colcon` and not the system-wide `colcon`.
> - Launching/running does not require the venv as the venv is part of the build and thus automatically used by the node. However, the python venv is unavailable when interpreting a launch.py file, so you cannot use venv libraries in a launcher.

My Jazzy and venv support is still in its early phase, if you want to override the global python packages (like what is done under foxy/humble) please do it manually by referring to the manual installation.

I do not plan to add similar venv support on foxy/humble in the installer, unless the need arises.

## Manual installation (advanced)

### Use rosdep to install ROS2 dependencies

Download `ros2-keyboard` in `src` manually because it is not part of rosdep.

```bash
cd ~/Motion-Stack/src
git clone https://github.com/cmower/ros2-keyboard
```

Run rosdep to install all other ros2 packages.

```bash
# source ros here
cd ~/Motion-Stack
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r
```

### Make a venv (for Jazzy)

```bash
# source ros here
cd ~/Motion-Stack
python3 -m venv --system-site-packages ./venv
. ./venv/bin/activate
python3 -m pip install --upgrade pip wheel
```

### Use pip to install Python dependencies

```bash
cd ~/Motion-Stack/src/motion_stack/
# source venv here if used
sudo apt install python3-pip
pip install pip-tools
pip-compile -o requirements.txt setup.py
pip install -r requirements.txt --force-reinstall --upgrade
rm -rf *.egg-info/ requirements.txt
```

#### NOTE
To install the dev requirements use `python3 -m piptools compile --extra dev -o requirements.txt setup.py`.

#### NOTE
If you have limited ram, try using `CXXFLAGS="-fno-fat-lto-objects --param ggc-min-expand=10 --param ggc-min-heapsize=2048"  MAKEFLAGS="-j1" pip install --no-cache-dir -r requirements.txt --force-reinstall --upgrade`
