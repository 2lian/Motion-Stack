# Installation

## ROS2

- [Humble (Ubuntu 22.04) installation guide.](https://docs.ros.org/en/humble/Installation.html)
- [Foxy (Ubuntu 20.04) installation guide.](https://docs.ros.org/en/foxy/Installation.html)

## Build tools

For installation, building, and docs, [doit](https://pydoit.org) is used. It is a build tool in the vein of *Make* but arguably easier to use. *doit* is **NOT** necessary for this repo, but dealing with robots, and multiple of them, such tools can help.

```bash
sudo apt install python3-pip python3-doit
```

## Download the workspace

```bash
git clone https://github.com/2lian/Motion-Stack.git
```

#### NOTE
This documentation assumes your workspace is  *~/Motion-Stack*

## Automated installation using *doit*

Install ROS2 and Python dependencies:

```bash
doit pydep-hard rosdep
```

#### WARNING
This pydep command will **–force-reinstall –update** all of your python package to a compatible version, regardless of other installed pip dependencies. Use `doit pydep-soft` or install manually to handle this yourself.

Build the workspace and Test python dependencies:

```bash
doit -n 8 build test_import
```

### List all available doit commands with: `doit list`

```console
build         Colcon builds packages
ci_badge      Copies fail/success.rst badge depending on last test result
gitdep        Install/updates github dependencies
html          Builds the documentation as html in docs/build/html
md            Post processes the .md docs for github integration
md_doc        Builds the documentation as markdown in ./docs/build/md
pipcompile    Compiles pyhton requirements
pydep-hard    Install python dependencies using --force-reinstall --upgrade
pydep-soft    Install python dependencies (not garanteed to work)
rosdep        Install ROS dependencies
test          Runs all test, using colcon test
test_import   Fast sanity check -- Tests all python file executability
```

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

### Use pip to install Python dependencies

```bash
cd ~/Motion-Stack/src/motion_stack/
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
