# Installation

## ROS2

### Humble (Ubuntu 22.04)

Installation guide of humble: [https://docs.ros.org/en/humble/Installation.html](https://docs.ros.org/en/humble/Installation.html)

### Foxy (Ubuntu 20.04)

Installation guide of foxy: [https://docs.ros.org/en/foxy/Installation.html](https://docs.ros.org/en/foxy/Installation.html)

## Download the workspace

Clone this repo in the Moonbot-Motion-Stack workspace (or anywhere else you like):

```bash
cd
git clone https://github.com/2lian/Moonbot-Motion-Stack.git
cd Moonbot-Motion-Stack
```

#### NOTE
This documentation assumes your workspace is  *~/Moonbot-Motion-Stack*

## Use rosdep to install ROS2 dependencies automatically

```bash
# source ros here
cd ~/Moonbot-Motion-Stack
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r
```

## Use pip to install Python dependencies automatically

```bash
cd ~/Moonbot-Motion-Stack/src/easy_robot_control
sudo apt install python3-pip
pip install pip-tools
pip-compile -o requirements.txt setup.py
pip install -r requirements.txt --force-reinstall --upgrade
rm -rf *.egg-info/ requirements.txt
```

#### NOTE
To install the dev requirements use `python3 -m piptools compile --extra dev -o requirements.txt setup.py`.

#### NOTE
If you have limited ram, try using `CXXFLAGS="-fno-fat-lto-objects -O2 --param ggc-min-expand=10 --param ggc-min-heapsize=2048"  MAKEFLAGS="-j1" pip install --no-cache-dir -r requirements.txt --force-reinstall --upgrade`

## (Testing)

Those installation steps are tested regularly, from a fresh Ubuntu install, using GitHub workflow. [See the installation test routine, for more details](https://github.com/2lian/Moonbot-Motion-Stack/blob/main/.github/workflows/stepbystep.yaml).

[![image](https://github.com/2lian/Moonbot-Motion-Stack/actions/workflows/stepbystep.yaml/badge.svg)](https://github.com/2lian/Moonbot-Motion-Stack/actions/workflows/stepbystep.yaml)
```yaml
name: Install foxy | humble
on:
  schedule:
    - cron: '0 15 * * *'
  workflow_dispatch:
jobs:
  test:
    name: Test ${{ matrix.os }}
    runs-on: ${{ matrix.os }}
    strategy:
      matrix:
        os: [ubuntu-20.04, ubuntu-22.04]
      
    steps:
      - uses: actions/checkout@v4
      - name: Set ROS_DISTRO
        run: echo "ROS_DISTRO=${{ matrix.os == 'ubuntu-20.04' && 'foxy' || 'humble' }}" >> $GITHUB_ENV
      - name: Cloning repo
        run: |
          cp -r ${{ github.workspace }} ~/Moonbot-Motion-Stack
      - name: Set up environment
        run: |
          locale  # check for UTF-8
          
          sudo apt update && sudo apt install locales
          sudo locale-gen en_US en_US.UTF-8
          sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
          export LANG=en_US.UTF-8
          
          locale  # verify settings
          
          sudo apt install software-properties-common
          sudo add-apt-repository universe
          sudo apt update && sudo apt install curl -y
          sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
          echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
      - name: Updating environement
        run: |
          sudo apt update
          sudo apt upgrade
      - name: Installing Ros2
        run: sudo apt install ros-$ROS_DISTRO-ros-base python3-argcomplete ros-dev-tools python3-colcon-common-extensions
      - name: Rosdep installing dependencies
        run: |
          source /opt/ros/$ROS_DISTRO/setup.bash
          cd ~/Moonbot-Motion-Stack
          sudo rosdep init || true
          rosdep update
          rosdep install --from-paths src --ignore-src -r
          if [[ "${{ matrix.os }}" == "ubuntu-20.04" ]]; then
            sudo apt install ros-foxy-xacro ros-foxy-joint-state-publisher
          fi
      - name: Pip installing dependencies
        run: |
          cd ~/Moonbot-Motion-Stack/src/easy_robot_control
          sudo apt install python3-pip
          pip install pip-tools # for dependencies
          python3 -m piptools compile -o requirements.txt setup.py # takes a while (2-3 min on embedded pc)
          pip install -r requirements.txt --force-reinstall --upgrade # for dependencies
          rm -rf *.egg-info/
      - name: Colcon Build
        run: |
          source /opt/ros/$ROS_DISTRO/setup.bash
          cd ~/Moonbot-Motion-Stack
          colcon build --cmake-args -Wno-dev
      - name: Colcon Test
        run: |
          cd ~/Moonbot-Motion-Stack
          source /opt/ros/$ROS_DISTRO/setup.bash
          colcon test --packages-select easy_robot_control ros2_m_hero_pkg rviz_basic --event-handlers console_cohesion+
          colcon test-result --verbose
```
