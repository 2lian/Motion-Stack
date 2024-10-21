#!/bin/bash
set -e -o pipefail
export M_LEG=ALL
export USE_RVIZ=TRUE

# rm -rf ./build
# rm -rf ./log
# rm -rf ./install
# chmod +x ./launch_stack.bash
. launch_stack.bash

# cd ~/Moonbot-Motion-Stack/src/urdf_packer/urdf/hero_7dof 
# xacro hero_dragon.xacro
# . install/setup.bash

# python ~/Moonbot-Motion-Stack/src/easy_robot_control/launch/hero_dragon.py
