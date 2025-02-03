#!/bin/bash

source /opt/ros/humble/setup.sh
xacro ./hero_7dofm1.xacro > ./hero_7dofm1.urdf
xacro ./hero_minimal.xacro > ./hero_minimal.urdf
xacro ./hero_wheel_module.xacro > ./hero_wheel_module.urdf
xacro ./hero_dragon.xacro > ./hero_dragon.urdf
xacro ./hero_3leg.xacro > ./hero_3leg.urdf
xacro ./hero_3legwheel.xacro > ./hero_3legwheel.urdf
xacro ./hero_vehicle.xacro > ./hero_vehicle.urdf
