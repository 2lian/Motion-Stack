#!/bin/bash

source /opt/ros/humble/setup.sh
xacro ./hero_7dof.xacro number:=1 > ./hero_7dof1.urdf
xacro ./hero_minimal.xacro limb:=1 wheel:=1 > ./hero_minimal.urdf
xacro ./hero_wheel_module.xacro number:=11 > ./hero_wheel_module.urdf
xacro ./hero_vehicle.xacro > ./hero_vehicle.urdf
xacro ./hero_dragon.xacro > ./hero_dragon.urdf
xacro ./hero_tricycle.xacro > ./hero_tricycle.urdf
