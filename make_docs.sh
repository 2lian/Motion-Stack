#!/bin/bash
# Generate Sphinx docs automatically
. ./install/setup.bash
sphinx-apidoc -f -d 2 -o ./docs/source ./src/easy_robot_control/easy_robot_control
# sphinx-apidoc -f -l -o source ../src/moonbot_zero_tuto/moonbot_zero_tuto
# sphinx-apidoc -f -l -o source ../src/ros2_m_hero_pkg/ros2_m_hero_pkg
cd ./docs
make clean
make html
echo "Documentation built at docs/build/html"
sphinx-build -M markdown ./source ./build
