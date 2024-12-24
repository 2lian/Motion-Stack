#!/bin/bash
# Generate Sphinx docs automatically
. ./install/setup.bash
sphinx-apidoc -M -f -d 2 -o ./docs/source/api ./src/easy_robot_control/easy_robot_control
# sphinx-apidoc -f -l -o source ../src/moonbot_zero_tuto/moonbot_zero_tuto
# sphinx-apidoc -f -l -o source ../src/ros2_m_hero_pkg/ros2_m_hero_pkg
cd ./docs

# rebuilds html from scratch
# make clean
make html
echo "Documentation built at docs/build/html"

# generates markdown
sphinx-build -M markdown ./source ./build

# copies markdown in root and manually adjusts path
cd ..
cp ./docs/build/markdown/manual/README.md ./README.md
prefix="docs/build/markdown/manual/"
input_file="./README.md"
sed -i "s|\[\([^]]*\)\](\([^)]*\.md.*\))|[\1]($prefix\2)|g" "$input_file"
sed -i '1s|^|<!-- This file is auto-generated from the docs. refere to ./docs/source/manual/README.rst -->\n|' README.md
echo "README.md updated"
