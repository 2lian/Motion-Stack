#!/bin/bash
# Generate Sphinx docs automatically
. ./install/setup.bash
sphinx-apidoc -M -e -f -d 2 -o ./docs/source/api ./src/easy_robot_control/easy_robot_control
sphinx-apidoc -M -e -f -d 2 -o ./docs/source/api ./src/motion_stack/motion_stack
# sphinx-apidoc -f -l -o source ../src/moonbot_zero_tuto/moonbot_zero_tuto
# sphinx-apidoc -f -l -o source ../src/ros2_m_hero_pkg/ros2_m_hero_pkg
# rm -r ./docs/build
cd ./docs

# rebuilds html from scratch
# make clean
make html
echo "Documentation built at docs/build/html"

# generates markdown
sphinx-build -M markdown ./source ./build

# copies markdown in root and manually adjusts path
cd ..
cp ./docs/build/markdown/index.md ./README.md
prefix="docs/build/markdown/"
input_file="./README.md"
sed -i "s|\[\([^]]*\)\](\([^)]*\.md.*\))|[\1]($prefix\2)|g" "$input_file"
sed -i '1s|^|<!-- This file is auto-generated from the docs. refere to ./docs/source/manual/README.rst -->\n|' README.md
newline="Clone, then open the full html documentation in your browser : \`./docs/build/html/index.html\`"
sed -i "/^# Guides:$/a $newline" README.md
newline="\ "
sed -i "/^# Guides:$/a $newline" README.md
echo "README.md updated"
