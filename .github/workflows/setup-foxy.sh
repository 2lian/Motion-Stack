cd
git clone https://$READ_ONLY_KEY@github.com/2lian/Moonbot-Motion-Stack.git
cd Moonbot-Motion-Stack

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

sudo apt update
sudo apt upgrade

sudo apt install ros-foxy-ros-base python3-argcomplete
sudo apt install ros-dev-tools
sudo apt install python3-colcon-common-extensions

source /opt/ros/foxy/setup.bash

# source ros here
cd ~/Moonbot-Motion-Stack
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r

cd ~/Moonbot-Motion-Stack/src/easy_robot_control
sudo apt install python3-pip
pip install pip-tools # for dependencies
#python3 -m piptools compile -o requirements.txt setup.py # takes a while (2-3 min on embedded pc)
#pip install -r requirements.txt # for dependencies
python3 setup.py egg_info
pip3 install -r *.egg-info/requires.txt --force-reinstall --upgrade
rm -rf *.egg-info/

cd ~/Moonbot-Motion-Stack
colcon build --cmake-args -Wno-dev
. install/setup.bash
colcon test --packages-select easy_robot_control ros2_m_hero_pkg rviz_basic
colcon test-result --verbose
