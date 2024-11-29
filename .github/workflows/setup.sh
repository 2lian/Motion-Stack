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

sudo apt install ros-humble-ros-base
sudo apt install ros-dev-tools

source /opt/ros/humble/setup.bash

# source ros here
cd ~/Moonbot-Motion-Stack
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src

cd ~/Moonbot-Motion-Stack/src/easy_robot_control
python setup.py egg_info
pip install -r *.egg-info/requires.txt
rm -rf *.egg-info/
