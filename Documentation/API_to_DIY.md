# API to DIY

Let's create a package to launch the moonbot zero but differently. And change the behaviors of the motion stack's nodes.

## Setup a new package

Source ros2 before all those commands

Go in you workspace source:
```bash
cd ~/Moonbot-Motion-Stack/src/
```

Create a package with a node named lvl1:
```bash
ros2 pkg create --build-type ament_python --node-name lvl1 moonbot_zero
cd moonbot_zero
```

Open `src/moonbot_zero/setup.py` and change it like below. This will make all your .launch.py files in `launch/` available in the share folder of the package so ros2 can find them
```python
from setuptools import find_packages, setup
from glob import glob # add this line

package_name = 'moonbot_zero_tuto'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (f"share/{package_name}/launch", glob("launch/*.launch.py")), # add this line
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ...
```

Create your own launcher in `launch/` of your new package:
```bash
mkdir launch
cd launch
touch myrobot.launch.py
```

Change launch_stack.bash like so:
```bash
#!/bin/bash

cd "${ROS2_MOONBOT_WS}" || echo No folder shortcut, working in $PWD
. "${ROS2_INSTALL_PATH}"/setup.bash || source /opt/ros/humble/setup.bash || source /opt/ros/foxy/setup.bash || echo Ros2 not found for auto-sourcing, continuing
export RCUTILS_COLORIZED_OUTPUT=1
colcon build --cmake-args -Wno-dev
. install/setup.bash
export RCUTILS_CONSOLE_OUTPUT_FORMAT="{message}"
export NUMBA_CACHE_DIR="./numba_cache" # this will compile numba in a permanant file

ros2 launch moonbot_zero myrobot.launch.py MS_up_to_level:=4
```


## Launch modification

Right now, with the default launch there is one robot_state_publisher per leg. That's a bit much. So let's make it one for the whole robot.

Let's also lower the refresh rate, change leg numbers, remap a few topics and add a new motor node to wait for before starting up.

Edit your `myrobot.launch.py` and lets start with the default launch provided by the motion stack:
```python
from easy_robot_control.launch.builder import LevelBuilder


ROBOT_NAME = "moonbot_7"  # name of the xacro to load

LEGS_DIC = {
    1: "end1",
    2: "end2",
    3: "end3",
    4: "end4",
}

lvl_builder = LevelBuilder(robot_name=ROBOT_NAME, leg_dict=LEGS_DIC)

def generate_launch_description():
    return lvl_builder.make_description()
```

You should be able to launch it:
```bash
bash launch_stack.bash
```

### Changing params


```python
...

new_params = {
    "std_movement_time": 10.0,
}

lvl_builder = LevelBuilder(
    robot_name=ROBOT_NAME, leg_dict=LEGS_DIC, params_overwrite=new_params
)

...
```

Now, movements are very slow:
```bash
ros2 service call /leg1/shift custom_messages/srv/TFService "{tf: {translation: {x: -100, y: 0, z: -100}, rotation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

Also, the minimum update rate of all joint is around 2Hz, so if nothing happens it will publish at 2Hz. With the above command, the joint1-1 does not move, so the full tf tree for joint1-2 and 1-3 is not updated fast, and it seems like the robot is moving at 2Hz. In reality the commands are sent at 10Hz (default), it is simply Rviz visuals of the full tf tree that is not updated.

If you do a movement with also joint1-1 moving, you'll see the display updating faster:
```bash
ros2 service call /leg1/shift custom_messages/srv/TFService "{tf: {translation: {x: -100, y: 50, z: -100}, rotation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

### Changing end effector and leg numbers


```python
...

LEGS_DIC = {
    1: "end2",
    2: "end1",
    3: "end3",
    40: "end4",
}

...
```

Now, leg2 is the one at the front:
```bash
ros2 service call /leg2/shift custom_messages/srv/TFService "{tf: {translation: {x: -100, y: 0, z: -100}, rotation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

And leg 40 is the one on the right:
```bash
ros2 service call /leg40/shift custom_messages/srv/TFService "{tf: {translation: {x: 20, y: 50, z: -50}, rotation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

### Overloading to have a single robot_state_publisher

```python
from typing import Any,   List, Mapping,  Union

from easy_robot_control.launch.builder import LevelBuilder, Node, ParameterValue
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch.substitutions import Command


class MyLevelBuilder(LevelBuilder):
    def __init__(
        self,
        robot_name: str,
        leg_dict: Mapping[int, Union[str, int]],
        params_overwrite: dict[str, Any] = ...,
    ):
        super().__init__(robot_name, leg_dict, params_overwrite)

    def state_publisher_lvl1(self) -> List[Node]:
        compiled_xacro = Command([f"xacro ", self.xacro_path])
        node_list = []
        repeat_state_onto = "/global_joint_read"
        node_list.append(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                namespace="",
                parameters=[
                    {
                        "robot_description": ParameterValue(
                            compiled_xacro, value_type=str
                        ),
                    }
                ],
                remappings=[
                    # (intside node, outside node),
                    ("joint_states", repeat_state_onto),
                ],
            ),
        )
        for param in self.lvl1_params():
            leg_namespace = f"leg{param['leg_number']}"
            node_list.append(
                Node(
                    package="topic_tools",
                    executable="relay",
                    name="relay",
                    namespace=leg_namespace,
                    parameters=[
                        {
                            "input_topic": "joint_read",
                            "output_topic": repeat_state_onto,
                        }
                    ],
                ),
            )
        return node_list


...

lvl_builder = MyLevelBuilder(
    robot_name=ROBOT_NAME, leg_dict=LEGS_DIC, params_overwrite=new_params
)


...
```

We created a new class `MyLevelBuilder` that changes the behavior of `LevelBuilder`. When `self.state_publisher_lvl1` is called, now, one robot_state_publisher is created in the global namespace, listening to `/global_joint_read`, and relay nodes from topic_tools repeat all the `leg?/joint_read` messages onto this topic.
