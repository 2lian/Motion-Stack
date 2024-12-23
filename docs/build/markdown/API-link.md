# Reprogram the Stack: Make It Yours.

I encourage you to dive into the source code and customize it to fit your robot’s unique needs. By importing the motion stack into your own package, you can keep your customizations separate from the core motion stack. This separation ensures that updates to either the motion stack or your code won’t interfere with each other.

In this section, I’ll walk you through an example: creating a package to launch the Moonbot Zero with a different configuration while modifying the behavior of the motion stack’s nodes.

## Setup a new package

Source ros2 before all those commands

Go in your workspace’s source:

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

package_name = 'moonbot_zero'

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
cd ~/Moonbot-Motion-Stack/src/moonbot_zero
mkdir launch
cd launch
touch myrobot.launch.py
```

Change `~/Moonbot-Motion-Stack/launch_stack.bash` like so:

```bash
#!/bin/bash

cd "${ROS2_MOONBOT_WS}" || echo No folder shortcut, working in $PWD
. "${ROS2_INSTALL_PATH}"/setup.bash || source /opt/ros/humble/setup.bash || source /opt/ros/foxy/setup.bash || echo Ros2 not found for auto-sourcing, continuing
export RCUTILS_COLORIZED_OUTPUT=1
colcon build --cmake-args -Wno-dev
. install/setup.bash
export RCUTILS_CONSOLE_OUTPUT_FORMAT="{message}"
export NUMBA_CACHE_DIR="./numba_cache" # this will compile numba in a permanant file

ros2 launch moonbot_zero myrobot.launch.py MS_up_to_level:=4 # changed line
```

## Launch modification

Right now, with the default launch there is one robot_state_publisher per leg. That’s a bit much. So let’s make it one for the whole robot.

Let’s also make the movement time longer, change leg numbers and remap a few topics.

Edit your `myrobot.launch.py` and let us start with the default launch provided by the motion stack:

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

You should be able to launch it (do this to see the effects of your changes):

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
ros2 service call /leg1/shift motion_stack_msgs/srv/TFService "{tf: {translation: {x: -100, y: 0, z: -100}, rotation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
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
ros2 service call /leg2/shift motion_stack_msgs/srv/TFService "{tf: {translation: {x: -100, y: 0, z: -100}, rotation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

And leg 40 is the one on the right:

```bash
ros2 service call /leg40/shift motion_stack_msgs/srv/TFService "{tf: {translation: {x: 20, y: 50, z: -50}, rotation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

Revert this back when you are done, otherwise you might get confused going further

```python
...

LEGS_DIC = {
    1: "end1",
    2: "end2",
    3: "end3",
    4: "end4",
}

...
```

### Overloading to have a single robot_state_publisher

Looking at the default launching behavior, you will see that each leg has it own state publishers. This has limited usefulness for our Moobot Zero because this robot makes use of one centralized computer and not one computer per leg.

Let’s change the launcher to centralize the state publishers in global namespace.

```python
...

class MyLevelBuilder(LevelBuilder):
    def state_publisher_lvl1(self) -> List[Node]:
        compiled_xacro = Command([f"xacro ", self.xacro_path])
        node_list = []
        leg_namespaces = [f"leg{param['leg_number']}" for param in self.lvl1_params()]
        all_joint_read_topics = [f"{ns}/joint_read" for ns in leg_namespaces]
        node_list.append(
            Node(
                package=self.ms_package,
                executable="joint_state_publisher",
                name="joint_state_publisher",
                # namespace=ns,
                arguments=["--ros-args", "--log-level", "warn"],
                parameters=[
                    {
                        "source_list": all_joint_read_topics,
                        "publish_default_positions": True,
                    }
                ],
                remappings=[
                    # (intside node, outside node),
                    ("joint_states", "continuous_joint_read"),
                ],
            ),
        )
        node_list.append(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                # namespace=ns,
                arguments=["--ros-args", "--log-level", "warn"],
                parameters=[
                    {
                        "robot_description": ParameterValue(
                            compiled_xacro, value_type=str
                        ),
                    }
                ],
                remappings=[
                    # (intside node, outside node),
                    ("joint_states", "continuous_joint_read"),
                ],
            ),
        )
        return node_list


...
```

We created a new class `MyLevelBuilder` that inherits the behavior of `LevelBuilder` and changes the one method `state_publisher_lvl1`. Now, when `self.state_publisher_lvl1` is called, one `joint_state_publisher` and `robot_state_publisher` is created in the global namespace listening to the list of topics `[leg1/joint_read, leg2/joint_read, ...]`.

### Remapping

Notice in the previous example, “joint_states” topic is used differently by several nodes. They need to be remapped onto other name in the launcher using the following:

```python
...
    remappings=[
        # (intside node, outside node),
        ("joint_states", "continuous_joint_read"),
    ],
...
```

This and namespaces are the main way to avoid conflicts when building your modular system.

### Automating modularity

Using python you can change the behavior of your launcher depending on where it is launch (on the robot brain, on leg #1, on leg #2, on any PC, on ground station, …). There is no one good way to do it, so I will explain my method with a very basic example:

I define environment variables in the OS of the computer, then launch different nodes base on that.

```python
class MyLevelBuilder(LevelBuilder):
    def __init__(
        self,
        robot_name: str,
        leg_dict: Mapping[int, Union[str, int]],
        params_overwrite: Dict[str, Any] = ...,
    ):
        # gets the "COMPUTER_ID" environement variable
        self.COMPUTER_ID = os.environ.get("COMPUTER_ID")
        if self.COMPUTER_ID in ["leg1", "leg2", "leg3", "leg4"]:
            # if running on one of the leg computer
            # we only start the assiciated leg/end-effector
            leg_number = int(self.COMPUTER_ID[-1])
            end_effector: Union[str, int, None] = leg_dict.get(leg_number)
            if end_effector is None:
                raise Exception("leg number has no entry in leg_dict")
            reduced_leg_dict = {leg_number: end_effector}
            leg_dict = reduced_leg_dict
        super().__init__(robot_name, leg_dict, params_overwrite)

    def make_levels(self) -> List[List[Node]]:
        if self.COMPUTER_ID in ["leg1", "leg2", "leg3", "leg4"]:
            # if running on one of the leg computer
            # we only start lvl1
            return [self.lvl1()]
        if self.COMPUTER_ID == "robot_brain":
            # if running on the main robot computer
            # we start lvl2-3-4
            return [self.lvl2(), self.lvl3(), self.lvl4()]
        if self.COMPUTER_ID == "ground_station":
            # if running on the ground station
            # we start only lvl5
            return [self.lvl5()]
        # if none of the previous cases, the default behavior runs all levels
        return super().make_levels()
```

### Loading you own node

In the next section we will replace the default motion stack lvl1 node with our own modified node, from our package. We will make the launcher load our node instead of the default.

In the launcher replace:

```python
return Node(
    ...
    package=self.ms_package,
    namespace=ns,
    executable="joint_node",
    name=f"joint_node",
    ...
)
```

with

```python
return Node(
    ...
    package="moonbot_zero",
    namespace=ns,
    executable="lvl1",
    name=f"lvl1",
    ...
)
```

Now let’s edit our own lvl1 node

## Changing behavior

The Motion Stack python code is design such that you can easily overload relevant part of the code and use it like an API in which you inject your code.

### Overloading

After completing the previous step, modify `src/moonbot_zero/moonbot_zero/lvl1.py`.

In this python file, import the joint_node of lvl1, make each joint move in a sinusoidal motion, and send every command also on a string topic, so you can listen to it.

```python
from typing import Iterable, List

from easy_robot_control.EliaNode import myMain, rosTime2Float
from easy_robot_control.joint_state_interface import JointNode
from easy_robot_control.utils.joint_state_util import JState

import numpy as np
from rclpy.node import Timer
from std_msgs.msg import String


class ZeroLvl1(JointNode):
    def __init__(self):
        super().__init__()  # default node intialization
        # our new code
        self.stringPUB = self.create_publisher(String, "display_angle_command", 10)
        self.sinTMR: Timer = self.create_timer(0.5, self.joint_sin)
        self.start_time = self.getNow()

    def joint_sin(self):
        """Makes a sinusoidal motion on every joint"""
        now = self.getNow()
        since_start = rosTime2Float(now - self.start_time)
        period = 10
        amplitue = 0.1
        sinusoid = np.sin(since_start * np.pi / period) * amplitue
        state_list: List[JState] = []
        for name in self.jointHandlerDic.keys():
            state = JState(name=name, time=now, position=sinusoid)
            state_list.append(state)
        self.coming_from_lvl2(state_list)

    def send_to_lvl0(self, states: Iterable[JState]):
        """This function is executed every time data needs to be sent down."""
        super().send_to_lvl0(states)  # executes default
        # our new code
        str_to_send: List[str] = []
        for joint_state in states:
            if joint_state.position is None:
                continue  # skips empty states with no angles
            str_to_send.append(
                f"lvl1 -> lvl0: {joint_state.name} "
                f"| {np.rad2deg(joint_state.position):.1f}"
            )
        if str_to_send:
            self.stringPUB.publish(String(data="\n".join(str_to_send)))


def main(args=None):
    myMain(ZeroLvl1)


if __name__ == "__main__":
    main()
```

You can now listen to the motor commands of leg1 using: `ros2 topic echo /leg1/display_angle_command`

Using overloading like this, you can easily add functionalities to the motion stack without adding a new whole node and with minimal knowledge of ros2. You can:

- Change where the data is sent and how it is formatted (like we did with the string topic).
- Change where the data comes from and its format (like we did with the timer, you can replace it with a subscriber).

Are designed for overloading in lvl1:

```python
class JointNode(EliaNode):
    def send_to_lvl0(self, states: Iterable[JState]):
        """Sends states to lvl0 (commands for motors).
        This function is executed every time data needs to be sent down.
        Change/overload this method with what you need"""

    def send_to_lvl2(self, states: Iterable[JState]):
        """Sends states to lvl2 (states for ik).
        This function is executed every time data needs to be sent up.
        Change/overload this method with what you need"""

    @error_catcher
    def js_from_lvl0(self, msg: JointState):
        """Callbk when a JointState arrives from the lvl0 (states from motor).
        Converts it into a list of states, then hands it to the general function
        """

    @error_catcher
    def js_from_lvl2(self, msg: JointState):
        """Callbk when a JointState arrives from the lvl2 (commands from ik).
        Converts it into a list of states, then hands it to the general function
        """

    def coming_from_lvl2(self, states: List[JState]):
        """Processes incomming commands from lvl2 ik.
        Call this function after processing the data to a List[JState].
        Always do super().coming_from_lvl0(states),
        Unless you know what you are doing"""

    def coming_from_lvl0(self, states: Iterable[JState]):
        """Processes incomming sensor states from lvl0 motors.
        Call this function after processing the data to a List[JState].
        Always do super().coming_from_lvl0(states),
        Unless you know what you are doing"""
```

### Injection

Injection is adding an object that adds functionalities to the parent object.

For now 3 injections are available:

- Remappers: Remaps states names, and applies shaping functions to the state data.
- Topic publisher: Publishes on individual Float64 topics instead of a JointStates topic.
- Offseter: Adds angle offsets to the output of lvl1 (and a little bit more)

Let’s use all 3:

```python
from typing import Iterable, List

from easy_robot_control.EliaNode import myMain, rosTime2Float
from easy_robot_control.injection.offsetter import OffsetterLvl0
from easy_robot_control.injection.topic_pub import StatesToTopic
from easy_robot_control.joint_state_interface import JointNode
from easy_robot_control.utils.joint_state_util import JState

from easy_robot_control.utils.state_remaper import Shaper, StateMap, StateRemapper
import numpy as np
from rclpy.node import Timer
from std_msgs.msg import String

remap_lvl1 = StateRemapper(
    name_map={"joint2-1": "my-new-joint"},
    state_map={"joint2-1": Shaper(position=lambda x: x * 2)},
    unstate_map={"joint2-1": Shaper(position=lambda x: x / 2)},
)


class ZeroLvl1(JointNode):
    def __init__(self):
        super().__init__()  # default node intialization
        # our new code
        self.stringPUB = self.create_publisher(String, "display_angle_command", 10)
        self.sinTMR: Timer = self.create_timer(0.5, self.joint_sin)
        self.start_time = self.getNow()
        self.lvl0_remap = remap_lvl1
        self.offsetter = OffsetterLvl0(self)
        self.state2topic = StatesToTopic(self)  # needs to be called in send_to_lvl0

    def joint_sin(self):
        """Makes a sinusoidal motion on every joint"""
        now = self.getNow()
        since_start = rosTime2Float(now - self.start_time)
        period = 10
        amplitue = 0.1
        sinusoid = np.sin(since_start * np.pi / period) * amplitue
        state_list: List[JState] = []
        for name in self.jointHandlerDic.keys():
            state = JState(name=name, time=now, position=sinusoid)
            state_list.append(state)
        self.coming_from_lvl2(state_list)

    def send_to_lvl0(self, states: Iterable[JState]):
        """This function is executed every time data needs to be sent down."""
        super().send_to_lvl0(states)  # executes default
        # our new code
        self.state2topic.publish(states)
        str_to_send: List[str] = [f"leg {self.leg_num}"]
        for joint_state in states:
            if joint_state.position is None:
                continue  # skips empty states with no angles
            str_to_send.append(
                f"lvl1 -> lvl0: {joint_state.name} "
                f"| {np.rad2deg(joint_state.position):.1f}"
            )
        if str_to_send[1:]:
            self.stringPUB.publish(String(data="\n".join(str_to_send)))


def main(args=None):
    myMain(ZeroLvl1)


if __name__ == "__main__":
    main()
```

Running  `ros2 topic echo /leg1/display_angle_command` you’ll see that `joint1-1` is now `my-new-joint`, and its value has been multiplied by 2.

Running  `ros2 topic list | grep .*/driver` you’ll see that topics have been created to publish the positions of the joints.

Running the code below, will add 1 radian to the output of joint1-2 (not in rviz, only on the motor command output).

```bash
ros2 service call /leg1/set_offset motion_stack_msgs/srv/SendJointState "{js: {name: [joint1-2], position: [1], velocity: [], effort: []}}"
```
