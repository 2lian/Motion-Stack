<a id="api-label"></a>

# API

I encourage you to dive into the source code and customize it to fit your robot’s unique needs. By importing the motion stack Python API into your own package and nodes, you keep customizations separate from the *core* motion stack while adapting it to each of your robot or robot module.

In this section, I’ll walk you through an example: creating a package to launch the Moonbot Zero with a different architecture and modifying the behavior of the nodes.

## Make your package

#### NOTE
Source ros2 before all those commands.

Go in your workspace’s source:

```bash
cd ~/Moonbot-Motion-Stack/src/
```

Create a package with a node named lvl1:

```bash
ros2 pkg create --build-type ament_python --node-name lvl1 moonbot_zero
```

Open `src/moonbot_zero/setup.py` and change it like below. This will make available in the share sirectory:

> - All your .launch.py files in `launch/` , so ros2 can find them.
> - All your meshes/ and urdf/ available
```python
from setuptools import find_packages, setup
from glob import glob # add this line

package_name = 'moonbot_zero'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (f"share/{package_name}/launch", glob("launch/*.py")),
        (f"share/{package_name}/urdf", glob("urdf/*", recursive=True)), # (Optional)
        (f"share/{package_name}/meshes", glob("meshes/*", recursive=True)), # (Optional)
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

#### NOTE
For the provided executable to launch your new launcher, change `~/Moonbot-Motion-Stack/launch_stack.bash` like so:

```bash
...
ros2 launch moonbot_zero myrobot.launch.py MS_up_to_level:=4
```

You can then launch and see your changes with `bash launch_stack.bash`:

## Using your URDF

### Making a URDF available from your custom package

In a ros package (here named *moonbot_zero*), create a `urdf/` and `meshes/` directories, then place you urdfs and meshes inside.

```bash
cd ~/Moonbot-Motion-Stack/src/moonbot_zero
mkdir meshes
mkdir urdf
```

Make those directories available in the package shared directory by changing the `setup.py`

```python
...

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (f"share/{package_name}/launch", glob("launch/*.py")),
        (f"share/{package_name}/urdf", glob("urdf/*", recursive=True)), # (this)
        (f"share/{package_name}/meshes", glob("meshes/*", recursive=True)), # (this)
        ]
    )
```

Make sure to adjust the paths of the urdf. `<mesh filename="{SOMETHING}/base_link.stl" />` should be `<mesh filename="package://moonbot_zero/meshes/base_link.stl" />` (where *moonbot_zero* is the name of the package).

### Loading your URDF

Using the launch API in the next section, you can load a URDF by providing the package name and path. Assuming the package is *moonbot_zero* and the URDF is inside `urdf/moonbot_zero.xacro`:

```python
urdf_path=xacro_path_from_pkg(
    package_name="moonbot_zero",
    xacro_path="urdf/moonbot_zero.xacro",
)
```

<a id="launch-api-label"></a>

## Launch API

To streamline the creation of numerous nodes, the [`motion_stack.api.launch`](../api/motion_stack/motion_stack.api.launch.md#module-motion_stack.api.launch) provides a python launch API – essentially wrapping around ROS2’s launch system. The class [`api.launch.builder.LevelBuilder`](../api/motion_stack/motion_stack.api.launch.md#motion_stack.api.launch.builder.LevelBuilder) creates the nodes to be launched and its ultimate method [`api.launch.builder.LevelBuilder.make_description()`](../api/motion_stack/motion_stack.api.launch.md#motion_stack.api.launch.builder.LevelBuilder.make_description) returns the launch description used by ROS2.

### Warming up

Edit your `myrobot.launch.py` and let us start with the default launch provided by the motion stack:

```python
from motion_stack.api.launch.builder import (
    LevelBuilder,
    xacro_path_from_pkg,
)


ROBOT_NAME = "moonbot_7"

LEGS_DIC = {
    1: "end1",
    2: "end2",
    3: "end3",
    4: "end4",
}

lvl_builder = LevelBuilder(
    urdf_path=xacro_path_from_pkg(
        package_name="moonbot_zero", xacro_path="urdf/moonbot_zero.xacro"
    ),
    leg_dict=LEGS_DIC,
)

def generate_launch_description():
    return lvl_builder.make_description()
```

### Changing params

```python
...
new_params = {
    "std_movement_time": 10.0,
}

lvl_builder = LevelBuilder(
    urdf_path=xacro_path_from_pkg(
        package_name="moonbot_zero", xacro_path="urdf/moonbot_zero.xacro"
    ),
    leg_dict=LEGS_DIC,
    params_overwrite=new_params,
)
...
```

After overwriting the `std_movement_time` parameter with 10 by passing it to the [`LevelBuilder`](../api/easy_robot_control/easy_robot_control.launch.md#easy_robot_control.launch.builder.LevelBuilder), movements are very slow:

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

After changing the `LEGS_DIC` dictionary specifying which end effector correspond to each leg and passing it to [`LevelBuilder`](../api/easy_robot_control/easy_robot_control.launch.md#easy_robot_control.launch.builder.LevelBuilder), leg2 is now the one at the front.

```bash
ros2 service call /leg2/shift motion_stack_msgs/srv/TFService "{tf: {translation: {x: -100, y: 0, z: -100}, rotation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

And leg 40 is the one on the right:

```bash
ros2 service call /leg40/shift motion_stack_msgs/srv/TFService "{tf: {translation: {x: 20, y: 50, z: -50}, rotation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

#### NOTE
Revert this back when you are done, otherwise you might get confused going further.

```python
LEGS_DIC = {1: "end1", 2: "end2", 3: "end3", 4: "end4"}
```

### Overloading to have a single robot_state_publisher

Looking at the default launching behavior, each leg has it own state publisher. This has limited usefulness for our Moobot Zero because this robot makes use of one centralized computer and not one computer per leg.

Let’s change `api.launch.LevelBuilder.state_publisher_lvl1()` to centralize the state publishers in global namespace. Comparing below with the original source code, not much changed aside from one loop and a remapping.

```python
...
from typing import Any, Dict, List, Mapping, Union
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from motion_stack.api.launch.builder import LevelBuilder, xacro_path_from_pkg
from launch.substitutions import Command

class MyLevelBuilder(LevelBuilder):
    def state_publisher_lvl1(self) -> List[Node]:
        compiled_xacro = Command([f"xacro ", self.xacro_path])
        node_list = []
        leg_namespaces = [f"leg{param['leg_number']}" for param in self.lvl1_params()]
        all_joint_read_topics = [f"{ns}/joint_read" for ns in leg_namespaces]
        node_list.append(
            Node(
                package=self.MS_PACKAGE,
                executable="lazy_joint_state_publisher",
                name="lazy_joint_state_publisher",
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

We created a new class `MyLevelBuilder` that inherits the behavior of `LevelBuilder` and overwrites the one method `state_publisher_lvl1`. Now, when `self.state_publisher_lvl1` is called, one `joint_state_publisher` and `robot_state_publisher` is created in the global namespace listening to the list of topics `[leg1/joint_read, leg2/joint_read, ...]`.

#### NOTE
`lazy_joint_state_publisher` is used, it is slightly different from the default `joint_state_publisher`. See [`motion_stack.ros2.utils.lazy_joint_state_publisher.LazyJointStatePublisher`](../api/motion_stack/motion_stack.ros2.utils.md#motion_stack.ros2.utils.lazy_joint_state_publisher.LazyJointStatePublisher)

### Remapping

Notice in the previous example, “joint_states” topic is used differently by several nodes. They need to be remapped onto other name to avoid conflicts:

```python
...
    remappings=[
        # (intside node, outside node),
        ("joint_states", "continuous_joint_read"),
    ],
...
```

#### NOTE
Remapping and namespaces are the main way to avoid conflicts when building your modular system.

### Automating modularity

Using python you can change the behavior of your launcher depending on where it is launch (on the robot brain, on leg #1, on leg #2, on any PC, on ground station, …). There is no one good way to do it, so I will explain my method with a very basic example:

I define environment variables in the OS of the computer, then launch different nodes base on that. Again, overwrite `api.launch.LevelBuilder.state_publisher_lvl1()` to add such functionalities.

```python
class MyLevelBuilder(LevelBuilder):
    def __init__(
        self,
        urdf_path: str,
        leg_dict: Mapping[int, Union[str, int]],
        params_overwrite: Dict[str, Any] = dict(),
        urdf: Union[None, str, Command] = None,
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
        super().__init__(urdf_path, leg_dict, params_overwrite, urdf)

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
        # if none of the previous cases, the default behavior runs everything
        return super().make_levels()
```

#### NOTE
This is not part of the tutorial, you do not need to make this work.

<a id="own-node-label"></a>

### Loading you own node

In the next section we will replace the default motion stack lvl1 node [`easy_robot_control.joint_state_interface.JointNode`](../api/easy_robot_control/easy_robot_control.md#easy_robot_control.joint_state_interface.JointNode) with our own modified node, from our package. We will make the launch API load our node instead of the default.

In your launcher overload [`LevelBuilder.get_node_lvl1()`](../api/easy_robot_control/easy_robot_control.launch.md#easy_robot_control.launch.builder.LevelBuilder.get_node_lvl1) with:

```python
class MyLevelBuilder(LevelBuilder):
    def get_node_lvl1(self, params: Dict[str, Any]) -> Node:
        ns = f"leg{params['leg_number']}"
        return Node(
            package="moonbot_zero",
            namespace=ns,
            executable="lvl1",
            name=f"lvl1",
            arguments=["--ros-args", "--log-level", "info"],
            emulate_tty=True,
            output="screen",
            parameters=[params],
            remappings=self.remaplvl1,
        )
```

<a id="lvl1-api-label"></a>

## Lvl1 specialization API

The Motion Stack low level python code is designed such that you can easily overload relevant part of the code and use it like an API in which you inject your code.

#### NOTE
After completing the previous step “[Loading you own node](#own-node-label)”, modify your node `src/moonbot_zero/moonbot_zero/lvl1.py`.

### Overloading

By importing the motion stack default node of lvl1 [`easy_robot_control.joint_state_interface.JointNode`](../api/easy_robot_control/easy_robot_control.md#easy_robot_control.joint_state_interface.JointNode), you can overload parts of it with the code you need.

This python file:
: - Overloads `JointNode.__init__()` to add a timer and publisher
  - Makes a new callback for the timer, moving each joint in a sinusoidal motion.
  - Overloads [`JointNode.send_to_lvl0()`](../api/easy_robot_control/easy_robot_control.md#easy_robot_control.joint_state_interface.JointNode.send_to_lvl0), it now also publishes every command on a string topic `display_angle_command`.

```python
"""Overloaded JointNode for Moonbot zero."""

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
        self.sinusiodTMR: Timer = self.create_timer(0.1, self.send_sinusoid)
        self.start_time = self.getNow()

    def send_sinusoid(self):
        """Sends a sinusoidal command on every joint based on the clock.
        Callback of the self.sinTMR timer."""
        # sinusoid creation
        now = self.getNow()
        since_start = rosTime2Float(now - self.start_time)
        PERIOD = 10
        AMPLITUDE = 0.1
        sinusoid = np.sin(since_start * np.pi / PERIOD) * AMPLITUDE

        # joint states data creation
        state_list: List[JState] = []
        for name in self.jointHandlerDic.keys():
            state = JState(name=name, time=now, position=sinusoid)
            state_list.append(state)

        # call of the function handling incomming joint commands from lvl2
        self.coming_from_lvl2(state_list)

    def send_to_lvl0(self, states: List[JState]):
        """Executes our custom code every time state data (commands) needs to be sent down to lvl0."""
        super().send_to_lvl0(states)  # executes default behavior
        # our new code

        # prepares string
        str_to_send: List[str] = [f"leg {self.leg_num}"]
        for joint_state in states:
            if joint_state.position is None:
                continue  # skips empty states with no angles
            str_to_send.append(
                f"lvl1 -> lvl0: {joint_state.name} "
                f"| {np.rad2deg(joint_state.position):.1f}"
            )

        # publishes string
        if str_to_send[1:]:
            self.stringPUB.publish(String(data="\n".join(str_to_send)))


def main(args=None):
    myMain(ZeroLvl1)


if __name__ == "__main__":
    main()
```

You can now listen to the motor commands of leg1 using:

```bash
ros2 topic echo /leg1/display_angle_command
```

Using the API and overloading like this, you can easily add functionalities to the motion stack without creating a new whole node, and with minimal knowledge of ros2. You can:

> - Change where the data is sent and how it is formatted (like we did with the string topic).
> - Change where the data comes from and its format (like we did with the timer, you can replace it with a subscriber).

Are designed for overloading and use as API in lvl1:
: - [`JointNode.send_to_lvl0()`](../api/easy_robot_control/easy_robot_control.md#easy_robot_control.joint_state_interface.JointNode.send_to_lvl0)
  - [`JointNode.send_to_lvl2()`](../api/easy_robot_control/easy_robot_control.md#easy_robot_control.joint_state_interface.JointNode.send_to_lvl2)
  - [`JointNode.js_from_lvl0()`](../api/easy_robot_control/easy_robot_control.md#easy_robot_control.joint_state_interface.JointNode.js_from_lvl0)
  - [`JointNode.js_from_lvl2()`](../api/easy_robot_control/easy_robot_control.md#easy_robot_control.joint_state_interface.JointNode.js_from_lvl2)
  - [`JointNode.coming_from_lvl0()`](../api/easy_robot_control/easy_robot_control.md#easy_robot_control.joint_state_interface.JointNode.coming_from_lvl0)
  - [`JointNode.coming_from_lvl2()`](../api/easy_robot_control/easy_robot_control.md#easy_robot_control.joint_state_interface.JointNode.coming_from_lvl2)
  - (click to open the doc)

### Injection

Injection consists in instantiating an object that adds functionalities to a parent object.
Right now a few injections are available in [`easy_robot_control.injection`](../api/easy_robot_control/easy_robot_control.injection.md#module-easy_robot_control.injection). The node’s empty remapper attributes [`JointNode.lvl0_remap`](../api/easy_robot_control/easy_robot_control.md#easy_robot_control.joint_state_interface.JointNode.lvl0_remap) and [`JointNode.lvl2_remap`](../api/easy_robot_control/easy_robot_control.md#easy_robot_control.joint_state_interface.JointNode.lvl2_remap) are also meant to be swapped if necessary.

- [`easy_robot_control.utils.state_remaper`](../api/easy_robot_control/easy_robot_control.utils.md#module-easy_robot_control.utils.state_remaper) : Remaps states names, and applies shaping functions to the state data.
- [`easy_robot_control.injection.topic_pub.StatesToTopic()`](../api/easy_robot_control/easy_robot_control.injection.md#easy_robot_control.injection.topic_pub.StatesToTopic) : Publishes on individual Float64 topics instead of a JointStates topic.
- [`easy_robot_control.injection.offsetter.OffsetterLvl0()`](../api/easy_robot_control/easy_robot_control.injection.md#easy_robot_control.injection.offsetter.OffsetterLvl0) : Adds angle offsets to the output of lvl1 (and a little bit more)

Let’s use all 3:

```python
"""Overloaded JointNode for Moonbot zero."""

from typing import Iterable, List

from easy_robot_control.EliaNode import myMain, rosTime2Float
from easy_robot_control.injection.offsetter import OffsetterLvl0
from easy_robot_control.injection.topic_pub import StatesToTopic
from easy_robot_control.utils.state_remaper import Shaper, StateRemapper
from easy_robot_control.joint_state_interface import JointNode
from easy_robot_control.utils.joint_state_util import JState

import numpy as np
from rclpy.node import Timer
from std_msgs.msg import String

remap_lvl1 = StateRemapper(
    name_map={"joint1-1": "my-new-joint"}, # changes the name of "joint1-1" is present
    state_map={"joint1-1": Shaper(position=lambda x: x * 2)}, # multiplies command by 2
    unstate_map={"joint1-1": Shaper(position=lambda x: x / 2)}, # divides sensor by 2
)


class ZeroLvl1(JointNode):
    def __init__(self):
        super().__init__()  # default node intialization
        # our new code
        self.stringPUB = self.create_publisher(String, "display_angle_command", 10)
        self.sinusiodTMR: Timer = self.create_timer(0.1, self.send_sinusoid)
        self.start_time = self.getNow()
        self.lvl0_remap = remap_lvl1 # Replaces default empty remap by our own
        self.offsetter = OffsetterLvl0(self) # Enables the offsetter, that's it
        self.state2topic = StatesToTopic(self)  # Publishing to be called in send_to_lvl0

    def send_sinusoid(self):
        """Sends a sinusoidal command on every joint based on the clock.
        Callback of the self.sinTMR timer."""
        # sinusoid creation
        now = self.getNow()
        since_start = rosTime2Float(now - self.start_time)
        PERIOD = 10
        AMPLITUDE = 0.1
        sinusoid = np.sin(since_start * np.pi / PERIOD) * AMPLITUDE

        # joint states data creation
        state_list: List[JState] = []
        for name in self.jointHandlerDic.keys():
            state = JState(name=name, time=now, position=sinusoid)
            state_list.append(state)

        # call of the function handling incomming joint commands from lvl2
        self.coming_from_lvl2(state_list)

    def send_to_lvl0(self, states: List[JState]):
        """Executes our custom code every time state data (commands) needs to be sent down to lvl0."""
        super().send_to_lvl0(states)  # executes default behavior
        # our new code
        self.state2topic.publish(states)

        # prepares string
        str_to_send: List[str] = [f"leg {self.leg_num}"]
        for joint_state in states:
            if joint_state.position is None:
                continue  # skips empty states with no angles
            str_to_send.append(
                f"lvl1 -> lvl0: {joint_state.name} "
                f"| {np.rad2deg(joint_state.position):.1f}"
            )

        # publishes string
        if str_to_send[1:]:
            self.stringPUB.publish(String(data="\n".join(str_to_send)))


def main(args=None):
    myMain(ZeroLvl1)


if __name__ == "__main__":
    main()
```

Running `ros2 topic echo /leg1/display_angle_command` you’ll see that `joint1-1` is now `my-new-joint`, and its value has been multiplied by 2.

Running  `ros2 topic list | grep .*/driver` you’ll see that topics have been created, publishing the positions of the joints.

Running the code below, will add 1 radian to the output of joint1-2 (not in rviz, only on the lvl0 motor command output).

```bash
ros2 service call /leg1/set_offset motion_stack_msgs/srv/SendJointState "{js: {name: [joint1-2], position: [1], velocity: [], effort: []}}"
```

## High level API
