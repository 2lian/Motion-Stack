.. _api-label:
API
=======

I encourage you to dive into the source code and customize it to fit your robot’s unique needs. By importing the motion stack Python API into your own package or nodes, you keep customizations separate from the *core* motion stack while adapting it to each of your robot or robot module.

In this section, I’ll walk you through an example: creating a package to launch the Moonbot Zero with a different architecture and modifying the behavior of the nodes.

Make your package
-----------------

.. Note::
    Source ros2 before all those commands

Go in your workspace's source:

.. code-block:: bash

    cd ~/Moonbot-Motion-Stack/src/

Create a package with a node named lvl1:

.. code-block:: bash

    ros2 pkg create --build-type ament_python --node-name lvl1 moonbot_zero
    cd moonbot_zero

Open ``src/moonbot_zero/setup.py`` and change it like below. This will make all your .launch.py files in ``launch/`` available in the share directory of the package, so ros2 can find them

.. code-block:: bash
    :emphasize-lines: 2, 11
    :linenos:

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

Create your own launcher in ``launch/`` of your new package:

.. code-block:: bash

    cd ~/Moonbot-Motion-Stack/src/moonbot_zero
    mkdir launch
    cd launch
    touch myrobot.launch.py

.. Note::

    For the provided executable to launch your new launcher, change ``~/Moonbot-Motion-Stack/launch_stack.bash`` like so:

    .. code-block:: bash

        ...
        ros2 launch moonbot_zero myrobot.launch.py MS_up_to_level:=4

    You can then launch and see your changes with ``bash launch_stack.bash``:

        
.. _launch-api-label:

Launch API
----------

To streamline the creation of numerous nodes, the :py:mod:`easy_robot_control.launch` provides a python launch API -- essentially wrapping around ROS2's launch system. The class :py:class:`.builder.LevelBuilder` creates the nodes to be launched and its ultimate method :py:meth:`LevelBuilder.make_description` returns the launch description used by ROS2.

Warming up
^^^^^^^^^^

Right now, with the default launch there is one `robot_state_publisher` per leg. That's a bit much. So let's make it one for the whole robot.
Let's also make the movement time longer, change leg numbers and remap a few topics.

Edit your ``myrobot.launch.py`` and let us start with the default launch provided by the motion stack:

.. code-block:: python

    from easy_robot_control.launch.builder import LevelBuilder


    ROBOT_NAME = "moonbot_7"

    LEGS_DIC = {
        1: "end1",
        2: "end2",
        3: "end3",
        4: "end4",
    }
    lvl_builder = LevelBuilder(robot_name=ROBOT_NAME, leg_dict=LEGS_DIC)

    def generate_launch_description():
        return lvl_builder.make_description()

Changing params
^^^^^^^^^^^^^^^


.. code-block:: python

    ...
    new_params = {
        "std_movement_time": 10.0,
    }

    lvl_builder = LevelBuilder(
        robot_name=ROBOT_NAME, leg_dict=LEGS_DIC, params_overwrite=new_params
    )
    ...

After overwriting the ``std_movement_time`` parameter with 10 by passing it to the :py:class:`.LevelBuilder`, movements are very slow:

.. code-block:: bash

    ros2 service call /leg1/shift motion_stack_msgs/srv/TFService "{tf: {translation: {x: -100, y: 0, z: -100}, rotation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"

Changing end effector and leg numbers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


.. code-block:: python

    ...
    LEGS_DIC = {
        1: "end2",
        2: "end1",
        3: "end3",
        40: "end4",
    }
    ...

After changing the ``LEGS_DIC`` dictionary specifying which end effector correspond to each leg and passing it to :py:class:`.LevelBuilder`, leg2 is now the one at the front.

.. code-block:: bash

    ros2 service call /leg2/shift motion_stack_msgs/srv/TFService "{tf: {translation: {x: -100, y: 0, z: -100}, rotation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"

And leg 40 is the one on the right:

.. code-block:: bash

    ros2 service call /leg40/shift motion_stack_msgs/srv/TFService "{tf: {translation: {x: 20, y: 50, z: -50}, rotation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"

.. Note::

    Revert this back when you are done, otherwise you might get confused going further.

    .. code-block:: python

        LEGS_DIC = {1: "end1", 2: "end2", 3: "end3", 4: "end4"}

Overloading to have a single robot_state_publisher
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Looking at the default launching behavior, each leg has it own state publisher. This has limited usefulness for our Moobot Zero because this robot makes use of one centralized computer and not one computer per leg.


Let's change :py:meth:`.LevelBuilder.state_publisher_lvl1` to centralize the state publishers in global namespace. Comparing below with the original source code, not much changed aside from one loop and a remapping.

.. code-block:: python


    from easy_robot_control.launch.builder import LevelBuilder, Node, ParameterValue, Command, ParameterValue

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

We created a new class ``MyLevelBuilder`` that inherits the behavior of ``LevelBuilder`` and changes the one method ``state_publisher_lvl1``. Now, when ``self.state_publisher_lvl1`` is called, one ``joint_state_publisher`` and ``robot_state_publisher`` is created in the global namespace listening to the list of topics ``[leg1/joint_read, leg2/joint_read, ...]``.

.. Note::

     \ ``easy_robot_control.joint_state_publisher`` is used, it is slightly different from the default ``joint_state_publisher``. See :py:class:`easy_robot_control.lazy_joint_state_publisher.LazyJointStatePublisher`

Remapping
^^^^^^^^^

Notice in the previous example, "joint_states" topic is used differently by several nodes. They need to be remapped onto other name to avoid conflicts:

.. code-block:: python

    ...
        remappings=[
            # (intside node, outside node),
            ("joint_states", "continuous_joint_read"),
        ],
    ...

.. Note::

    Remapping and namespaces are the main way to avoid conflicts when building your modular system.

Automating modularity
^^^^^^^^^^^^^^^^^^^^^

Using python you can change the behavior of your launcher depending on where it is launch (on the robot brain, on leg #1, on leg #2, on any PC, on ground station, ...). There is no one good way to do it, so I will explain my method with a very basic example:

I define environment variables in the OS of the computer, then launch different nodes base on that. Again, overload :py:meth:`.LevelBuilder.state_publisher_lvl1` to add such functionalities.

.. code-block:: python

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

.. Note::

   This is not part of the tutorial, you do not need to make this work.

.. _own-node-label:

Loading you own node
^^^^^^^^^^^^^^^^^^^^

In the next section we will replace the default motion stack lvl1 node :py:class:`easy_robot_control.joint_state_interface.JointNode` with our own modified node, from our package. We will make the launch API load our node instead of the default.

In your launcher overload :py:meth:`.LevelBuilder.get_node_lvl1` with:

.. code-block:: python
    :emphasize-lines: 5-8

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


.. _lvl1-api-label:

Lvl1 specialization API
-----------------------

The Motion Stack low level python code is designed such that you can easily overload relevant part of the code and use it like an API in which you inject your code.

.. Note::

    After completing the previous step ":ref:`own-node-label`", modify your node ``src/moonbot_zero/moonbot_zero/lvl1.py``.

Overloading
^^^^^^^^^^^

By importing the motion stack default node of lvl1 :py:class:`easy_robot_control.joint_state_interface.JointNode`, you can overload parts of it with the code you need.


This python file:
    - Overloads :py:meth:`.JointNode.__init__` to add a timer and publisher
    - Makes a new callback for the timer, moving each joint in a sinusoidal motion.
    - Overloads :py:meth:`.JointNode.send_to_lvl0`, it now also publishes every command on a string topic ``display_angle_command``.

.. literalinclude:: ../../../src/moonbot_zero_tuto/moonbot_zero_tuto/lvl1.py
   :linenos:
   :lines: 1-5, 9-15, 22-29, 33-56, 58-
   :language: python

You can now listen to the motor commands of leg1 using:

.. code-block:: bash

   ros2 topic echo /leg1/display_angle_command

Using the API and overloading like this, you can easily add functionalities to the motion stack without creating a new whole node, and with minimal knowledge of ros2. You can:

    - Change where the data is sent and how it is formatted (like we did with the string topic).
    - Change where the data comes from and its format (like we did with the timer, you can replace it with a subscriber).

Are designed for overloading and use as API in lvl1:
 - :py:meth:`.JointNode.send_to_lvl0`
 - :py:meth:`.JointNode.send_to_lvl2`
 - :py:meth:`.JointNode.js_from_lvl0`
 - :py:meth:`.JointNode.js_from_lvl2`
 - :py:meth:`.JointNode.coming_from_lvl0`
 - :py:meth:`.JointNode.coming_from_lvl2`
 - (click to open the doc)


Injection
^^^^^^^^^

Injection consists in instantiating an object that adds functionalities to a parent object.
Right now a few injections are available in :py:mod:`easy_robot_control.injection`. The node's empty remapper attributes :py:attr:`.JointNode.lvl0_remap` and :py:attr:`.JointNode.lvl2_remap` are also meant to be swapped if necessary.

- :py:mod:`easy_robot_control.utils.state_remaper` : Remaps states names, and applies shaping functions to the state data.
- :py:meth:`easy_robot_control.injections.topic_pub.StatesToTopic` : Publishes on individual Float64 topics instead of a JointStates topic.
- :py:meth:`easy_robot_control.injections.offsetter.OffsetterLvl0` : Adds angle offsets to the output of lvl1 (and a little bit more)

Let's use all 3:

.. literalinclude:: ../../../src/moonbot_zero_tuto/moonbot_zero_tuto/lvl1.py
   :linenos:
   :emphasize-lines: 6-8,16-20, 30-32, 57
   :language: python

Running ``ros2 topic echo /leg1/display_angle_command`` you'll see that ``joint1-1`` is now ``my-new-joint``, and its value has been multiplied by 2.

Running  ``ros2 topic list | grep .*/driver`` you'll see that topics have been created, publishing the positions of the joints.

Running the code below, will add 1 radian to the output of joint1-2 (not in rviz, only on the lvl0 motor command output).

.. code-block:: bash

    ros2 service call /leg1/set_offset motion_stack_msgs/srv/SendJointState "{js: {name: [joint1-2], position: [1], velocity: [], effort: []}}"

High level API
--------------

.. error::

   This section is a work in progress.
