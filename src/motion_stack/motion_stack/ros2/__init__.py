"""
ROS2 specific API and nodes, for the motion stack core.

Links ROS2 systems and and the pure pyhton core.
Timer, Messages, Publisher, Subscription ... all of those ROS2 tools are created here then executes functions of the core.

- :py:mod:`.ros2.base_node` Provides the API template to use the python core through ROS2 nodes.
- :py:mod:`.ros2.default_node` uses this API to make the default nodes.

.. Warning::

    Non-ROS2-related opertation must NOT be implemented here.

"""
