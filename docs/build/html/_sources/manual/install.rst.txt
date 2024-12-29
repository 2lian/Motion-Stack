Installation
============

ROS2
----

Humble (Ubuntu 22.04)
^^^^^^^^^^^^^^^^^^^^^^^^^^

Installation guide of humble: https://docs.ros.org/en/humble/Installation.html

Foxy (Ubuntu 20.04)
^^^^^^^^^^^^^^^^^^^^^^^^^^

Installation guide of foxy: https://docs.ros.org/en/foxy/Installation.html

Download the workspace
----------------------

Clone this repo in the Moonbot-Motion-Stack workspace (or anywhere else you like):

.. code-block:: bash

    cd
    git clone https://github.com/2lian/Moonbot-Motion-Stack.git
    cd Moonbot-Motion-Stack

.. Note::
   This documentation assumes your workspace is *~/Moonbot-Motion-Stack*

Use rosdep to install ROS2 dependencies automatically
-----------------------------------------------------

.. code-block:: bash

    # source ros here
    cd ~/Moonbot-Motion-Stack
    sudo rosdep init
    rosdep update
    rosdep install --from-paths src --ignore-src -r

.. caution::

    If using foxy you need to run manually: ``sudo apt install ros-foxy-xacro ros-foxy-joint-state-publisher``

Use pip to install Python dependencies automatically
----------------------------------------------------

.. code-block:: bash

    cd ~/Moonbot-Motion-Stack/src/easy_robot_control
    sudo apt install python3-pip
    pip install pip-tools
    pip-compile -o requirements.txt setup.py
    pip install -r requirements.txt --force-reinstall --upgrade
    rm -rf *.egg-info/ requirements.txt

.. Danger::

   This pip install command will **force** all of your python package to a compatible version, regardless of other installed dependencies. To handle dependencies yourself, use ``pip install -r requirements.txt``.

.. Note::

   To install the dev requirements use ``python3 -m piptools compile --extra dev -o requirements.txt setup.py``.

.. Note::

   If you have limited ram, try using ``CXXFLAGS="-fno-fat-lto-objects -O2 --param ggc-min-expand=10 --param ggc-min-heapsize=2048"  MAKEFLAGS="-j1" pip install --no-cache-dir -r requirements.txt --force-reinstall --upgrade``
    

(Testing)
---------

Those installation steps are tested regularly, from a fresh Ubuntu install, using GitHub workflow. `See the installation test routine, for more details <https://github.com/2lian/Moonbot-Motion-Stack/blob/main/.github/workflows/stepbystep.yaml>`_.

.. image:: https://github.com/2lian/Moonbot-Motion-Stack/actions/workflows/stepbystep.yaml/badge.svg
   :target: https://github.com/2lian/Moonbot-Motion-Stack/actions/workflows/stepbystep.yaml

.. literalinclude:: ../../../.github/workflows/stepbystep.yaml
      :language: yaml
