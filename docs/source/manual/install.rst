Installation using Pixi
========================

.. image:: https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/prefix-dev/pixi/main/assets/badge/v0.json
           :target: https://pixi.sh

Pixi can install different versions of ROS2! Quickly! In a virtual environment! Please check out their fantastic work and documentation.

.. Note::

   Our ``pixi.toml`` defines an environment, it is not an installable package.

0. `Install Pixi <https://pixi.sh/latest/installation/>`_
-----------------------------------------------------------

.. code-block:: bash

   curl -fsSL https://pixi.sh/install.sh | sh

.. Critical::

   Pixi does not work if you source ROS 2: **DO NOT SOURCE** your ROS 2 installation. Simply run our commands, the ROS, python and workspace environments will automatically be enabled.

1. Download the workspace
-------------------------

.. code-block:: bash

    git clone https://github.com/2lian/Motion-Stack.git
    cd Motion-Stack

2. Install dependencies and build
----------------------------------------

Pixi will install all dependencies **including ROS** into a *venv*. The ``run`` command then calls ``colcon build`` using the *venv*.

.. code-block:: bash

   pixi install # (optional)
   pixi run -e default build

.. Note::

    The TUI requires ``ros2-keyboard`` from Christopher Mower (GPLv2 license).

    .. code-block::

       git clone https://github.com/cmower/ros2-keyboard ./src/ros2-keyboard

You are all set!
----------------

You can launch the *Moonbot Zero*:

.. code-block:: bash

   # Terminal 1
   pixi run ros2 launch motion_stack moonbot_zero.launch.py

.. code-block:: bash
    
   # Terminal 2
   pixi run bash launch_simu_rviz.bash


.. Tip::

   You can specify the ROS 2 distro using ``-e <distro>`` as in ``pixi run -e humble ...``. Remember to build for the specific distro first!


Installation from source
========================

ROS2
----

Install ROS2:

- \ `Jazzy (Ubuntu 24.04) installation guide. <https://docs.ros.org/en/jazzy/Installation.html>`_
- \ `Humble (Ubuntu 22.04) installation guide. <https://docs.ros.org/en/humble/Installation.html>`_
- \ `Foxy (Ubuntu 20.04) installation guide. <https://docs.ros.org/en/foxy/Installation.html>`_

.. Note::

   The core of the Motion-Stack is pure python, ROS2 is only the communication interface, hence necessary.
   If you are a developper wanting to use something else instead of ROS2 (pure async python for minimal overhead, Zenoh ...), you could develop your own interface.

Build tools
-----------

For installation, building, and docs, `doit <https://pydoit.org>`_ is used. It is a build tool in the vein of *Make* but arguably easier to use. *doit* is **NOT** necessary for this repo, but dealing with multiple robots, multiple ROS2 distros, such tools can help.

.. code-block:: bash

    sudo apt install python3-pip python3-doit

Download the workspace
----------------------

.. code-block:: bash

    git clone https://github.com/2lian/Motion-Stack.git
    cd Motion-Stack

.. Note::

   This documentation assumes your workspace is *~/Motion-Stack*

Automated installation using *doit*
-----------------------------------

Install ROS2 and Python dependencies:

.. code-block:: bash

    doit pydep rosdep

Build the workspace and Test python dependencies:

.. code-block:: bash

    doit -n 8 build test_import

.. dropdown:: Available doit CLI arguments and settings

    You can pass variables to *doit*, such as ``doit build syml=y``, thus changing some installation and build settings. Available config is given and can be changed in ``doit_config.py``:

    .. literalinclude:: ../../../doit_config.py
      :language: python
      :lines: 8-28

.. dropdown:: List all available doit commands with: ``doit list``

   .. code-block:: console

        build         Colcon builds packages. Uses symlink if cli_arg has 'syml=y'
        ci_badge      Copies fail/success.rst badge depending on last test result
        gitdep        Install/updates github dependencies
        html          Builds the documentation as html in docs/build/html
        md            Post processes the .md docs for github integration
        md_doc        Builds the documentation as markdown in /home/elian/Motion-Stack/docs/build/md
        pipcompile    Compiles python requirements
        pydep         Install python dependencies. If cli_arg has 'pipforce=y' pydep command will –force-reinstall –update.
        python_venv   Creates python venv if cli_arg has 'venv=y'
        rosdep        Install ROS dependencies
        test          Runs all test, using colcon test
        test_import   Fast sanity check -- Tests all python file executability

.. _install-venv:
Regarding Python dependencies and virtual environments
----------------------------------------------------------

.. Important::

    If facing pip dependencies issues, try ``doit pydep pipforce=y``. This command will **pip --force-reinstall --update** all of your python package to a compatible version, regardless of other installed pip dependencies.

.. Caution::

   My Python virtual environment support is still in its early phase. 

.. Note:: 
   You can find the venv inside ~/Motion-Stack/venv/ after executing ``doit pydep``. To install additional python dependencies in this venv, activate it with ``source ~/Motion-Stack/venv/bin/activate`` before using ``pip install ...``

   ``doit clean`` will delete this venv.

ROS2 `Jazzy with Ubuntu 24.04 requires a python virtual environment <https://docs.ros.org/en/jazzy/How-To-Guides/Using-Python-Packages.html#installing-via-a-virtual-environment>`_, this is quite tricky to use. 

     - The venv is only necessary when running Motion-Stack code. If you are using the motion stack through ROS2 messages (as opposed to the python API) and not building it (by working in you own workspace) you do not need to worry about it.
     - When developping with the Motion-Stack you must not only source the workspace, but first also source the venv using ``. venv/bin/activate``.
     - To build: source the venv, then use colcon through ``python3 -m colcon`` and not the system-wide ``colcon``.
     - Launching/running does not require the venv as the venv is part of the build and thus automatically used by the node. However, the python venv is unavailable when interpreting a launch.py file, so you cannot use venv libraries in a launcher.

My Jazzy and venv support is still in its early phase, if you want to override the global python packages (like what is done under foxy/humble) please do it manually by referring to the manual installation.

I do not plan to add similar venv support on foxy/humble in the installer, unless the need arises.

Manual installation (advanced)
------------------------------

Use rosdep to install ROS2 dependencies
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Download ``ros2-keyboard`` in ``src`` manually because it is not part of rosdep.

.. code-block:: bash

    cd ~/Motion-Stack/src
    git clone https://github.com/cmower/ros2-keyboard

Run rosdep to install all other ros2 packages.

.. code-block:: bash

    # source ros here
    cd ~/Motion-Stack
    sudo rosdep init
    rosdep update
    rosdep install --from-paths src --ignore-src -r

.. caution::

    If using foxy you need to run manually: ``sudo apt install ros-foxy-xacro ros-foxy-joint-state-publisher``

Make a venv (for Jazzy)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

    # source ros here
    cd ~/Motion-Stack
    python3 -m venv --system-site-packages ./venv
    . ./venv/bin/activate
    python3 -m pip install --upgrade pip wheel

Use pip to install Python dependencies
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

    cd ~/Motion-Stack/src/motion_stack/
    # source venv here if used
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

   If you have limited ram, try using ``CXXFLAGS="-fno-fat-lto-objects --param ggc-min-expand=10 --param ggc-min-heapsize=2048"  MAKEFLAGS="-j1" pip install --no-cache-dir -r requirements.txt --force-reinstall --upgrade``
