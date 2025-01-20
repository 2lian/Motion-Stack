Installation
============

ROS2
----

- \ `Humble (Ubuntu 22.04) installation guide. <https://docs.ros.org/en/humble/Installation.html>`_
- \ `Foxy (Ubuntu 20.04) installation guide. <https://docs.ros.org/en/foxy/Installation.html>`_

Build tools
-----------

For installation, building, and docs, `doit <https://pydoit.org>`_ is used. It is a build tool in the vein of *Make* but arguably easier to use. *doit* is not necessary for this repo, but dealing with robots, and multiple of them, such tools can help.

.. code-block:: bash

    sudo apt install python3-pip
    pip install doit

Download the workspace
----------------------

.. code-block:: bash

    git clone https://github.com/2lian/Moonbot-Motion-Stack.git

.. Note::

   This documentation assumes your workspace is *~/Moonbot-Motion-Stack*

Do it using *doit*
--------------------

Install ROS2 and Python dependencies:

.. code-block:: bash

    doit pydep-hard rosdep

.. Warning::

   This pydep command will **--force-reinstall --update** all of your python package to a compatible version, regardless of other installed pip dependencies. Use ``doit pydep-soft`` or install manually to handle this yourself.

Build the workspace and Test python dependencies:

.. code-block:: bash

    doit -n 8 build test_import

.. dropdown:: List all available doit commands with: ``doit list``

   .. code-block:: console

        build         Colcon builds packages
        html_doc      Builds the documentation as html in docs/build/html
        main_readme   Creates ./README.md from the documentation
        md_doc        Builds the documentation as markdown in ./docs/build/md
        pipcompile    Compiles pyhton requirements
        pydep-hard    Install python dependencies using --force-reinstall --upgrade
        pydep-soft    Install python dependencies (not garanteed to work)
        rosdep        Install ROS dependencies
        test          Runs all test, using colcon test
        test_import   Fast sanity check -- Tests all python file executability

Manual installation
-------------------

Use rosdep to install ROS2 dependencies
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

    # source ros here
    cd ~/Moonbot-Motion-Stack
    sudo rosdep init
    rosdep update
    rosdep install --from-paths src --ignore-src -r

.. caution::

    If using foxy you need to run manually: ``sudo apt install ros-foxy-xacro ros-foxy-joint-state-publisher``

Use pip to install Python dependencies
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

   If you have limited ram, try using ``CXXFLAGS="-fno-fat-lto-objects --param ggc-min-expand=10 --param ggc-min-heapsize=2048"  MAKEFLAGS="-j1" pip install --no-cache-dir -r requirements.txt --force-reinstall --upgrade``


(Testing)
^^^^^^^^^

Those installation steps are tested regularly, from a fresh Ubuntu install, using GitHub workflow. `See the installation test routine, for more details <https://github.com/2lian/Moonbot-Motion-Stack/blob/main/.github/workflows/stepbystep.yaml>`_.

.. image:: https://github.com/2lian/Moonbot-Motion-Stack/actions/workflows/doit_install.yaml/badge.svg
   :target: https://github.com/2lian/Moonbot-Motion-Stack/actions/workflows/doit_install.yaml

.. literalinclude:: ../../../.github/workflows/stepbystep.yaml
      :language: yaml
