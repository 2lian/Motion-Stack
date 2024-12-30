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

Build tools
-----------

Python's *doit*, a build tool similar to *Make* but arguably easier to use, is use for installation, building, and docs. *doit* is not necessary for this repo, but dealing with robots, and multiple of them, such tools can greatly simplify a system.

.. code-block:: bash

    sudo apt install python3-pip
    pip install doit

Download the workspace
----------------------

.. code-block:: bash

    cd
    git clone https://github.com/2lian/Moonbot-Motion-Stack.git
    cd Moonbot-Motion-Stack

.. Note::

   This documentation assumes your workspace is *~/Moonbot-Motion-Stack*

Install through doit
--------------------

Install python dependencies:

.. code-block:: bash

    doit pydep-hard

.. Warning::

   This pydep command will **--force-reinstall --update** all of your python package to a compatible version, regardless of other installed pip dependencies. To handle dependencies yourself, use ``doit pydep-soft``.

Install ROS2 dependencies, Build the workspace and Test python dependencies:

.. code-block:: bash

    doit -n 8 rosdep build test_import

Build the html documentation:

.. code-block:: bash

    doit html_doc

Built documentation can be found in ``./docs/build/html``.

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

.. image:: https://github.com/2lian/Moonbot-Motion-Stack/actions/workflows/stepbystep.yaml/badge.svg
   :target: https://github.com/2lian/Moonbot-Motion-Stack/actions/workflows/stepbystep.yaml

.. literalinclude:: ../../../.github/workflows/stepbystep.yaml
      :language: yaml
