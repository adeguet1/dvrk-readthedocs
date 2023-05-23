ROS 2
=====

This code hasn't been tested extensively.  We welcome any feedback.
The following has been tested on Ubuntu 20.04 with ROS 2 Galactic and
Ubuntu 22.04 with ROS 2 Humble.

ROS 2 and extra packages
------------------------

Install ROS 2 following instructions from www.ros.org.  The following two packages might not be installed by default:

.. code-block:: bash

   sudo apt install python3-vcstool python3-colcon-common-extensions # for colcon
   sudo apt install python3-pykdl # for the CRTK Python client library


For cisst/SAW and dVRK, you will also need the following Ubuntu packages:

.. tabs::

   .. tab:: Ubuntu 20.04

      .. code-block:: bash

	 sudo apt install libxml2-dev libraw1394-dev libncurses5-dev qtcreator swig sox espeak cmake-curses-gui cmake-qt-gui git subversion gfortran libcppunit-dev libqt5xmlpatterns5-dev libbluetooth-dev
	 sudo apt install ros-galactic-joint-state-publisher* ros-galactic-xacro

   .. tab:: Ubuntu 22.04

      .. code-block:: bash

	 sudo apt install libxml2-dev libraw1394-dev libncurses5-dev qtcreator swig sox espeak cmake-curses-gui cmake-qt-gui git subversion gfortran libcppunit-dev libqt5xmlpatterns5-dev libbluetooth-dev gfortran-9
	 sudo apt install ros-humble-joint-state-publisher* ros-humble-xacro

Compile cisst/SAW components with ROS dependencies
--------------------------------------------------

Create your ROS 2 workspace and clone all repositories using ``vcs``:

.. code-block:: bash

   source /opt/ros/galactic/setup.bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   vcs import --input https://raw.githubusercontent.com/jhu-dvrk/dvrk_robot_ros2/main/dvrk.vcs --recursive

.. warning:: The URL used as input for ``vcs import`` might be different based on which branches you're using.

.. warning:: If you forgot the ``--recursive`` option, go in ``~/ros2_ws/src/cisst-saw/sawRobotIO1394`` and run ``git submodule init; git submodule update`` (this is to pull the "AmpIO" code).

Compile using ``colcon``:

.. code-block:: bash
   cd ~/ros2_ws
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
   source ~/ros2_ws/install/setup.bash

.. note:: `colcon build` is not as smart as ``catkin build``: you need
   to be in the top directory of your workspace to build (for example
   ``~/ros2_ws``).  Do not try to build in a sub-directory in your
   workspace, colcon will create a new set of ``build``, ``install``
   and ``log`` directories.  ``catkin build`` recursively look in
   parent directories until it finds the workspace root, ``colcon``
   doesn't.

ROS 2 broadcasts (a lot)
------------------------

By default, ROS2 broadcasts messages based on your network mask
settings.  If you have multiple computers on the same subnet/mask,
they will all share the same "space" by default.  So if you start 2
instances of the dVRK console they will use the same topics, services,
tf names...  This is a bit dangerous as you might be controlling
someone else's robot by accident.  There are multiple ways to handle
this but here are two simple solutions that should cover most cases:

* All your ROS node will be on the same computer and nobody else has
  nodes running on the same computer, use the local host only approach

* Your nodes might be spread on multiple computers or there's a chance
  another user has ROS nodes on any of the computers you're using,
  domain ID will work (as long as no one uses the same ID)

If you use any of the methods below and need to test the results, make
sure you stop and restart the ROS 2 daemon after your
``export``/``unset`` since it will cache some of the discovery
information: ``ros2 daemon stop; ros2 daemon start``

Local host
^^^^^^^^^^

You can set a unique ROS Domain ID , either in your own ``~/.profile``
or for all users with ``/etc/profile.d/ros2.sh``.

.. code-block:: bash

   export ROS_LOCALHOST_ONLY=1

Note that the variable ``ROS_LOCALHOST_ONLY`` just has to be defined.
Setting it to ``0`` doesn't turn this feature off, you would have to
use ``unset`` to disable the local host only broadcast.

Domain ID
^^^^^^^^^

You can set a unique ROS Domain ID , either in your own ``~/.profile``
or for all users with ``/etc/profile.d/ros2.sh``.

.. code-block:: bash

   export ROS_DOMAIN_ID=33

If your organization uses a centralized authentication server (SSO),
one can use the Unix user ID to define the ROS Domain ID.
Unfortunately the domain ID should be between 0 and 101 (see
[ROS_DOMAIN_ID](https://docs.ros.org/en/humble/Concepts/About-Domain-ID.html))
so we can't use the full Unix user Id To automatically set the ROS
Domain ID.  The following configuration file will generate the domain
ID based on the last 2 digits of the user ID.  Create or edit the file
``/etc/profile.d/ros2.sh`` to contain:

.. code-block:: bash

   # set domain id based on last 2 digits of user id
   export ROS_DOMAIN_ID=$(id -u | rev | cut -c 1-2 | rev)

.. warning:: Since this relies on the last two digits of the user ID,
   there is still a strong possibility 2 users will have the same ROS
   Domain ID.  Make sure you run ``ros2 node list`` to check nobody is
   using your domain.

Usage
-----

Example of session
^^^^^^^^^^^^^^^^^^

* Terminal 1: starting the dVRK main console
  * with a real system:
    .. code-block:: bash

       source ~/ros2_ws/install/setup.bash
       cd ~/ros2_ws/install/dvrk_config_jhu # we assume each group has created their own configuration file repository!
       ros2 run dvrk_robot dvrk_console_json -j share/jhu-dVRK-Si/console-PSM1.json

  * with a simulated arm:
    .. code-block:: bash

       source ~/ros2_ws/install/setup.bash
       cd ~/ros2_ws/install/sawIntuitiveResearchKitAll/share/sawIntuitiveResearchKit
       ros2 run dvrk_robot dvrk_console_json -j share/console/console-PSM1_KIN_SIMULATED.json

* Terminal 2: using a Python test script to make the arm move
  .. code-block:: bash

     source ~/ros2_ws/install/setup.bash
     ros2 run dvrk_python dvrk_arm_test.py -a PSM1

* Terminal 3: starting the ROS 2 joint and robot state publishers so we can visualize the arm in RViz
  .. code-block:: bash

     source ~/ros2_ws/install/setup.bash
     ros2 launch dvrk_model dvrk_state_publisher.launch.py arm:=PSM1

* Terminal 4: starting RViz
  .. code-block:: bash

     source ~/ros2_ws/install/setup.bash
     ros2 run rviz2 rviz2 -d ~/ros2_ws/install/dvrk_model/share/dvrk_model/rviz/PSM1.rviz

Note that all the configuration files are installed in the
``ros2_ws/install`` directory during the build so you can
automatically locate them when you write your own ROS launch files.

Useful commands
^^^^^^^^^^^^^^^

* tf2 to pdf: ``ros2 run tf2_tools view_frames`` (then ``evince frames.pdf`` to view)
