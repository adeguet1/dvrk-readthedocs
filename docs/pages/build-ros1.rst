ROS1
====

We suggest 2 different ways to retrieve all the dVRK required source
repositories and compile them.  These methods are for Linux/ROS users
only and rely on the catkin build tools.

`catkin build` and `rosinstall`
-------------------------------

The `rosinstall` configuration file is provided in dVRK 2.x and higher
but can also be used with older versions (see
https://github.com/jhu-dvrk/dvrk-ros).  The `rosinstall` file defines
all the github repositories that need to be cloned in your workspace
as well as which branches.

Debian packages
^^^^^^^^^^^^^^^

This section assumes you already have ROS installed (see https://www.ros.org).  You will need to install a few more packages for the dVRK software:
* **Ubuntu 16.04** with ROS Kinetic: ``sudo apt-get install libxml2-dev libraw1394-dev libncurses5-dev qtcreator swig flite sox espeak cmake-curses-gui cmake-qt-gui libopencv-dev git subversion gfortran libcppunit-dev qt5-default python-wstool python-catkin-tools``
* **Ubuntu 18.04** with ROS Melodic: ``sudo apt install libxml2-dev libraw1394-dev libncurses5-dev qtcreator swig sox espeak cmake-curses-gui cmake-qt-gui git subversion gfortran libcppunit-dev libqt5xmlpatterns5-dev  libbluetooth-dev python-wstool python-catkin-tools``
* **Ubuntu 20.04** with ROS Noetic:``sudo apt install libxml2-dev libraw1394-dev libncurses5-dev qtcreator swig sox espeak cmake-curses-gui cmake-qt-gui git subversion gfortran libcppunit-dev libqt5xmlpatterns5-dev libbluetooth-dev python3-wstool python3-catkin-tools python3-osrf-pycommon``

.. warning::
   for any dVRK software version greater than 2.1
   * Ubuntu 16.04 support has been dropped
   * Ubuntu 18.04 support requires clang instead of gcc.  You will need to install clang with ``sudo apt install clang`` and configure your workspace using: ``catkin config --cmake-args -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++``

Catkin workspace, clone and build
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you're using ROS Noetic and the master branch, you can just
copy/paste the following block of commands in a terminal.  For other
configurations, make sure your replace ``noetic`` by ``melodic`` or
whatever version of ROS you're using.  For the ``devel`` branches,
replace ``master`` by ``devel``.

.. code-block:: bash

   source /opt/ros/noetic/setup.bash # or use whatever version of ROS is installed!
   mkdir ~/catkin_ws                  # create the catkin workspace
   cd ~/catkin_ws                     # go in the workspace
   wstool init src                    # we're going to use wstool to pull all the code from github
   catkin init                        # create files for catkin build tool
   catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release # all code should be compiled in release mode
   cd src                             # go in source directory to pull code
   wstool merge https://raw.githubusercontent.com/jhu-dvrk/dvrk-ros/master/dvrk_ros.rosinstall # or replace master by devel
   wstool up                          # now wstool knows which repositories to pull, let's get the code
   catkin build --summary             # ... and finally compile everything

Environment variables
^^^^^^^^^^^^^^^^^^^^^

This is recommended if you're going to use a single catkin workspace
(as most users do):
https://github.com/jhu-cisst/cisst/wiki/Compiling-cisst-and-SAW-with-CMake#setting-up-your-environment-variables-for-ros

`catkin build` and git submodules
---------------------------------

This approach is a bit more complicated and will add some extra
repositories for SAW components not needed for the dVRK software.
Compilation time will be slightly longer.  Most repositories in
cisst/SAW will be cloned using git sub-modules.

cisstNetlib, cisst, SAW components and cisst-ros bridge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

You will first need to build cisst and its dependencies.  Follow the
instructions provided for [cisst/SAW catkin
build](https://github.com/jhu-cisst/cisst/wiki/Compiling-cisst-and-SAW-with-CMake#13-building-using-catkin-build-tools-for-ros)
and then come back to this page for the dVRK/ROS specific packages.

dvrk-ros
^^^^^^^^

These packages are not part of the cisst-saw repositories so you have
to clone them manually.  You first need to download the cisst-saw
libraries and components (see instructions above) and then do:

.. code-block:: bash

   cd ~/catkin_ws/src
   git clone https://github.com/jhu-dvrk/dvrk-ros
   git clone https://github.com/jhu-dvrk/dvrk-gravity-compensation
   git clone https://github.com/collaborative-robotics/crtk_msgs crtk/crtk_msgs
   git clone https://github.com/collaborative-robotics/crtk_python_client crtk/crtk_python_client
   git clone https://github.com/collaborative-robotics/crtk_matlab_client crtk/crtk_matlab_client
   catkin build --summary
