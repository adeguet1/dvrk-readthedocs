.. _Introduction:

************
Introduction
************

This documentation is not ready, use the `dVRK wiki
<https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki>`_ for now.

Accounts
========

You will need to create two accounts to access some privates resources:

* `Intuitive Surgical hardware wiki
  <http://research.intusurg.com/dvrk>`_: please contact ISI to get an
  account created for each active user.  This wiki contains some
  important documentation regarding the hardware, including unboxing
  instructions and wiring fixes.

* `Google group <https://groups.google.com/d/forum/research-kit-for-davinci>`_ and ``research-kit-for-davinci@googlegroups.com``: we strongly encourage users to send questions using this google group.

  * Go to https://groups.google.com/forum/#!forum/research-kit-for-davinci

  * Use the *Apply for membership* link to request membership, don't
    forget to mention your group/university so the group admin can
    identify you.

Documentation
=============

Make sure you read all the documentation, including:

* `Unboxing the arms
  <http://research.intusurg.com/dvrkwiki/index.php?title=DVRK:Docs:Main>`_
  (on the ISI maintained wiki)

* `da Vinci Classic User Manual
  <https://dvrk.lcsr.jhu.edu/downloads/manuals/davinci-classic-user-manual.pdf>`_,
  this is useful to understand how the real da Vinci should be setup

You can browse the dVRK documentation using the side menu.  The main steps are:

* :ref:`Compilation`: This has to be performed once per user, each
  user will likely maintain their own version of the software in their
  home directory.  The core software for the dVRK can be built without
  ROS but we strongly recommend you use ROS and the catkin build
  tools.

* [Controller Connectivity](/jhu-dvrk/sawIntuitiveResearchKit/wiki/ControllerConnection) - This has to be performed once per computer.  The goal is to make sure you have the proper hardware and OS configuration to communicate with the controllers over FireWire.

* [XML Configuration](/jhu-dvrk/sawIntuitiveResearchKit/wiki/XMLConfig) - This step has to be performed once per robotic arm.  Once the configuration has been generated, we strongly recommend to save the files in a safe place (e.g. github, feel free to contact Anton Deguet @ JHU for help)

* [Hardware Setup](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Hardware) - This section describes the few hardware modifications required  and physical connections between the controllers and the arms as well as between the controllers and the PC.

* [Calibration](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Calibration) - Once the controllers are physically connected, the software has been compiled and you have the base configuration files, you can proceed to the calibration to identify some values specific to each arm and controller pair.  This should be a one time step and we strongly recommend you save all your configurations files to share between the users in your group.

* [Examples](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Examples) - This section shows how to run the different examples provided with the dVRK.  The first 3 examples are used to test the system when you receive it.  Once your system is set up and calibrated, you will mostly use the ROS `dvrk_robot dvrk_console_json` application.
