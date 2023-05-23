# Introduction

The dVRK software stack is meant to be as open as possible so it can be integrated in an end-user application.  Once you've configured your subset of dVRK arms to use in your console.json, the dVRK console class will automatically setup some Qt widgets for visual debugging as well as some ROS topics, services and tf2 transforms.  This is described in the [Software Architecture](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Software-Architecture) page.  The API is described in the [dVRK API 2.x](/jhu-dvrk/sawIntuitiveResearchKit/wiki/API-2.x).  The dVRK API uses the CRTK naming convention as much as possible so you should familiarized yourself with [CRTK](https://github.com/collaborative-robotics/documentation/wiki/Robot-API).

<a href="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/component-options/dVRK-component-standard.png"><img src="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/component-options/dVRK-component-standard.png" width="450"></a>

# Using ROS (1 or 2, sockets, OpenIGTLink)

All the middleware solutions described in this section are not specific to the dVRK, i.e. they can be applied to all [cisst/SAW components](https://github.com/jhu-cisst/cisst/wiki/cisst-libraries-and-SAW-components).

## ROS 1

The simplest way to write an application for the dVRK is to use ROS.  The dVRK ROS node `dvrk_robot dvrk_console_json` exposes most of the dVRK features as topics.

<a href="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/component-options/dVRK-component-ROS-teleop.png"><img src="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/component-options/dVRK-component-ROS-teleop.png" width="650"></a>

Pros:
* There is no need to understand the internal dVRK components nor learn anything about the [cisst](https://github.com/jhu-cisst/cisst/wiki) libraries
* For your ROS node, you can use any programming language ROS supports, C++, Python, Java...
* If you're using either Python or Matlab, we also provide some libraries to simplify your code:
  * [CRTK Python client](https://github.com/collaborative-robotics/crtk_python_client)
  * [CRTK Matlab client](https://github.com/collaborative-robotics/crtk_matlab_client)
* Your application will be a different process.  It can even run on a different computer so the computing load will not impact the dVRK console.  This might actually be a required if your application is very CPU or IO intensive.
* Since you're using ROS, you can `rosbag` everything for further data analysis.  You can also use tools such as RViz and PlotJuggler to debug your application.

Cons:
* Since ROS is a middleware, there is a performance cost due to the serialization, sending and de-serialization of your messages.  The cost is somewhat relative, specially with modern computers.  In our experience, this drawback is not prohibitive for most applications:
  * You can still close a control loop at 500Hz or more if your client is written in C++ or Python.  Matlab might not be able to sustain frequencies that high.  You can use `rostopic hz` to monitor the frequency at which topics are published.
  * There is also some added latency but mostly likely under a millisecond for most messages.  We paid special attention to use ROS messages with a header so you also can rely on the `timestamp` to figure out when the data was generated.

## Notes

* By default the dVRK console publishes both synchronous (events) and asynchronous data (state data).  Events (such as `operating_state` are published as fast as possible.  State data (such as `measured_js`...) is published periodically.  By default the dVRK console publishes data at 100Hz (10ms).  This can be increased using the `-p` command line argument.  By default the dVRK arm components are running at 1.5KHz so it doesn't make sense to publish at any rate higher than 1.5KHz.
* We provide a full fledge dVRK client API for both Python (`import dvrk`) and Matlab (`dvrk.`).  These are very convenient for quick testing and sending commands from an interactive interpreter but they come at a cost.  To provide all the possible features, these dVRK clients have to subscribe to all the dVRK topics and this will definitely slow down your interpreter.  This is specially true for the Matlab client.  You can look at the `dvrk_bag_replay.py` example in [dvrk-ros/dvrk-python](https://github.com/jhu-dvrk/dvrk-ros/blob/devel/dvrk_python/scripts/dvrk_bag_replay.py) to see how to use the `crtk.utils` to configure your client to use only the topics you need.

## ROS 2, sockets (JSON), OpenIGTLink

ROS 1 has been added to the dVRK many years ago and is still the middleware we recommend (as of 2022).  We also support ROS 2, UDP socket with JSON messages and OpenIGTLink.

### ROS 2

The support for ROS 2 has been added to the dVRK but not extensively used (i.e. not heavily tested either).  The core features replicate the ROS 1 dVRK bridge but there are a couple of missing tools:
* Matlab client library
* MTM gravity calibration scripts (using Matlab)
* Potentiometer calibration scripts

At this point, we strongly recommend you run ROS 1 in parallel to perform all the calibrations steps.  Once you have all your configuration files generated, you can use them with ROS 2.  If you use Ubuntu 20.04, you can have ROS 1 (noetic) and ROS 2 (galactic) installed on the same computer.

To compile the dVRK stack for ROS 2, follow the instructions from the [dVRK Wiki](/jhu-dvrk/sawIntuitiveResearchKit/wiki/BuildROS2).

### Sockets with JSON

One of the drawbacks of ROS 1 is that it is hard to install on Windows or MacOS.  If your application can't easily run on Ubuntu, for example Unity for HoloLens, you can use the [_sawSocketStreamer_](https://github.com/jhu-saw/sawSocketStreamer).  This cisst/SAW component can be dynamically loaded and configured to connect to any cisst/SAW component to:
* Get data from the dVRK
* Send commands to the dVRK

All the messages are serialized in JSON.  This way you can use any existing JSON parser to serialized/de-serialize the messages.  These messages are also somewhat human readable so it makes debugging easier.  The main drawbacks of the text based serialization are the computing coast and lost of accuracy for floating point numbers.  We found that in most cases, the ease of integration outweighs these drawbacks.

To use the _sawSocketStreamer_, clone the repository in your ROS workspace, under `src/`.  Note that the _sawSocketStreamer_ can run along ROS 1 and ROS 2.  You can then build it using `catkin build` (ROS 1), `colcon build` (ROS 2) or with CMake/make.

Once you have the _sawSocketStreamer_ compiled, you will need at least two files.  The first file is used to tell the dVRK console which components should be dynamically created (by the `cisstMultiTask` component manager).  The command line parameter to indicate which component manager configuration files need to be used is `-m`.  You can use multiple `-m` options.   The second file is a configuration file for the `sawSocketManager` component itself.  It is used to specify which command to bridge (using the name and payload type).

You can find some examples of configuration files, usage and a simple python client for the dVRK in [sawIntuitiveResearchKit/share/socket-streamer](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/tree/devel/share/socket-streamer).

### OpenIGTLink

We also implemented an [OpenIGTLink](https://github.com/openigtlink/OpenIGTLink) bridge for cisst/SAW components: [_sawOpenIGTLink_](https://github.com/jhu-saw/sawOpenIGTLink).  The main application for IGTL is [Slicer3D](https://www.slicer.org).  If you use ROS 2, you might also consider the [SlicerROS2 module](https://github.com/rosmed/slicer_ros2_module).

The main difference between _sawSocketStreamer_ and _sawOpenIGTLink_ is that we use the OpenIGTLink sockets and serialization instead of UDP with JSON.  You will need to use OpenIGTLink (C++ or Python) on the end-user application side to receive/send messages.

You can find some examples of configuration files for the dVRK in [sawIntuitiveResearchKit/share/igtl](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/tree/devel/share/igtl).

# Using cisst/SAW components

Instead of using a middleware and implement the end-user application's logic in a separate process, you can also add cisst/SAW components, either existing ones or your own, to the dVRK console.  This is actually the mechanism we use for the ROS (1 and 2), _sawSocketStreamer_ and _sawOpenIGTLink_ described above.  The main advantage of this approach is performance, i.e. the communication between components doesn't require any serialization/de-serialization nor sockets.  _cisstMultiTask_ also provides non-blocking and thread-safe communication mechanisms between threads so you can take advantage of modern CPUs with multi-cores.

## Derived components

Since the dVRK stack relies on _cisstMultiTask_ components (see [tutorial](https://github.com/jhu-cisst/cisst/wiki/cisstMultiTask-concepts)), one can technically replace any existing component by their own.  It's possible but not necessarily easy nor the best approach.  If your changes are modest, it might be easier to start from the existing component and alter it.  The best way to do so is to derive from the default dVRK class.  The main advantages of derived classes are:
* All the interfaces (provided and required) the console expects are already defined so the console can connect the existing ROS bridge (1 or 2), Qt widget, PID, IO, etc.
* All the existing configuration parameters will still be there so you can re-use them.
* The code related the component creation is already defined in the base class so you'll have less code to manage.

As of 2023, the dVRK console supports derived classes for the arm and the PSM teleoperation.  You can find documented examples for:
* Derived `mtsTeleOperationPSM` in [examples/derivedTeleOperationPSM](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/tree/devel/).  This example hows a single derived C++ class.
* Derived `mtsIntuitiveResearchKitPSM` in [examples/derivedPSMQtROS](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/tree/devel/).  This example shows a derived C++ class as well as a custom Qt Widget and ROS bridge (ROS 1) so one can communicate with the derived class with custom messages.

## Generic components

### Alternate hardware

The dVRK console also supports generic arms, i.e. one can use a different type of hardware as long as there is a cisst/SAW component for it and the component has a _provided interface_ that matches the dVRK arm it is meant to replace.  We've successfully integrated some alternate for the MTM:
* Sensible Phantom Omni using [sawSensablePhantom](https://github.com/jhu-saw/sawSensablePhantom): The Omni doesn't provide a gripper so when used with the default dVRK teleoperation, the jaws are ignored.  The two buttons on the stylus can be used to emulated the dVRK for pedals for "operator present" and "clutch".  
* ForceDimension haptic devices and Novint Falcon using [sawForceDimensionSDK](https://github.com/jhu-saw/sawForceDimensionSDK): The ForceDimension devices offer different features based on the model.  We only tested models with 7 degrees of freedom, i.e. position, orientation and gripper.  If the orientation is motorized, it can be used like a da Vinci MTM and we can enforce that the MTM orientation matches the PSM orientation.  ForceDimension devices don't have buttons so we either have to has a USB foot pedal or used the GUI for "operator present" and "clutch".  The Novint Falcon is not as useful for real applications since it doesn't have a wrist but can be used for simple demos and debugging.  See example of configuration file in [https://github.com/dvrk-config/dvrk_config_jhu](https://github.com/dvrk-config/dvrk_config_jhu/blob/main/jhu-dVRK-Si-demo/console-Novint-Falcon-PSM1-Teleop.json).

<a href="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/component-options/dVRK-component-ForceDimension.png"><img src="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/component-options/dVRK-component-ForceDimension.png" width="450"></a>

### ROS component

The ROS component _dvrk_arm_from_ros_ can be treated as an example of alternate hardware.  When used, the console uses ROS to communicate with a generic arm (e.g. PSM or substitute).  This can be used if you need your tele operation to work across the network.  The black diagram shows the "PSM over ROS" coming from an actual dVRK but could be any other device as long as the ROS topics are the same.

<a href="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/component-options/dVRK-component-PSM-from-ROS.png"><img src="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/component-options/dVRK-component-PSM-from-ROS.png" width="450"></a>

Note: this can be a bit hard to debug since mismatch in topic names are not reported as opposed to dynamically loaded components.  You will likely need to use `rostopic list` and `rostopic info` to find all the existing topics and check which nodes subscribe and publish to them.

You can find an example of dVRK console configuration file in [https://github.com/dvrk-config/dvrk_config_jhu](https://github.com/dvrk-config/dvrk_config_jhu/blob/main/jhu-dVRK/console-MTML-PSM1_ROS-Teleop.json).