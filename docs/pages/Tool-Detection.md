<!--ts-->
   * [Introduction](#introduction)
   * [Hardware](#hardware)
      * [Prerequisites](#prerequisites)
      * [dMIB Rev F or newer](#dmib-rev-f-or-newer)
      * [dMIB Rev A-E](#dmib-rev-a-e)
   * [Software](#software)
      * [Testing](#testing)
      * [dVRK console configuration](#dvrk-console-configuration)

<!-- Added by: anton, at:  -->

<!--te-->

# Introduction

The da Vinci instruments can be automatically identified when inserted in the sterile adapter using the `add-only` chip embedded in the tool (aka Dallas chip, DS2505).  This feature was not supported in early versions of the dVRK both from a software and hardware perspective.

To retrieve the instrument tool type, the dVRK hardware can use two different approaches:
* **Dallas driver interface (DS2480B):**  The FPGA communicates serially with the DS2480B chip, which then communicates with the DS2505 chip in the tool using a 1-wire interface.  The DS2480B chip is either on the dMIB (Rev F or greater) or on a dongle that is attached to a connector on the rear of the controller (See requirements below).
* **FPGA 1-wire interface:**  In this scenario, the FPGA/QLA communicates directly with the tool's DS2505 chip.  This method requires QLA Version 1.4 or greater.

**Note:** This page describes the **Dallas driver interface**, which requires FPGA Firmware Version 8, because that is expected to become the preferred interface. We expect that it will be more robust than the FPGA 1-wire interface and it will also work with all versions of controller hardware. The (soon to be deprecated) **FPGA 1-wire interface**, which requires QLA V1.4+ and FPGA Firmware Version 7+, is described
[here](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Tool-Detection-(FPGA-1-Wire)).

# Hardware

## Prerequisites

Reading the instrument info via the **Dallas driver interface** requires:
* dMIB Rev F or newer, or a modified dMIB Rev A-E (see below)
* FPGA Firmware Version 8 or newer
* dVRK software 2.0 or higher

The dMIB versions are printed on the silkscreen on both boards. To locate the dMIB, see [Controller Boxes](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Controller-Boxes).

Hardware batches/builds can be found for each system in the [list of dVRK sites](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Timeline).

If your dVRK controller is shipped later than 2019, you do not need to modify. You should have dMIB Rev F or greater and the instrument info is supported out of the box (see following section).

Otherwise, you can modify your controller as described in the next section (dMIB Rev A-E).

## dMIB Rev F or newer

<a href="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/tool-detection/dallas-driver-dmib.jpg"><img src="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/tool-detection/dallas-driver-dmib.jpg" width="750"></a>

The above figure illustrates the tool interface using the Dallas driver chip (DS2480B) that is located on dMIB Rev F or greater, with yellow highlight on the active wires. The POSLIM3 and DOUT3 signals from QLA #2 (which become POSLIM7 and DOUTB6 on the dMIB) provide a serial interface to the DS2480B, which then drives the bidirectional 1-wire interface to the DS2505 chip inside the da Vinci instrument.

Note that it is necessary to install a jumper  on J42, which is located between the two SCSI-68 cables on the internal face of the dMIB (see image below). You do not need to remove the dMIB from the controller to access the jumper. To use the dMIB Dallas driver, it is necessary to jump pins 2 and 3 (right-most two pins).

<a href="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/tool-detection/dmib-tool-jumper-empty.jpg"><img src="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/tool-detection/dmib-tool-jumper-empty.jpg" width="350"></a>

## dMIB Rev A-E

:warning: | Do not do this if you have a recently built controller (with dMIB Rev F or newer)
:---: | :---

<a href="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/tool-detection/dallas-driver-dongle.jpg"><img src="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/tool-detection/dallas-driver-dongle.jpg" width="750"></a>

The above figure illustrates the tool interface using the Dallas driver chip (DS2480B) that is located on a dongle connected to the DOF 7 HD15 connector on the rear of the controller (J20 on the dMIB schematic), with yellow highlight on the active wires. The POSLIM3 and DOUT3 signals from QLA #2 (which become POSLIM7 and DOUTB6 on the dMIB) provide a serial interface to the DS2480B, which then drives the bidirectional 1-wire interface. The path for the 1-wire signal is somewhat convoluted -- it is first connected to the HOME7 signal (assuming the jumper on the dongle is installed). The HOME7 signal is also present on the footpedal connector and the jumper plug on the footpedal connector routes this signal to pin 9, which is normally not used. A jumper wire, soldered on the dMIB (see below), connects footpedal pin 9 to the 1-wire signal on the DL-156 connector. The actual dongle set and setup on a physical controller box, as an example, are shown in the following figures.

<a> <img src="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/tool-detection/dongle-sets.jpg" width="400">
<img src="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/tool-detection/dongle-sets-on-box.jpg" width="400"></a>

While it would have been possible to solder the jumper wire directly from the DL-156 connector to HOME7 (and eliminate the jumper plug on the footpedal connector), the downside of that approach is that the modification would interfere with operation of the footpedal connector, even if the DS2480B dongle is removed.

This option requires the following modifications:
1. Solder a jumper wire on the dMIB inside the controller box (see below)
2. Install a dongle on the DOF 7 (HD15) connector on the rear of the controller box (see figure above)
3. Install a jumper plug on the footpedal connector (DB15) on the rear of the controller box (see figure above)

You need the PSM dVRK controllers, screwdrivers/nut drivers/hex wrenches, a piece of small insulated wire or magnet wire, and a soldering iron.

**Step 1.** Unplug power. Unplug cables from the dMIB/QLA so you can work on the back side of the 156-pin ITT Cannon connector (that mates with the robot arm connector) or take the dMIB out. Please make sure to label the cables as you unplug them.

**Step 2.** (optional) Remove dMIB from the PSM dVRK controller box. This step may be optional if you have small dexterous fingers and good soldering skills (or use the EndoWrist soldering iron).

**Step 3.** See the figure below. Solder a jumper wire between the `R1 pin` in the 156-pin connector (J3) and the top left pin, aka pin 9, in the 'foot pedal' DB-9 connector (J24). *Some dMIB have misaligned silkscreen for the 156-pin connector, like the rev. D in the figure.*

<a href="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/tool-detection/dmib-modification.jpg"><img src="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/tool-detection/dmib-modification.jpg" width="750"></a>

**Step 4.** Reconnect the cables between QLA and dMIB. Connect the PSM and test the functionality. Reassemble the controller box.

# Software

## Testing

You can test the hardware and firmware configuration using the command line tool `instrument` provided with the low level software (along `qladisp` in `AmpIO`).  The `instrument` program will dump the memory from the instrument in a text file.  Make sure you have an instrument properly seated in the sterile adapter before launching the program.
```
instrument [-pP] <board num>
```

## dVRK console configuration

This requires the dVRK stack rev 2.0 or higher.  In your PSM configuration file, you can set the tool detection to be manual, automatic or fixed:
```json
{
    "kinematic": "kinematic/psm.json",
    "tool-detection": "AUTOMATIC"
    // "tool-detection": "MANUAL"
    // "tool-detection": "FIXED",
    // "tool": "LARGE_NEEDLE_DRIVER:400006"
}
```
The different options for `tool-detection` are:
* `AUTOMATIC`: this will rely on the Dallas chip query
* `MANUAL`: when a tool is inserted, the user or application has to specify which tool to use.  This can be done using the Arm GUI with a drop down menu or using a programmatic interface (e.g. ROS topic).
* `FIXED`: fixed type of tool, i.e. there is only one type of tool used.  To change tool, you will need to stop the program, change the configuration file and restart the program.  The configuration file must then define the tool type using `tool`.  Tool definitions can be found in `share/tool`.  If the revision number is needed, it can be specified using `[]` (for example: `LARGE_NEEDLE_DRIVER:420006[12]`).
