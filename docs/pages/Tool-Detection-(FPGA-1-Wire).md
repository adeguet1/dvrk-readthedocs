<!--ts-->
   * [Introduction](#introduction)
   * [Hardware](#hardware)
      * [Prerequisites](#prerequisites)
      * [dMIB Rev F or newer](#dmib-rev-f-or-newer)
      * [dMIB Rev A-E](#dmib-rev-a-e)
   * [Software](#software)

<!-- Added by: anton, at:  -->

<!--te-->

# Introduction

This page describes the (soon to be deprecated) **FPGA 1-wire interface** to the DS2505 chip inside the da Vinci instruments.
Reading this chip enables the software to automatically determine which tool is installed on the PSM.
The recommended approach is the **Dallas driver interface** described [here](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Tool-Detection).

**Note:** Currently, this is the only option supported by Firmware Version 7, but we recommend that you wait for Firmware Version 8 and implement the modification described [here](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Tool-Detection).
Also, the approach on this page requires QLA Version 1.4 or greater.

# Hardware

## Prerequisites

Reading the instrument info via the **FPGA 1-wire interface** requires:
* dMIB Rev F or newer, or a modified dMIB Rev A-E (see below)
* FPGA Firmware Verion 7 or newer
* QLA Version 1.4 or greater
* dVRK software 2.0 or higher

The dMIB and QLA versions are printed on the silkscreen on both boards. You can also check the version of QLA by querying the serial number.  To locate the dMIB and QLA, see [Controller Boxes](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Controller-Boxes).

Hardware batches/builds can be found for each system in the [list of dVRK sites](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Timeline). The board versions for each build can be found in the [FPGA](https://github.com/jhu-cisst/FPGA1394#release-notes) and [QLA](https://github.com/jhu-cisst/QLA#release-notes) release notes.

If your dVRK controller is shipped later than 2019, you do not need to modify. You should have QLA V1.4+ and dMIB Rev F+. The instrument info is supported out of the box (see following section).

Otherwise, you can modify your controller as described in the next section (dMIB Rev A-E).

## dMIB Rev F or newer

<a href="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/tool-detection/dallas-1wire-revf.jpg"><img src="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/tool-detection/dallas-1wire-revf.jpg" width="750"></a>

The above figure illustrates the tool interface using the FPGA 1-wire interface, rather than the Dallas driver chip (DS2480B), with yellow highlight on the active wires. The DOUT3 signal from QLA #2 (which becomes DOUTB6 on the dMIB) directly drives the bidirectional 1-wire interface to the DS2505 chip inside the da Vinci instrument.

To configure the dMIB to use the FPGA 1-wire interface, you need to jump the first 2 pins on J42, which is located between the two SCSI-68 cables on the dMIB (see picture below).

<a href="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/tool-detection/dmib-tool-jumper-12-FPGA.jpg"><img src="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/tool-detection/dmib-tool-jumper-12-FPGA.jpg" width="350"></a>

## dMIB Rev A-E

:warning: | Do not do this if you have a recently built controller (with dMIB Rev F or newer)
:---: | :---

<a href="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/tool-detection/dallas-1wire.jpg"><img src="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/tool-detection/dallas-1wire.jpg" width="750"></a>

The above figure illustrates the tool interface using the FPGA 1-wire interface on dMIB versions prior to Rev F, with yellow highlight on the active wires. The DOUT3 signal from QLA #2 (which becomes DOUTB6 on the dMIB) directly drives the bidirectional 1-wire interface to the DS2505 chip inside the da Vinci instrument.
This requires a jumper wire (shown in red above) to be soldered on the dMIB inside the controller box, as decribed below.

You need the PSM dVRK controllers, screwdrivers/nut drivers/hex wrenches, a piece of small insulated wire or magnet wire, and a soldering iron.

**Step 1.** Unplug power. Unplug cables from the dMIB/QLA so you can work on the back side of the 156-pin ITT Cannon connector (that mates with the robot arm connector) or take the dMIB out. Please make sure to label the cables as you unplug them.

**Step 2.** (optional) Remove dMIB from the PSM dVRK controller box. This step may be optional if you have small dexterous fingers and good soldering skills (or use the EndoWrist soldering iron).

**Step 3.** See the figure below. Solder a jumper wire between the `R1 pin` in the 156-pin connector and the resistor `R69` pad that is closest to the SCSI connector. *Some dMIB have misaligned silkscreen for the 156-pin connector, like the rev. D in the figure.* Do not remove the resistor R69. If you did so and cannot solder the original part back in, you can jump an approximately 1 kOhm resistor between the `R69` pad you connected the jumper wires to and the `T6 pin` of the 156-pin connector.

![](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/tool-detection/dmib-tool-info-mod.jpg)

**Step 4.** Reconnect the cables between QLA and dMIB. Connect the PSM and test the functionality. Reassemble the controller box.

# Software

See [this page](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Tool-Detection#software).
