# Introduction

You can find examples of configuration files in the "shared" directory:
https://github.com/jhu-dvrk/sawIntuitiveResearchKit/tree/master/share

**Pay close attention to units as we used different ones in different sections!**

# IO configuration files (XML)

Configuration files for `sawRobotIO1394` are used to configure the low-level IOs with the dVRK controller.  Some of the files are arm specific (potentiometer and electronics calibration) while some are shared accross systems (e.g. digital inputs that won't require calibration).  Shared configuration files for standard digital IOs such as foot pedals, head sensor, endoscope focus controller... can be found in `share/io`.  For all other files, see [sawRobotIO1394 XML](/jhu-dvrk/sawIntuitiveResearchKit/wiki/robotio-xml).

# Console configuration files (JSON)

Starting with the dVRK **version 2.0**, the documentation for JSON based configuration files is generated using JSON schemas.  Schemas can be found in the directory `share/schemas`.

Documentation for:
* [2.1](https://dvrk.lcsr.jhu.edu/documentation/schemas/v2.1/dvrk-console.html) - [2.0](https://dvrk.lcsr.jhu.edu/documentation/schemas/v2.0/dvrk-console.html)
* [1.x](/jhu-dvrk/sawIntuitiveResearchKit/wiki/console-json-1.x)

# Arms configuration files (JSON)

Starting with the dVRK **version 2.0**, the documentation for JSON based configuration files is generated using JSON schemas.  Schemas can be found in the directory `share/schemas`.

Documentation for:
* Arms:
  * ECM: [2.1](https://dvrk.lcsr.jhu.edu/documentation/schemas/v2.1/dvrk-ecm.html) - [2.0](https://dvrk.lcsr.jhu.edu/documentation/schemas/v2.0/dvrk-ecm.html)
  * MTM: [2.1](https://dvrk.lcsr.jhu.edu/documentation/schemas/v2.1/dvrk-mtm.html) - [2.0](https://dvrk.lcsr.jhu.edu/documentation/schemas/v2.0/dvrk-mtm.html)
  * PSM: [2.1](https://dvrk.lcsr.jhu.edu/documentation/schemas/v2.1/dvrk-psm.html) - [2.0](https://dvrk.lcsr.jhu.edu/documentation/schemas/v2.0/dvrk-psm.html)
* [1.x](/jhu-dvrk/sawIntuitiveResearchKit/wiki/kinematic-json-1.x)

# Instruments

The instrument's list, introduced in **version 2.1** describes all the instruments (tools) supported by the dVRK.  Users can also define their own list to support new or custom instruments.

Documentation for:
* Instrument's list: [2.1](https://dvrk.lcsr.jhu.edu/documentation/schemas/v2.1/dvrk-tool-list.html)