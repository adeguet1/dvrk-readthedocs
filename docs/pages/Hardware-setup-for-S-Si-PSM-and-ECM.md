# 🐥 Beta hardware and software. Subject to change.

This guide applies to the PSM and ECM from a da Vinci S or Si. The label underneath the robot arm should read `PSM2` or `ECM3`. Currently, the older `PSM1` and `ECM2` are not supported.

# Mount the arm

Use the Intuitive-provided mounting hardware to hang the robot on your frame.

If you have a setup joint, see [Hardware setup for S/Si SUJ]().

# Re-program the arm

The circuit board (ESPM) inside the arm reads the encoders, magnetic potentiometers, buttons, and the instrument memory and sends them to the rest of the system. JHU, with help from Intuitive, developed a closed-source FPGA firmware that speaks an open protocol. The firmware is publicly available in a binary image. The arm programmer ("brain parasite") re-programs the arm at powers up. The alternative firmware is not persistent and the arm will revert to the original firmware after a power cycle if the brain parasite is removed.

Remove the power to the arm. Turn off the controller if connected.

Remove the plastic cover on the robot arm. You need an imperial allen wrench. Wiggle the cover to pull out the retaining mechanism.

![A screwdriver removing a screw on a plastic cover](https://user-images.githubusercontent.com/5226846/174355931-190018de-89c5-4373-949b-1e6732bf0dbe.jpg)

Connect the brain parasite pigtail cable to **J22 JTAG** on ESPM. There are multiple identical connectors on the board. Make sure you positively identify the connector by the label before you plug the cable in. The connector is keyed and please do not force it upside down. Do not unplug the cable from the brain parasite end. The cable has identical connectors on both ends but plugging it in backward will not work. If you are confused, look closely at the picture to see which pins are populated.

![A cable runs between the robot and the brain parasite. The connector on the robot is highlighted.](https://user-images.githubusercontent.com/5226846/174386903-a53ddc78-43a8-46ae-83db-61379bc47f78.jpg)

Put the brain parasite in the pouch, then stick the pouch on the robot as shown. Make sure that the switch on the brain parasite is set to `infect`, and that the micro SD is present. Put the plastic cover back on the arm. Make sure the cover is not pinching the cable.

![A brain parasite in an adhesive pouch on a robot arm](https://user-images.githubusercontent.com/5226846/174387381-494be4f4-f687-401d-ba88-8e82a54c4478.jpg)

# Connect the cables

Power off the controller.

Plug the two female D-sub cable ends into the robot. Make sure they are fully seated and stress relieved, as there is no locking mechanism. 

![Robot connectors](https://user-images.githubusercontent.com/5226846/174387729-1806fa8c-9b02-4239-a040-162fbe6c8a20.jpg)

Plug the other ends of the cables into the controller. 

![Contoller connectors](https://user-images.githubusercontent.com/5226846/174387738-bea9210d-fed1-4800-996a-04b33f0536ba.jpg)

The E-stop wiring is the same as classic controllers. See [E-stop wiring](/jhu-dvrk/sawIntuitiveResearchKit/wiki/ESTOP). The pinout is the same, but the physical connector is rotated 180 degrees.

![E-stop pinout](https://user-images.githubusercontent.com/5226846/174389563-03340609-ec9d-48ff-9fd1-0be26f9e3440.png)

Connect the firewire or ethernet to your computer.

# Power on

Turn on the power switch on the controller.

In a couple seconds, the arm should light up a purple LED that indicates the alternative firmware. If not, the brain parasite should report an error with yellow light blinks. Count the number of blinks and refer to the [brain parasite error messages]().

![arm with purple LED](https://user-images.githubusercontent.com/5226846/174390164-3c73cd29-b3b8-4fdb-bb0f-23e637744936.jpg)

The controller should light a pulsating FPGA LED indicating the FPGA is programmed. If the arm is programmed, you should also see a purple ESPM light, indicating that the arm is talking to the controller.

- fpga: pulsating light, 
- 48v: green=on. red=off, safety chain open. off=off, safety chain closed. It is normal that the safety chain is open when not using the robot because the relays are only closed when you enable power in the software.
- amp: red=any amp error. green=any amp on, no error. off=all amps off.
- espm: purple=espm communication active. off=no communication.
- fw/eth: inoperative

 
![IMG_2068](https://user-images.githubusercontent.com/5226846/174390484-9cc763c8-d410-48ec-83b4-30d7fa777d42.jpg)

# Calibrate the joints

TBD

# Run the software

TBD