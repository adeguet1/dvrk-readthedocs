# Introduction

* dESSJ custom boards to replace the ESSJ in the SUJ itself.  The boards are pass-through for the LVDS communication between the arm and the controller (FireWire B physical connector, not a real FireWire port).  For the SUJ's analog potentiometers, we use an Arduino with either USB or BLE (Bluetooth Low Energy) to communicate with the PC.
* dSIB adapter boards for each dVRK Si controller.

## Connectivity

You first need to make sure you have a Bluetooth adapter on your PC and said adapter is turned on.  You can use GUI tools for this or the command line tool `hciconfig`.  To test the dESSJ itself, you can use the `bluetoothctl` command line tool.

### `hciconfig`
* `hciconfig` list all adapters.  In the examples below, `hci0` identifies the Bluetooth adapter found
* `sudo hciconfig hci0 down` to stop Bluetooth
* `sudo hciconfig hci0 up` to start Bluetooth

### `bluetoothctl` command line examples
* `bluetoothctl list` to figure out if the computer has a Bluetooth adapter
* `bluetoothctl devices` to list devices found.  For SUJ,  `bluetoothctl devices| grep -i suj`.  Then note the MAC Address (Media Access Control Address) for the Arduino board on the dESSJ
* `bluetoothctl info 68:7E:57:32:FD:63` to get info.  UUID info is defined in ino (Arduino program)
* `bluetoothctl connect 68:7E:57:32:FD:63`

### `bluetoothctl` interactive shell
Start `bluetoothctl`.  When using `bluetoothctl` shell, note that autocomplete works very well
* `devices` to list available BT devices and find UUID
* `connect 68:7E:57:32:FD:63`, the MAC address will differ (but autocomplete helps)
* `info 68:7E:57:32:FD:63 `, not required but will provide more data re. the Arduino BLE
* `menu gatt`
* `list-attributes` to see all attributes
* `select-attribute /org/bluez/hci0/dev_68_7E_57_32_FD_63/service000a/char000b`, the service name will differ based on the MAC address
* `read` should return the binary result as well as the human readable string.  You can repeat this command multiple times or you can use `notify on` and `notify off` to get new values automatically
* `back` to get back to previous menu
* `disconnect 68:7E:57:32:FD:63` to disconnect

### Firmware

See https://github.com/jhu-dvrk/dESSJ-firmware
