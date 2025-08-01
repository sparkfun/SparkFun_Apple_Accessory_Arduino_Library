# SparkFun Apple Accessory Arduino Library

An Arduino Library for Apple Accessories using iAP2 connections - precompiled for ESP32

This library supports communication with Apple Accessories using the iAP2 protocol, as defined in the Accessory Interface Specification (Release R44). Unfortunately, the Interface Specification is only available under NDA. This library is precompiled for the ESP32 to protect the source code.

iAP2 is supported over several transports, including Bluetooth, UART and USB. The [ESP32_BluetoothSerial](./examples/ESP32_BluetoothSerial/ESP32_BluetoothSerial.ino) example demonstrates how to communicate with an Accessory using ESP32 BluetoothSerial as the transport. The example depends on a modified version of BluetoothSerial which makes ACL more accessible. It also depends on a modified version of libbt.a which has been compiled with SPP enabled.

## Repository Contents

* **/examples** - Example code 
* **/src** - Precompiled source code for ESP32

## License Information

Please review the LICENSE.md file for license information. 

If you have any questions or concerns on licensing, please contact technical support on our [SparkFun forums](https://community.sparkfun.com/c/development-boards/esp-system-on-a-chip-soc/92).

Distributed as-is; no warranty is given.

- Your friends at SparkFun.
