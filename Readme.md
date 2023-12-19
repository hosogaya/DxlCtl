# DxlCtl
Driver of dynamixel motors with teensy

# Dependencies
* Dynamixel SDK: https://github.com/hideakitai/Dynamixel 
* TeensyThreads

# Reference
* [Dynamixel XH-540-W270](https://emanual.robotis.com/docs/en/dxl/x/xh540-w270/)
* [Dynamixel control table 2.0 for X-series](https://www.besttechnology.co.jp/modules/knowledge/?X%20Series%20Control%20table)
* [Dynamixel control table 2.0 for MX-series](https://www.besttechnology.co.jp/modules/knowledge/?MX%20Series%20Control%20table%282.0%29)

# ToDo
* Implement thread processing
* Mutex for serial communication. see [port_handler](./include//Dynamixel-0.2.0/Dynamixel/lib/DynamixelSDK/include/port_handler_arduino.h)