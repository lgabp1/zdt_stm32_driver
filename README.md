# ZDT stm32 driver
An interface to drive the Emm_V5.0 Closed-Loop Stepper Drive by 张大头 (Zhang Da Tou, ZDT) through CAN using an STM32-based controller.

The code is fairly similar to [ZDT python-can driver](https://github.com/lgabp1/zdt_pythoncan_driver).

## Requirements
This library was made for the STM32CubeIDE environment, and designed for the [DM-MC02 Board](https://gitee.com/kit-miao/dm-mc02) based on [STM32h723vgT6](https://www.st.com/en/microcontrollers-microprocessors/stm32h723vg.html) MCU. With a bit of tweaking, the library should be compatible with any STM32-based board supporting Classical CAN.

## Repository content
This repository provides several resources:
* [`STM32CubeIDE_project`](./STM32CubeIDE_project/) contains an example STM32CubeIDE project
* [`driver_files`](./driver_files/) contains the driver's source code
A README.md file can be found in each of these folders to provide additionnal information.

## Features
* Checksums which are not Checksums.CS0x6B checksums are not implemented yet.
* Issue with ZDT_cmd_M_M_synchronized_motion where broadcasting the "Go" message does not work.
* Full implementation of all functions described in [Emm_V5.0 Manual Rev1.3]((https://blog.csdn.net/zhangdatou666/article/details/132644047)), and support for multiple such devices at once.

## Known issues
* Docstring should be improved.
* A better way to check if CAN handler is already initialized should be implemented.
* As for now, only support FD-CAN-compatible devices (because of types).

## Contributing
Beside the issues cited above, it would be great to add support for more ZDT stepmotor controllers and their commands. I expect most of the commands to be mostly similar between the different controllers, so it should be fairly easy to add support for them but I have only access to the Emm_V5.0 controller.

I would be happy to receive any contributions to this project. Please also feel free to fork and use the code as you see fit.

## License

Not yet decided as this repository shares STM32CubeIDE projects.

## Acknowledgements
This library is heavily based on [Emm_V5.0 Manual Rev1.3]((https://blog.csdn.net/zhangdatou666/article/details/132644047)), a very well documented manual.

## Version history
* `v1.00` Initial release
* `v1.03` Added new queue system to support several ZDT devices at once
