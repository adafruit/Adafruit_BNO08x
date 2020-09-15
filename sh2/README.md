# SH2 Sensorhub driver for MCU application (No RTOS version)

The files in this repository provide application-level SH2 sensor hub functionality.

To use this code, an application developer will need to:
* Incorporate this code into a project.
* Provide platform-level functions, as specified in sh2_hal.h
* Develop application logic to call the functions in sh2.h

More complete instruction can be found in the User's Guide:
* [SH2 Library User's Guide](UserGuide.pdf)

An example project based on this driver can be found here:
* [sh2-demo-nucleo](https://github.com/hcrest/sh2-demo-nucleo)
