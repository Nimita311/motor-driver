# Motor Drivers on STM32

## Introduction
Welcome to ENGN2920F Sensors and Actuators for Real Systems at [Brown University][brown]. This repository hosts the reference design of the following labs:

- BLDC/DC motor driver with PID control,
- Two-axis stepper motor driver for 2D plotter,
- Multi-axis stepper motor driver for 3D printing.

This project mainly serves teaching purposes. The source code is well commented and is intended to be read and studied. Application code is located in [`App/`][app] where your reading should begin. All dependencies are included in the form of source code for your easy reference.

Build instructions and makefiles are intentionally excluded. For experienced users, it should be straightforward to configure STM32Cube or any of your favorite toolchains to build the project.

## Hardware Target
The target board is NUCLEO-H743ZI made by STMicroelectronics. It uses a 144-pin [STM32H743ZIT6][stm32h743] microprocessor with an [Arm Cortex-M7][cortexm7] core. Hardware dependent part of this project is implemented with STM32CubeH7 development package and STM32CubeMX code generation tool.

## Source Structure
Directory | Description
--- | ---
[`App/`][app] | Application on STM32.
[`App/app_lib`][lib] | Application library for functional logics, e.g. PID controller and FIR filter. (Hardware independent)
[`App/app_serv`][serv] | Application services with runable tasks, e.g. BLDC driver and UART communication. (Hardware dependent)
[`App/app_msg`][msg] | Message descriptions in Protocol Buffers.
[`Core/`][core] | Main, startup, and interupt service routines (ISR).
[`Utils/`][util] | Utilities on desktop, e.g. monitors and consoles.
Others | Dependencies for your reference.

## Documentations
Frequently asked questions in office hours are addressed here. [Email me][hzz] to get your questions answered here.

- [Nested functions in C and C++](Docs/nested-functions.md)

## Credits
This project is made possible by the following open source projects.
- [FreeRTOS](https://www.freertos.org/)
- [Protocol Buffers](https://developers.google.com/protocol-buffers)
- [Nanobp](https://jpa.kapsi.fi/nanopb/)

## Copyright
ENGN2920F Sensor and Actuators for Real Systems is designed and taught by [Prof. Harvey Silverman][hfs] at Brown University School of Engineering.

This repository is maintained by Brown graduate student [Haoze Zhang][hzz]. Source code in [`App/`][app] and [`Utils/`][util] is distributed under MIT license.

[brown]: https://www.brown.edu
[hfs]: mailto:hfs@lems.brown.edu
[hzz]: mailto:haoze_zhang@brown.edu
[stm32h743]: https://www.st.com/en/microcontrollers-microprocessors/stm32h743zi.html
[cortexm7]: https://www.arm.com/products/silicon-ip-cpu/cortex-m/cortex-m7

[app]: App/
[lib]: App/app_lib
[serv]: App/app_serv
[msg]: App/app_msg
[core]: Core/
[util]: Utils/
