# Motor Drivers on STM32

## Introduction
Welcome to _ENGN2920F Sensors and Actuators for Real Systems_ at [Brown University][brown]. This repository hosts the reference design of the following labs:

- BLDC/DC motor driver with PID control,
- Two-axis stepper motor driver for 2D plotter,
- Multi-axis stepper motor driver for 3D printing.

## Target Hardware
The target hardware is NUCLEO-H743ZI evaluation board made by STMicroelectronics. It uses a 144-pin [STM32H743ZIT6][stm32h743] microprocessor with an [Arm Cortex-M7][cortexm7] core. This project uses STM32CubeH7 development package and STM32CubeMX code generation tool.

## Purpose
This project serves teaching purposes. The source code is well commented and is intended to be read and studied. Application code is located in the `App/` directory, which should be the starting point of your study, while all dependencies are included in the form of source code for your reference.

Build instructions and makefiles are intentionally excluded. But for experienced users, it should be straightforward to configure the STM32Cube or any of your favorite toolchains to build the project.

## Copyright
The _ENGN2920F Sensor and Actuators for Real Systems_ course is designed and taught by [Prof. Harvey Silverman][hfs] at [Brown University][brown] in 2020.

The reference design is made by Brown graduate student [Haoze Zhang][hzz] in 2020 and is distributed under MIT license.

[brown]: https://www.brown.edu
[hfs]: mailto:hfs@lems.brown.edu
[hzz]: mailto:haoze_zhang@brown.edu
[stm32h743]: https://www.st.com/en/microcontrollers-microprocessors/stm32h743zi.html
[cortexm7]: https://www.arm.com/products/silicon-ip-cpu/cortex-m/cortex-m7
