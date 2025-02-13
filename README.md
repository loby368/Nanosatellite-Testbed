# Nano-Satellite Project
## Project Overview
This project was conducted as part of my master's Thesis in Aerospace Engineering at KTH Royal Institute of Technology, Stockholm. 

The project aimed to develop a NanoSatellite testbed on which Simulink control algorithms could be tested in a zero-g-like environment.
The hardware consists of a 1U CubeSat with a 1-axis reaction wheel mounted upon an air bearing to provide frictionless rotation.

The project involved designing the system architecture, developing the embedded system avionics units, building the mechanical chassis, and running control algorithm tests.

## Files
There are two directories in this github;
1) 'Sensor Drivers' contains the bare driver code that is later used on in the main.c superloop to communicate to the peripherals. This code was written based on the sensor datasheets.
2) 'STM32Cube Projects' contains the project file for the STMCubeIDE environment where it can be built and deployed onto the target.
The \Torque_Control\Src\main.c file contains the superloop which initiates the code, implements the sensor drivers, and calls the simulink API to execute the contorl algorithm.

## Video
You can watch a video of a force feedback control algorithm running on the testbed [here.](https://youtu.be/84-hkJHZbc4)

## Learnings:
From this project, I have improved my skills with:
- C
- I2C, SPI, CAN protocols
- STM32
- STMCUBEMX
- STMCUBEIDE
- HAL libraries
- Device drivers and datasheets
- Simulink Embedded Coder
- PID and Wave-Based control algorithms

## Final Report
The write-up can be found on the [KTH DIVA Web Portal](https://kth.diva-portal.org/smash/record.jsf?pid=diva2%3A1871652&dswid=-184)

![Screenshot 2025-02-03 215528](https://github.com/user-attachments/assets/7a9347cc-95cc-4c39-adc7-127bdff41616)

### An example of the telemetry that the testbed generates during a control experiment
![image](https://github.com/user-attachments/assets/569ac2e3-defb-411d-a349-8d157c78d817)
