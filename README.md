# BLDC-BalanceBot

## Overview

**BLDC-BalanceBot** is a project for a two-wheeled self-balancing robot powered by brushless DC (BLDC) motors. This repository is built as an abstraction and modification of the BLDC motor drivers originally developed in [BLDCMotorDriver](https://github.com/zackvega1/BLDCMotorDriver) by Zack Vega, Clayton Elwell, and Ted Ivanac.

The project leverages a custom-designed MBOT, created and fabricated by Zack Vega and Andrew Rodriguez. By modifying the default SimpleFOC motor drivers, an integrated IMU, and two BLDC motors, this system effectively maintains balance and control velocity through a double-nested PID control loop.

## Features

- **Custom MBOT Design**: A two-wheeled robot using brushless DC motors.
- **IMU Integration**: Real-time balance control using IMU data fed into the main controller loop.
- **PID Control**:
  - Loop for balance stabilization.
  - Loop for velocity control.
- **Modified SimpleFOC Drivers**: Modified BLDC motor drivers to fit the dual motor requirement of the BalanceBot.

## How It Works
1. **IMU and Motor Calibration**: When the robot is powered on the FOC Motor Control and IMU are calibrated ideally on a flat surface.
1. **IMU Data Processing**: The IMU sends real-time angle and acceleration data to the Raspberry Pi Pico.
2. **Control Loop**:
   - The data is processed in the main loop of the controller.
   - The system calculates the required adjustments using two PID loops:
     - **Angle PID Loop**: Maintains the robot’s balance.
     - **Velocity PID Loop**: Controls the velocity of the robot.
3. **Motor Control**: The processed control signals are sent to the motor drivers, which operate the BLDC motors to achieve stable and responsive motion.

## Setup and Installation

### Prerequisites
- **Hardware**:
  - Two-Wheeled MBOT.
  - Raspberry Pi Pico.
  - 2 Motor Supporting BLDC Control Board
  - 2 BLDC Motors.
  - 2 Motor Encoders that support I2C Communication
  - IMU module (We used the BHI 160)
- **Software**:
  - [SimpleFOC Library](https://github.com/simplefoc/Arduino-FOC).
  - Raspberry Pi Pico SDK or Arduino IDE.
 
### Modifications From Base Repository
1. IMU Data Handling and Processing Addition
2. CAN -> I2C Motor Encoder Communication
3. Dual Motor Support
4. Modified BLDC Motor Class to support Angle and Velocity PID
5. Modified FOC Motor class to take in IMU Data and support Angle/Velocity PID
6. Modified PID Class to handle Angle and Velocity PID.
