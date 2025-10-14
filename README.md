# FRC 2025 Tank Drive Robot

This repository contains the control software for the 2025 FRC robot, which features a tank drive system, an elevator mechanism, and an intake subsystem.  
The robot is programmed in Java using WPILib and REV Spark MAX motor controllers.

## Overview
The robot uses a dual-joystick tank drive for teleoperated control and encoder-based autonomous movement.  
The elevator uses a closed-loop PID control system with limit switches at the top and bottom positions, while the intake system is manually operated through controller inputs.

## Key Features
- Tank drive with independent left and right motor control  
- Autonomous routine using encoder-based position tracking  
- Elevator system with closed-loop control and positional setpoints  
- Intake mechanism with forward and reverse operation  
- Safety logic including joystick deadband and limit switch protection  

## Technologies Used
- Java (WPILib)
- REV Spark MAX and Spark Flex motor controllers
- Closed-loop PID control
- Encoder feedback and digital limit switches

## File Structure
- `WorkingCode.java`: Main robot class implementing autonomous, teleoperated, and initialization phases using the TimedRobot framework

---

Developed for the 2025 FRC season as part of the team's robot control software.
