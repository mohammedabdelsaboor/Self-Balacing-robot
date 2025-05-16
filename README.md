# Self-Balancing Robot

This repository presents the implementation of a two-wheeled self-balancing robot utilizing two distinct development environments:

- **LabVIEW** with the LINX Toolkit for graphical programming and hardware interfacing.
- **Arduino IDE** using C++ for low-level embedded control.

---

## Objective

The primary objective of this project is to design and implement a real-time control system capable of maintaining the vertical balance of a two-wheeled mobile robot. This is achieved through continuous feedback from an Inertial Measurement Unit (MPU6050) and a closed-loop PID control algorithm.

---
![IMG_0265](https://github.com/user-attachments/assets/0f9456c4-af40-45c2-a49a-6bac624c76b0)

## Hardware Components

### Shared Components

| Component             | Description                                          |
|----------------------|------------------------------------------------------|
| Arduino Uno           | Microcontroller unit for control and interfacing    |
| MPU6050               | 6-axis IMU combining a gyroscope and accelerometer  |
| Wheels and Chassis    | Mechanical structure for mobility and balance       |
| Li-ion Battery Pack   | Power supply for motors and controller              |

### Arduino Implementation

| Component                    | Description                                       |
|-----------------------------|---------------------------------------------------|
| DRV8825 Stepper Drivers (x2)| High-performance stepper motor drivers            |
| NEMA 17 Stepper Motors (x2) | Bipolar stepper motors for precise actuation      |

### LabVIEW Implementation

| Component             | Description                                 |
|----------------------|---------------------------------------------|
| L298N Motor Driver    | Dual H-Bridge driver for brushed DC motors |
| DC Gear Motors (x2)   | DC motors used for basic mobility           |

> **Note:**  
> The Arduino-based version uses stepper motors for finer control, while the LabVIEW-based version is designed for use with standard DC motors.

---

## Software Architecture

### Arduino Version

- Developed using the Arduino IDE (C++).
- Utilizes the MPU6050 for real-time tilt angle estimation.
- Implements a PID controller to maintain balance.
- Motor control is achieved using the AccelStepper library in conjunction with DRV8825 drivers.

### LabVIEW Version

- Developed in LabVIEW using the LINX Toolkit.
- Implements control logic and data visualization within the LabVIEW environment.
- Communicates with the Arduino for sensor acquisition and motor control.

---
