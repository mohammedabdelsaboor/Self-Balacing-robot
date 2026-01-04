# Self-Balancing Robot

This repository presents the design and implementation of a **two-wheeled self-balancing robot**, developed using multiple hardware configurations and software environments.  
The project focuses on comparing different **actuation methods** (stepper motors vs DC motors) and **control environments** (Arduino vs LabVIEW), while highlighting real-world limitations of inverted pendulum systems.

<p align="center">
  <img src="https://github.com/user-attachments/assets/0f9456c4-af40-45c2-a49a-6bac624c76b0" width="45%" />
  <img src="https://github.com/user-attachments/assets/b35528be-25dc-4952-9109-12a4ea6aa7e9" width="45%" />
</p>

<p align="center">
  <b>Left:</b> Arduino + Stepper Motors &nbsp;&nbsp; | &nbsp;&nbsp;
  <b>Right:</b> LabVIEW + DC Motors
</p>

---

## Objective

The primary objective of this project is to design and implement a **closed-loop control system** capable of maintaining the upright balance of a two-wheeled robot in real time.

This is achieved through:
- Continuous orientation feedback from an **MPU6050 Inertial Measurement Unit (IMU)**.
- On-board sensor fusion using the **MPU6050 Digital Motion Processor (DMP)**.
- A **PID-based feedback controller**.
- Actuation using either **stepper motors** or **DC motors**, depending on the implementation.

The project also investigates how **hardware limitations and mechanical design**, such as actuator dynamics and center of mass height, affect overall system stability.

---

## Project Implementations

### Arduino-Based Implementations

The Arduino platform was used to implement **two different motor configurations**, allowing a direct comparison between stepper motor control and DC motor control.

#### 1. Arduino + Stepper Motors

- Uses **NEMA 17 stepper motors** driven by **DRV8825** drivers.
- Entire control loop implemented in **C++ on the Arduino**.
- Orientation estimation handled by the **MPU6050 DMP**, providing filtered pitch angle data.
- Provides precise, deterministic motor behavior.
- Suitable for experiments requiring accurate low-speed motion and repeatable results.

---

#### 2. Arduino + DC Motors

- Uses **DC gear motors** driven by an **L298N motor driver**.
- Control algorithm implemented entirely on the Arduino using **C++**.
- Orientation data obtained from the **MPU6050 DMP** to reduce sensor noise and computational load.
- Demonstrates practical challenges such as actuator delay, friction, and limited torque.
- Highlights the effect of hardware constraints on balance performance.

---

### LabVIEW-Based Implementation (DC Motors Only)

- Uses **DC gear motors** driven by an **L298N motor driver**.
- High-level control, monitoring, and visualization implemented in **LabVIEW**.
- Communication between LabVIEW and Arduino is handled via **VISA serial communication**.
- The Arduino:
  - Reads IMU data from the **MPU6050**.
  - Uses the **DMP** to compute stable orientation estimates.
  - Streams processed angle data to LabVIEW.
- LabVIEW computes the control action and sends motor commands back to the Arduino.
- Emphasizes rapid prototyping, real-time visualization, and ease of controller tuning.

> **Note:**  
> Stepper motors were used **only** in the Arduino-based implementation.  
> The LabVIEW-based implementation was designed exclusively for **DC motor control**.

---

## Hardware Components

### Shared Components

| Component           | Description                                         |
|--------------------|-----------------------------------------------------|
| Arduino Uno         | Microcontroller for sensing and control             |
| MPU6050             | 6-axis IMU with on-board Digital Motion Processor   |
| Wheels and Chassis  | Mechanical structure for mobility and balance       |
| Li-ion Battery Pack | Power supply for motors and controller              |

---

### Stepper Motor Configuration (Arduino)

| Component                    | Description                                  |
|-----------------------------|----------------------------------------------|
| DRV8825 (×2)                | High-current stepper motor drivers            |
| NEMA 17 Stepper Motors (×2) | Bipolar stepper motors for precise actuation |

---

### DC Motor Configuration (Arduino & LabVIEW)

| Component           | Description                                |
|--------------------|--------------------------------------------|
| L298N Motor Driver  | Dual H-bridge driver for DC motors         |
| DC Gear Motors (×2) | Brushed DC motors for mobile actuation     |

---

## Software Architecture

### IMU Processing (MPU6050 DMP)

- The **MPU6050 Digital Motion Processor (DMP)** is used for real-time sensor fusion.
- Combines accelerometer and gyroscope data internally.
- Outputs a filtered and drift-reduced **pitch angle** used directly by the controller.
- Reduces computational load on the microcontroller compared to software-based filtering.

---

### Arduino-Based Control (C++)

- Developed using the **Arduino IDE**.
- Reads orientation data from the MPU6050 via the **DMP FIFO buffer**.
- Implements a **real-time PID controller** for balance stabilization.
- Motor control handled using:
  - **AccelStepper library** for stepper motors.
  - Direct PWM-based control for DC motors.
- Emphasizes deterministic timing and fast feedback loops.

---

### LabVIEW-Based Control (VISA)

- Developed using **LabVIEW** with **VISA serial communication**.
- LabVIEW is responsible for:
  - Control logic implementation.
  - Real-time data visualization (tilt angle, error, control effort).
  - Online parameter tuning and monitoring.
- Arduino functions as a **sensor and actuator interface**, streaming DMP-processed orientation data and receiving motor commands.
- Demonstrates clear separation between high-level control and low-level hardware interfacing.

---

## Key Observations

- Using the **MPU6050 DMP** significantly improves angle stability compared to raw sensor data.
- Stepper motors provide **high positional accuracy and deterministic behavior**, making controller tuning easier.
- DC motors introduce **nonlinearities, delay, and torque limitations**, which directly affect stability.
- Increasing the **center of mass height** significantly reduces stability margins.
- System performance is often limited by **hardware constraints rather than control algorithm design alone**.

---

## Conclusion

This project demonstrates that achieving stable balance in a self-balancing robot is a combined challenge involving:
- Sensor fusion,
- Control algorithm design,
- Actuator selection,
- Mechanical configuration,
- And software architecture.

The use of the **MPU6050 DMP** enables reliable orientation estimation, while the comparison between stepper and DC motor implementations provides practical insight into real-world control trade-offs and limitations.

---

## Future Work

- Upgrade motor drivers to reduce delay and increase torque bandwidth.
- Lower the center of mass to improve stability margins.
- Implement **state-space or LQR control**.
- Add observers or advanced sensor fusion techniques.
