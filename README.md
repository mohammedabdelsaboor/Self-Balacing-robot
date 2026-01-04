# Self-Balancing Robot

This repository presents the development and comparison of a **two-wheeled self-balancing robot** implemented using two different control and development approaches:

- **Arduino (C++)** for low-level real-time embedded control.
- **LabVIEW** using **VISA serial communication** for high-level control, monitoring, and visualization.

The project investigates how different hardware choices and software architectures affect the performance and stability of an inverted pendulum–based mobile robot.

---

## Objective

The main objective of this project is to design and implement a **closed-loop control system** capable of maintaining the upright balance of a two-wheeled robot.

This is achieved through:
- Continuous orientation feedback from an **MPU6050 Inertial Measurement Unit (IMU)**.
- A **PID-based feedback controller** running in real time.
- Actuation using either **stepper motors** or **DC motors**, depending on the implementation.

The project also highlights the **practical limitations imposed by actuator dynamics and hardware constraints**, especially in systems with a high center of mass.

---

## Project Versions

### Version 1 – Stepper Motor–Based Balancing Robot  
*(Arduino Implementation)*

![IMG_0265](https://github.com/user-attachments/assets/0f9456c4-af40-45c2-a49a-6bac624c76b0)

- Uses **NEMA 17 stepper motors** driven by **DRV8825** drivers.
- Provides precise and repeatable motor control.
- Entire control loop implemented in **C++ on the Arduino**.
- Suitable for experiments requiring accurate low-speed motion and deterministic behavior.

---

### Version 2 – DC Motor–Based Balancing Robot  
*(LabVIEW + Arduino Implementation)*

![image](https://github.com/user-attachments/assets/b35528be-25dc-4952-9109-12a4ea6aa7e9)

- Uses **DC gear motors** driven by an **L298N motor driver**.
- High-level control and visualization implemented in **LabVIEW**.
- Communication between LabVIEW and Arduino is handled using **VISA serial communication**.
- Emphasizes rapid prototyping, live monitoring, and ease of parameter tuning.

---

## Hardware Components

### Shared Components

| Component           | Description                                         |
|--------------------|-----------------------------------------------------|
| Arduino Uno         | Main microcontroller for sensing and control        |
| MPU6050             | 6-axis IMU (accelerometer + gyroscope)              |
| Wheels and Chassis  | Mechanical structure for mobility and balance       |
| Li-ion Battery Pack | Power supply for motors and controller              |

---

### Arduino (Stepper Motor) Implementation

| Component                    | Description                                  |
|-----------------------------|----------------------------------------------|
| DRV8825 (×2)                | High-current stepper motor drivers            |
| NEMA 17 Stepper Motors (×2) | Bipolar stepper motors for precise actuation |

---

### LabVIEW (DC Motor) Implementation

| Component           | Description                                |
|--------------------|--------------------------------------------|
| L298N Motor Driver  | Dual H-bridge driver for DC motors         |
| DC Gear Motors (×2) | Brushed DC motors for mobile actuation     |

---

## Software Architecture

### Arduino-Based Control (C++)

- Developed using the **Arduino IDE**.
- Direct acquisition of IMU data from the **MPU6050**.
- Implements a **real-time PID controller** for balance stabilization.
- Motor control handled using:
  - **AccelStepper library** for stepper motors.
  - Direct PWM control for DC motors (where applicable).
- Emphasizes low-level timing accuracy and fast feedback loops.

---

### LabVIEW-Based Control (VISA)

- Developed using **LabVIEW** with **VISA serial communication**.
- LabVIEW is responsible for:
  - Control logic implementation.
  - Real-time data visualization (tilt angle, error, control effort).
  - Online parameter tuning and monitoring.
- Arduino acts as a **hardware interface**, streaming IMU data and receiving motor commands via serial communication.
- Demonstrates separation between high-level control and low-level hardware interfacing.

---

## Key Observations

- The **stepper motor version** provides higher positional accuracy and predictable behavior.
- The **DC motor version** exposes practical challenges such as actuator delay, friction, and limited torque.
- Increasing the **center of mass height** significantly reduces stability margins, even with well-tuned controllers.
- System performance is strongly influenced by **hardware limitations**, not only by the control algorithm.

---

## Conclusion

This project provides a practical study of self-balancing robots, demonstrating the interaction between:
- Control algorithms,
- Actuator selection,
- Mechanical design,
- And software architecture.

It highlights that achieving stable balance is not solely a control problem, but a combined **control–mechanical–hardware** challenge.

The project forms a solid foundation for further exploration into advanced techniques such as **state-space control, LQR, and observer-based systems**.

---

## Future Improvements

- Upgrade motor drivers to reduce delay and increase torque bandwidth.
- Lower the center of mass to improve stability margins.
- Implement full **state-space or LQR control**.
- Add observers or sensor fusion for improved state estimation.
