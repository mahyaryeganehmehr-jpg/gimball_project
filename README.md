# 🎥 STM32-Based Camera Gimbal Controller

<div align="center">

![Header](https://capsule-render.vercel.app/api?type=waving&color=0:0f2027,50:203a43,100:2c5364&height=200&section=header&text=Camera%20Gimbal&fontSize=52&fontColor=ffffff&animation=fadeIn&fontAlignY=38&desc=STM32%20%7C%20MPU6050%20%7C%20Kalman%20Filter%20%7C%20Servo%20Control&descAlignY=60&descSize=17)

[![Platform](https://img.shields.io/badge/Platform-STM32-blue?style=for-the-badge&logo=stmicroelectronics)](https://www.st.com/)
[![Language](https://img.shields.io/badge/Language-C-00599C?style=for-the-badge&logo=c)](https://en.wikipedia.org/wiki/C_(programming_language))
[![Sensor](https://img.shields.io/badge/IMU-MPU6050-orange?style=for-the-badge)](https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/)
[![MATLAB](https://img.shields.io/badge/Analysis-MATLAB-red?style=for-the-badge&logo=mathworks)](https://www.mathworks.com/)

**Real-time camera stabilization using embedded control systems and inertial sensing.**

</div>

---

## 📑 Table of Contents

- [Overview](#-overview)
- [Project Objectives](#-project-objectives)
- [System Architecture](#-system-architecture)
- [Hardware Components](#-hardware-components)
- [Software & Development Tools](#-software--development-tools)
- [How It Works](#-how-it-works)
- [STM32 Firmware](#-stm32-firmware)
- [MATLAB Development](#-matlab-development)
- [Hardware Connections](#-hardware-connections)
- [Getting Started](#-getting-started)
- [Development Roadmap](#-development-roadmap)
- [Author](#-author)

---

## 🎯 Overview

This project focuses on developing an embedded camera gimbal controller based on an **STM32 microcontroller** and an **MPU6050 inertial measurement unit (IMU)**.

The main objective is to compensate for unwanted rotational movements of a quadcopter and keep the camera orientation as stable as possible.

The system measures angular motion using the gyroscope, estimates the camera platform's orientation through sensor processing, and generates PWM signals to control servo motors.

The development follows a modular approach, separating sensor interfacing, signal processing, angle estimation, and motor control into independent software components.

The project is being developed in multiple stages, starting with a single-axis stabilization prototype and progressing toward a fully functional three-axis gimbal.

## 🚀 Project Objectives

The main engineering goals are:

- Develop a reusable MPU6050 driver using STM32 HAL and I2C.
- Acquire and process real-time inertial sensor data.
- Implement a Kalman filter in C for embedded angle estimation.
- Develop modular PWM-based servo control libraries.
- Design a feedback control system for camera stabilization.
- Use MATLAB for sensor visualization, data acquisition, and algorithm validation.
- Build a scalable firmware architecture for multi-axis stabilization.

## 🏗️ System Architecture

The system follows a modular signal-processing and control architecture.

```text
       Physical Motion
       (Drone Rotation)
              |
              v
       +--------------+
       |   MPU6050    |
       | Gyro + Accel |
       +--------------+
              |
           I2C Bus
              |
              v
       +--------------+
       |    STM32     |
       | Microcontroller
       +--------------+
              |
              v
       +--------------+
       | Kalman Filter|
       | Angle Estimation
       +--------------+
              |
              v
       +--------------+
       | Control Loop |
       | PID / Control|
       +--------------+
              |
              v
       +--------------+
       | Servo Driver |
       | PWM Generation
       +--------------+
              |
              v
       +--------------+
       | Camera Mount |
       +--------------+

       UART <----> MATLAB
              |
       Data Logging
       Visualization
       Filter Analysis
```

### Main Software Modules

| Module | Responsibility |
|---|---|
| MPU6050 Driver | Sensor communication and raw data acquisition |
| Kalman Filter | Sensor processing and angle estimation |
| Servo Driver | PWM generation and servo positioning |
| Control System | Stabilization and correction calculations |
| MATLAB Tools | Data acquisition, visualization and algorithm testing |

The architecture is designed to keep hardware drivers independent from estimation and control algorithms.

---

## 🔧 Hardware Components

| Component | Description |
|---|---|
| STM32F103C8T6 | Main microcontroller (Blue Pill) |
| MPU6050 | 6-axis gyroscope and accelerometer |
| MG90S | Micro servo motor for mechanical stabilization |
| External Power Supply | Dedicated servo power source |
| Quadcopter | Target platform |

### Development Hardware

- STM32 Blue Pill development board
- MPU6050 IMU module
- MG90S servo motor
- USB-to-TTL converter
- ST-Link programmer

**Important:** Servo motors should be powered from an appropriate external supply. The servo supply and STM32 must share a common ground.

---

## 💻 Software & Development Tools

| Tool | Application |
|---|---|
| STM32CubeMX | Peripheral configuration and initialization |
| Keil µVision | Embedded C development and debugging |
| STM32 HAL | Hardware abstraction layer |
| MATLAB | Sensor data analysis and filter development |
| Git & GitHub | Version control and project documentation |

---

## 🧠 How It Works

The stabilization process consists of three main stages.

### 1. Inertial Sensing

The MPU6050 provides two types of measurements:

**Gyroscope**
- Measures angular velocity.
- Provides fast and responsive motion measurements.
- Angular integration introduces accumulated drift over time.

**Accelerometer**
- Measures linear acceleration, including gravity.
- Can estimate tilt angles relative to gravity.
- Is sensitive to vibration and external acceleration.

Combining these measurements allows more reliable tilt estimation than using either sensor independently.

### 2. Kalman Filter – Angle Estimation

A Kalman filter is implemented to estimate the system angle by combining gyroscope and accelerometer measurements.

The state vector is defined as:

$$
x_k =
\begin{bmatrix}
\theta_k \\
b_k
\end{bmatrix}
$$

Where:

- $\theta$: Estimated angle.
- $b$: Gyroscope bias.

**Prediction**

The gyroscope angular velocity is integrated to predict the next angle:

$$
\hat{\theta}_{k|k-1} =
\hat{\theta}_{k-1}
+
\Delta t(\omega_k-\hat{b}_{k-1})
$$

The covariance prediction is:

$$
P_{k|k-1}=FP_{k-1}F^T+Q
$$

Where:

$$
F=
\begin{bmatrix}
1 & -\Delta t\\
0 & 1
\end{bmatrix}
$$

**Update**

The accelerometer provides an independent angle measurement:

$$
K_k=
\frac{P_{k|k-1}H^T}
{HP_{k|k-1}H^T+R}
$$

The estimated state is corrected using:

$$
\hat{x}_k=
\hat{x}_{k|k-1}
+
K_k(\theta_{acc}-H\hat{x}_{k|k-1})
$$

This approach combines the responsiveness of the gyroscope with the long-term reference provided by gravity-based accelerometer measurements.

> **Note:** Accelerometer-based correction works for roll and pitch, but not absolute yaw. A magnetometer or another heading reference is required for long-term yaw stabilization.

### 3. Servo Control

The estimated angular position is compared with the desired camera orientation.

$$
e(t)=\theta_{target}-\theta_{estimated}
$$

The control algorithm calculates the required correction, which is converted into a PWM signal for the servo motor.

The mechanical system then rotates in the opposite direction to compensate for the drone's movement.

The control architecture is designed to support PID-based stabilization.

---

## ⚙️ STM32 Firmware

The firmware is organized into independent C modules to improve readability, maintainability, and reusability.

### Repository Structure

```text
Gimbal-Project/
│
├── MATLAB/
│   ├── read_data.m
│   ├── kalman_filter.m
│   └── data/
│
├── STM32/
│   ├── Core/
│   │   ├── Inc/
│   │   └── Src/
│   │
│   ├── Drivers/
│   │   ├── MPU6050/
│   │   │   ├── mpu6050.c
│   │   │   └── mpu6050.h
│   │   │
│   │   ├── Kalman/
│   │   │   ├── kalman_filter.c
│   │   │   └── kalman_filter.h
│   │   │
│   │   └── Servo/
│   │       ├── servolib.c
│   │       └── servolib.h
│   │
│   └── Gimbal.ioc
│
├── Documentation/
│
└── README.md
```

### 🧭 MPU6050 Driver

Responsible for:

- Initializing the MPU6050 through I2C.
- Checking the sensor using the WHO_AM_I register.
- Reading raw gyroscope and accelerometer data.
- Converting raw measurements into physical units.
- Performing initial sensor bias calibration.

Example usage:

```c
MPU6050_Init();

MPU6050_ReadAll(&imu_data);
```

### 🧮 Kalman Filter Library

A lightweight C implementation of the Kalman filter.

Features:

- Independent filter instances.
- Gyroscope bias estimation.
- Configurable process and measurement noise.
- Real-time angle estimation.

Example usage:

```c
Kalman_Init(&kf);

angle = Kalman_GetAngle(
    &kf,
    acc_angle,
    gyro_rate,
    dt
);
```

### ⚙️ Servo Control Library

Responsible for generating PWM signals and controlling servo positioning.

Features:

- Hardware timer-based PWM generation.
- Angle-to-pulse conversion.
- Adjustable servo operating limits.
- Reusable functions for different gimbal axes.

Example usage:

```c
Servo_Init();

Servo_SetAngle(90.0f);
```

*Function names are illustrative and may differ from the actual implementation.*

---

## 📊 MATLAB Development

MATLAB is used as a development and validation environment before deploying signal-processing algorithms to the STM32.

### Available Scripts

| File | Description |
|---|---|
| `read_data.m` | Reads real-time sensor data from STM32 through UART and visualizes measurements. |
| `kalman_filter.m` | Implements and evaluates the Kalman filter using recorded sensor data. |

### Development Workflow

1. Acquire raw sensor measurements from STM32.
2. Transfer data to MATLAB through UART.
3. Visualize gyroscope measurements and analyze sensor noise.
4. Implement and tune the Kalman filter.
5. Compare raw and filtered measurements.
6. Port the validated algorithm to embedded C.
7. Evaluate the real-time behavior on STM32.

This workflow makes it possible to experiment with filter parameters before deploying the algorithm on resource-constrained hardware.

---

## 🔌 Hardware Connections

The following table shows an example peripheral configuration. Update it according to the actual CubeMX configuration.

| Signal | STM32 Pin | Peripheral |
|---|---|---|
| MPU6050 SCL | PB6 | I2C1_SCL |
| MPU6050 SDA | PB7 | I2C1_SDA |
| Servo PWM | PA0 | TIM2_CH1 |
| UART TX | PA9 | USART1_TX |
| UART RX | PA10 | USART1_RX |

**Power considerations:**
- MPU6050: 3.3V supply (according to the module specifications).
- Servo: External regulated supply suitable for its voltage and current requirements.
- Common GND between STM32, IMU and servo power supply.

---

## 🛠️ Getting Started

### Prerequisites

- STM32CubeMX
- Keil µVision
- STM32 HAL libraries
- MATLAB (for data acquisition and analysis)
- STM32F103C8T6 development board

### Setup

1. Clone the repository:

```bash
git clone https://github.com/YOUR_USERNAME/Gimbal-Project.git
```

2. Open the STM32 project in Keil µVision.

3. Configure the required peripherals using STM32CubeMX:
   - I2C for MPU6050 communication.
   - TIM for servo PWM generation.
   - UART for serial communication.

4. Build and flash the firmware to the STM32.

5. Connect the IMU and servo according to the hardware configuration.

6. Run the MATLAB scripts to acquire and analyze sensor data.

---

## 🗺️ Development Roadmap

The project is being developed incrementally.

- [x] STM32 development environment setup.
- [x] MPU6050 communication over I2C.
- [x] Raw gyroscope data acquisition.
- [x] UART communication with MATLAB.
- [x] Sensor data visualization.
- [x] Kalman filter implementation and testing in MATLAB.
- [x] Basic servo PWM control.
- [x] Modular firmware development.
- [ ] Embedded Kalman filter integration and validation.
- [ ] PID controller implementation and tuning.
- [ ] Single-axis camera stabilization.
- [ ] Multi-axis attitude estimation and stabilization.
- [ ] Magnetometer integration for absolute heading estimation.
- [ ] Look-at / pointing control mode.
- [ ] Custom PCB design using Altium Designer.
- [ ] Integration with the quadcopter.
- [ ] Real-world flight testing.

---

## 👨‍💻 Author

**Mahyar**

Electrical Engineering Student  
Shahid Beheshti University

**Technical Interests**
- Embedded Systems
- Digital Electronics
- Microcontrollers
- Signal Processing
- Control Systems
- Robotics

[GitHub Profile](https://github.com/YOUR_USERNAME)

---

<div align="center">

### ⭐ Thanks for visiting!

If you find this project interesting, consider giving it a star.

![Footer](https://capsule-render.vercel.app/api?type=waving&color=0:2c5364,100:0f2027&height=110&section=footer)

</div>