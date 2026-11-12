<div align="center"> <img src="https://capsule-render.vercel.app/api?type=waving&color=0:0f2027,50:203a43,100:2c5364&height=200&section=header&text=3-Axis%20Camera%20Gimbal&fontSize=48&fontColor=ffffff&animation=fadeIn&fontAlignY=38&desc=STM32%20%C2%B7%20IMU%20%C2%B7%20Kalman%20Filter%20%C2%B7%20Servo%20Control&descAlignY=60&descSize=18" width="100%" alt="header"/> <a href="https://git.io/typing-svg"> <img src="https://readme-typing-svg.herokuapp.com?font=Fira+Code&weight=600&size=22&pause=1000&color=36BCF7&center=true&vCenter=true&width=700&lines=Keeping+the+camera+steady+while+the+drone+moves;Sensor+fusion+with+a+Kalman+filter+written+in+C;Real-time+servo+stabilization+on+STM32;Validated+offline+in+MATLAB" alt="Typing SVG" /> </a> <br/>

Show Image Show Image Show Image Show Image Show Image Show Image

</div>
📖 Table of Contents
Overview
Key Features
System Architecture
Repository Structure
Hardware
How It Works
STM32 Firmware
MATLAB Tools
Getting Started
Pin Mapping
Results
Roadmap
Author
🎯 Overview

This project is a 3-axis camera gimbal designed to be mounted on a quadcopter. Its job is simple to state and interesting to build:

No matter how the drone tilts, rolls or yaws, the camera keeps pointing in the same direction.

An IMU measures the attitude of the drone, a Kalman filter fuses the noisy gyroscope and accelerometer readings into a clean angle estimate, and servo motors counter-rotate the camera platform to cancel the motion.

The gimbal has two operating modes:

Mode	Description
🛰️ Stabilization	The camera orientation stays fixed in space while the drone's attitude changes.
🎯 Look-at (Pointing)	When the drone is stationary, a command rotates the camera to look at a specific direction.

The repository is split into two parts: the STM32 firmware that runs the gimbal in real time, and a set of MATLAB scripts used to log raw sensor data and design/validate the Kalman filter offline before porting it to C.

✨ Key Features
🧭 IMU-based attitude sensing over I2C (gyroscope + accelerometer)
🧮 Kalman filter implemented from scratch in C for real-time angle estimation
⚙️ Modular driver libraries — separate, reusable modules for the servo, the gyro/IMU and the Kalman filter
📈 MATLAB data acquisition & analysis — read live sensor data, plot it, and prototype the filter
🔁 Offline-to-online workflow — algorithm tuned in MATLAB first, then ported to embedded C
🧱 Clean separation of concerns — hardware drivers, estimation and control are independent
🏗️ System Architecture
I2C
PWM
physical motion
UART
🧭 IMUGyro + Accel
🧠 STM32MCU
🧮 Kalman FilterAngle Estimation
🎛️ Control LogicError = Target − Angle
⚙️ Servo MotorsRoll / Pitch / Yaw
📷 Camera Platform
💻 MATLABLogging & Analysis

The loop runs continuously: the IMU senses the motion, the filter estimates the true angle, the controller computes the correction, and the servos move the camera platform — which the IMU then measures again.

📂 Repository Structure
text
📦 gimbal-project
 ┣ 📂 MATLAB
 ┃ ┣ 📜 read_data.m          # Reads sensor data coming from the STM32
 ┃ ┗ 📜 kalman_filter.m      # Kalman filter prototype / offline validation
 ┣ 📂 STM32
 ┃ ┃  📜 servolib.c / servolib.h        # Servo (PWM) driver
 ┃ ┃  📜 mpu6050.c  / mpu6050.h         # IMU / gyroscope driver (I2C)
 ┃ ┃  📜 kalman_filter.c / kalman_filter.h      # Kalman filter in C
 ┃ ┗ 📜 main.c
 ┗ 📜 README.md

📝 File names above are illustrative — adjust them to match the actual files in the repository.

🔧 Hardware
Component	Purpose
STM32 microcontroller	Main processor — I2C, timers (PWM), UART
IMU module (MPU-series)	3-axis gyroscope + 3-axis accelerometer
Servo motors	Actuate the roll / pitch / yaw axes of the gimbal
External power supply	Servos draw significant current — power them separately from the MCU
Quadcopter	Target platform for the final system

Tools: Keil µVision (MDK), STM32CubeMX (HAL), MATLAB.

⚠️ Always share a common ground between the servo supply and the STM32, and never power servos directly from the MCU's 3.3 V pin.

🧠 How It Works
1️⃣ Sensing

The gyroscope gives the angular rate (smooth, but it drifts over time when integrated). The accelerometer gives the gravity direction (no drift, but noisy and disturbed by vibration). Neither is good enough alone.

2️⃣ Sensor fusion with a Kalman filter

The Kalman filter combines both sources. For each axis the state is the angle and the gyro bias:

$$x_k = \begin{bmatrix} \theta_k \\ b_k \end{bmatrix}$$

Predict — integrate the gyro rate:

$$\hat{\theta}_{k|k-1} = \hat{\theta}_{k-1} + \Delta t\,(\omega_k - \hat{b}_{k-1})$$
$$P_{k|k-1} = F P_{k-1} F^{T} + Q, \qquad F = \begin{bmatrix} 1 & -\Delta t \\ 0 & 1 \end{bmatrix}$$

Update — correct with the accelerometer angle $\theta_{acc}$:

$$
K_k = \frac{P_{k|k-1} H^{T}}{H P_{k|k-1} H^{T} + R}, \qquad
\hat{x}_k = \hat{x}_{k|k-1} + K_k\,(\theta_{acc} - H\hat{x}_{k|k-1})
$$

The result is an angle estimate that is as smooth as the gyro and as drift-free as the accelerometer.

3️⃣ Control

The filtered angle is compared with the desired camera angle. The error is converted into a servo PWM pulse (typically 1–2 ms), moving each axis in the opposite direction of the drone's motion.

4️⃣ Two-stage development
Raw data is streamed to MATLAB, where the filter is designed and its parameters (Q, R) are tuned on real recordings.
The tuned algorithm is ported to C and runs on the STM32 in real time.
💾 STM32 Firmware

The firmware is organized into independent libraries, each responsible for one part of the system.

<details> <summary><b>⚙️ Servo library</b> — click to expand</summary> <br/>
Generates PWM signals using hardware timers
Converts a target angle (°) to a pulse width
Provides simple functions to set the angle of each axis
Includes angle limiting to protect the mechanics
c
Servo_Init();
Servo_SetAngle(SERVO_PITCH, 90.0f);
</details> <details> <summary><b>🧭 Gyro / IMU library</b> — click to expand</summary> <br/>
Communicates with the IMU over I2C (HAL)
Verifies the device by reading its WHO_AM_I register
Reads raw gyroscope and accelerometer registers and converts them to physical units (°/s and g)
Handles basic offset calibration
c
IMU_Init();
IMU_ReadAll(&imu_data);
</details> <details> <summary><b>🧮 Kalman filter library</b> — click to expand</summary> <br/>
Lightweight, dependency-free C implementation of the filter described above
One filter instance per axis
Tunable noise parameters (process noise Q, measurement noise R)
c
Kalman_Init(&kf_pitch);
float pitch = Kalman_GetAngle(&kf_pitch, acc_angle, gyro_rate, dt);
</details>

📝 Replace the example function names with the actual API of your libraries.

📊 MATLAB Tools
Script	Description
read_data.m	Opens the serial port, reads the data sent by the STM32 and plots it live — useful for debugging the IMU and checking raw sensor behavior.
kalman_filter.m	Applies the Kalman filter to recorded data so that Q and R can be tuned and the filtered output compared against the raw measurements before running it on the MCU.

Typical workflow

matlab
% 1) Acquire data from the STM32
read_data

% 2) Test and tune the filter offline
kalman_filter

🔌 Pin Mapping
Signal	STM32 Pin	Notes
I2C SCL	PB6	IMU clock
I2C SDA	PB7	IMU data
Servo 1 PWM	PA0	e.g. Pitch
Servo 2 PWM	PA1	e.g. Roll
Servo 3 PWM	PA2	e.g. Yaw
UART TX	PA9	Data to MATLAB

📝 Update this table with the pins you actually used.

📈 Results

Add plots and photos here — they make a README stand out!

Raw vs. filtered angle	Gimbal in action
plot	demo
🗺️ Roadmap
 Read IMU data over I2C on STM32
 Kalman filter prototype in MATLAB
 Kalman filter ported to embedded C
 Modular servo / gyro / filter libraries
 Full 3-axis stabilization tuning (PID)
 Look-at / pointing command mode
 Magnetometer fusion for drift-free yaw
 Custom PCB for the gimbal controller (Altium)
 Flight tests on the quadcopter
👤 Author

Mahyar Electrical Engineering Student — Shahid Beheshti University

Interests: digital electronics, embedded systems, communications & systems

GitHub

<div align="center">

⭐ If you found this project useful, consider giving it a star!

<img src="https://capsule-render.vercel.app/api?type=waving&color=0:2c5364,100:0f2027&height=100&section=footer" width="100%" alt="footer"/> </div>