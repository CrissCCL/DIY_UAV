# 🚁 UAV with PID Control ⚠️ *Work in Progress*

![UAV](https://img.shields.io/badge/UAV-DIY%20Platform-blue)
![Embedded](https://img.shields.io/badge/Embedded-Control%20Stack-green)
![Sensors](https://img.shields.io/badge/Sensors-IMU%2FGNSS-orange)
![Hardware](https://img.shields.io/badge/Hardware-PCB%20Modules-lightgrey)
![License](https://img.shields.io/badge/License-MIT-lightgrey)


## 📖 Overview
This DIY UAV (Unmanned Aerial Vehicle) project is being developed with an automatic control loop using PID controllers.  
The system controls angular position (Pitch and Roll) and angular velocity (Pitch, Roll and Yaw) to achieve stable and precise flight.  

During development, the signal-to-noise ratio (S/N) of measurements was improved, and the stability of the electronic circuitry was enhanced. Brushless motors are used for propulsion, and a Teensy microcontroller handles the control loop in real time. Feedback is provided by gyroscopes and accelerometers, enabling precise stabilization and responsiveness to disturbances.

> ⚠️ **Note:** This project is currently under development. Features and hardware are subject to change.

## 📂 Contents
- 

## 🧩 System Architecture

<p align="center">
  <img src="https://github.com/user-attachments/assets/e5505aba-81e0-4fd2-8924-9d0b97acb467" width="500">
</p>
<p align="center">
  <sub>
  Modular hardware stack: Flight Control Stage (top) → Power Stage → UAV frame integration
  </sub>
</p>

## 🧩 Hardware Modules

The UAV electronics follow a **modular architecture**, where control and power subsystems are implemented on **independent PCBs**.  
This separation improves reliability, reduces electrical noise coupling, simplifies maintenance, and allows faster hardware iteration.

| Module | Description |
|------------|---------------------------------------------|
| 🛩️ **Flight Control Stage** | Teensy-based control PCB with dual-IMU redundancy, real-time control and PWM actuation |
| ⚡ **Power Stage** | Power distribution and regulation for ESCs and control electronics |

### Repositories

- 🛩️ Flight Control Stage → https://github.com/CrissCCL/UAV_Flight_Control_Stage
- ⚡ Power Stage → https://github.com/CrissCCL/UAV_PowerStage  

## 📊 Project Status
| Component                  | Status                     |
|-----------------------------|----------------------------|
| Brushless Motors Setup      | ✅ Completed               |
| Teensy PID Control Loop     | ✅ Completed              |
| Model Development          |   ✅ Completed                 |
| Sensor Signal improvement   |  ✅ Completed                 |
| Dual IMU testing    | ✅ Completed                 |
| Pitch, Roll & Yaw Angular Velocity Control| ⚙️ In Progress          |
| Pitch & Roll Stabilization  | ⚙️ In Progress             |
| PCB Design with Improved structure and connections  | ⚙️ In Progress             |
| Prototype Flight Testing    | ⚠️ Not Started             |



## 🌐 YouTube
📺 [Test 1](https://youtube.com/shorts/LwX8zSV23eY?feature=share)

📺 [Test 2](https://youtube.com/shorts/dZo7ZcapqBg?feature=share)

📺 [Test 3](https://youtube.com/shorts/Z043N4uVOiI?feature=share)

📺 [Upgrade Power Stage](https://youtube.com/shorts/vSKApaUKDLE?feature=share)


## ⚙️ System Description

- **Controller:** Teensy 4.0 microcontroller (high-speed MCU for improved stability and processing)   
- **Sensors:** 3-axis accelerometer and gyroscope (IMU module) - Dual sensors are testing
- **Actuators:** Brushless motors with ESCs  
- **Control Strategy:** PID control for Pitch, Roll (angular position) and Yaw (angular velocity)  
- **Estimation filter:** 1D Kalman filter applied to angular measurements (per-axis)  
- **Sampling period:** **0.005 seconds (200 Hz)**  
- **Data transmission:** UART link to a **Raspberry Pi 4B** for telemetry and logging  
- **Data logging:** Raspberry Pi stores telemetry streams to `.csv` files for model validation and analysis  
- **Visualization:** Offline monitoring and plotting via Matlab


## 🔄 Control Hardware Update (In Progress)

As part of the ongoing system upgrade, the control stage is being redesigned to include a **dual-IMU architecture**, integrating:

- **BMI088** – robust IMU with excellent vibration tolerance
- **ICM-4205** – high-bandwidth IMU for improved dynamic response

The use of two different IMUs aims to:
- Improve sensor redundancy
- Compare noise characteristics and dynamic behavior
- Enhance state estimation robustness under high vibration conditions

This update is currently under development and will be validated through bench testing and flight experiments.




## 🔄 Control Loops
<p align="center">
<img width="500" alt="UAV Control Loop" src="https://github.com/user-attachments/assets/9d342de3-f207-44a3-a338-7d241f2a026d" />
</p>

### Controlled Variables
- **Pitch & Roll** → Angular position control (stabilization)
- **Pitch, Roll & Yaw** → Angular velocity control (rotation rate)

## 📐 Digital PID Control

The PID controllers implemented in this project are **incremental (velocity form)** and use **trapezoidal integration** for the integral term.  
This ensures:
- Accurate discrete-time implementation suitable for microcontrollers.
- Consistency between simulation and embedded hardware behavior.
- Avoidance of integral windup when combined with actuator saturation or anti-windup mechanisms.

The Module uses a discrete PID and P controller implemented on a Teensy microcontroller.  

### P Controller for UAV

#### Positional Form (Original)
The digital P controller in the outer loop was originally implemented for **Roll** and **Pitch** angles:

$$
error_{posRoll}(n) = Ref_{Roll}(n) - Angle_{Roll}(n)
$$

$$
error_{posPitch}(n) = Ref_{Pitch}(n) - Angle_{Pitch}(n)
$$

Control actions:

$$
error_{rateRoll}(n) = K_{Roll} \cdot error_{posRoll}(n)
$$

$$
error_{ratePitch}(n) = K_{Pitch} \cdot error_{posPitch}(n)
$$

#### Incremental Form (Current)
The UAV control has been updated from a **positional P controller** to an **incremental (velocity form) P controller**.  

Digital P controller in outer loop is implemented for Roll and Pitch angles:

$$
error_{posRoll}(n) = Ref_{Roll}(n) - Angle_{Roll}(n)
$$

$$
error_{posPitch}(n) = Ref_{Pitch}(n) - Angle_{Pitch}(n)
$$

Incremental control actions:

$$
error_{rateRoll}(n) = error_{rateRoll}(n-1) + K_{Roll} \cdot (error_{posRoll}(n) - error_{posRoll}(n-1))
$$

$$
error_{ratePitch}(n) = error_{ratePitch}(n-1) + K_{Pitch} \cdot (error_{posPitch}(n) - error_{posPitch}(n-1))
$$

> ⚠️ **Note:** The incremental P controller is a recent update and **has not been fully tested on the UAV**. Results may vary until validation is complete.

#### Parameters P Controller:

The parameters are adjusted for each of the angles, $$K_{Roll}$$ and $$K_{Pitch}$$.


### PID controller (Incremental form):
The control law for a PI controller in the digital domain is expressed as:

$$
u(n) = u(n-1) + K_0 e(n) + K_1 e(n-1)+K_2 e(n-2)
$$

Digital PI controller in inner loop is implemented for Roll, Pitch and Yaw rates,

$$
error_{RateRoll}(n)=Ref_{rateRoll}(n)-Rate_{Roll}(n)
$$
$$
error_{RatePitch}(n)=Ref_{ratePitch}(n)-Rate_{Pitch}(n)
$$
$$
error_{RateYaw}(n)=Ref_{rateYaw}(n)-Rate_{Yaw}(n)
$$

$$
u_{RollRate}(n) = u_{RollRate}(n-1) + K_0 \cdot error_{RateRoll}(n) + K_1 \cdot error_{RateRoll}(n-1)+ K_2 \cdot error_{RateRoll}(n-2)
$$
$$
u_{PitchRate}(n) = u_{PitchRate}(n-1) + K_0 \cdot error_{RatePitch}(n) + K_1 \cdot error_{RatePitch}(n-1) + K_2 \cdot error_{RatePitch}(n-2)
$$
$$
u_{YawRate}(n) = u_{YawRate}(n-1) + K_0 \cdot error_{RateYaw}(n) + K_1 \cdot error_{RateYaw}(n-1)+ K_2 \cdot error_{RateYaw}(n-2)
$$

#### Parameters PID controller:

The parameters are adjusted for each of the angular rates,

$$
K_0 = K_p + \frac{K_p}{2T_i} T_s + \frac{K_p \cdot T_d}{T_s} 
$$

$$
K_1 = -K_p + \frac{K_p}{2T_i} T_s- 2\frac{K_p \cdot T_d}{T_s} 
$$

$$
K_2 = \frac{K_p \cdot T_d}{T_s} 
$$


#### Control Signal Inner Loop:

$$
motor_1=u_{PWR}-u_{RollRate}-u_{PitchRate}-u_{YawRate}
$$
$$
motor_2=u_{PWR}-u_{RollRate}+u_{PitchRate}+u_{YawRate}
$$
$$
motor_3=u_{PWR}+u_{RollRate}+u_{PitchRate}-u_{YawRate}
$$
$$
motor_4=u_{PWR}+u_{RollRate}-u_{PitchRate}+u_{YawRate}
$$

## 🔉 Signal Processing: 1D Kalman Filter  

To improve the accuracy of angular position (Pitch, Roll) measurements, a **1D Kalman Filter** was implemented.  
This filter provides an optimal estimation by combining sensor measurements with a predictive model, effectively reducing noise and improving stability for the control loop.  

### Filter Equations  

**Prediction step:** 

$$
\hat{x}_{k|k-1} = \hat{x}_{k-1|k-1}
$$  

$$
P_{k|k-1} = P_{k-1|k-1} + Q
$$  

**Update step:**  

$$
K_k = \frac{P_{k|k-1}}{P_{k|k-1} + R}
$$  

$$
\hat{x}_{k|k} = \hat{x}_{k|k-1} + K_k \big(z_k - \hat{x}_{k|k-1}\big)
$$  

$$
P_{k|k} = (1 - K_k) P_{k|k-1}
$$  

Where:  
- $$( \hat{x}_{k|k})$$: estimated state (filtered measurement)  
- $$z_k$$: raw sensor measurement  
- $$P$$: error covariance  
- $$Q$$: process noise covariance  
- $$R$$ : measurement noise covariance  
- $$K_k$$: Kalman gain  

### Filter Equations implementation
The 1D Kalman filter is implemented as follows:

$$
x_{k|k-1} = x_{k-1|k-1} + T_s \cdot u_k
$$
$$
P_{k|k-1} = P_{k-1|k-1} + T_s^2 \cdot \sigma_{\dot{\theta}}^2
$$
$$
K_k = \frac{P_{k|k-1}}{P_{k|k-1} + \sigma_{\theta}^2}
$$
$$
x_{k|k} = x_{k|k-1} + K_k \cdot (x_{k|k-1})
$$
$$
P_{k|k} = (1 - K_k) P_{k|k-1}
$$

Where:
- $$x$$ = estimated angle  
- $$u_k$$ = angular rate (gyro input)  
- $$\sigma_{\dot{\theta}}^2$$ = gyro variance (set constant value)
- $$\sigma_{\theta}^2$$ = accelerometer variance (set constant value) 

### Angle Estimation from Accelerometer

The roll and pitch angles are estimated from accelerometer measurements as:

$$
\text{Roll} = \arctan\left(\frac{Acc_Y}{\sqrt{Acc_X^2 + Acc_Z^2}}\right) \cdot \frac{180}{\pi}
$$
$$
\text{Pitch} = -\arctan\left(\frac{Acc_X}{\sqrt{Acc_Y^2 + Acc_Z^2}}\right) \cdot \frac{180}{\pi}
$$

Where:
- $$(Acc_X, Acc_Y, Acc_Z)$$ = accelerometer readings in each axis.  



## :triangular_ruler: Connection Diagram
<p align="center">
<img width="700" alt="Esquema de conexiones" src="https://github.com/user-attachments/assets/f63e5bc2-419b-4b56-9e3a-c95954f03a04" />
</p>

## 🖼️ 3D PCB Render
The following renders show the **next hardware update**, where the control PCB and the power stage have been fully modularized into separate boards.
<table>
  <tr>
    <td align="center">
      <img  alt="control_dron_v5" src="https://github.com/user-attachments/assets/f69de7d6-d442-445a-9913-7fef57ca80f0" width="550"><br>
      <sub>PCB Render - Control Stage - NEXT UPDATE </sub>
    </td>
    <td align="center">
        <img alt="stage power dron_v1_front" src="https://github.com/user-attachments/assets/54d63556-4770-4720-82ef-f94c9d20400e" width="550"><br>
      <sub>PCB Render - Power Stage - NEXT UPDATE </sub>
    </td>
  </tr>
</table>

Below is the layout of the currently installed PCB.

<p align="center">
<img src="https://github.com/user-attachments/assets/74c0878c-502a-40b0-9939-4558bfcc2535" alt="PCB Render" width="500">
</p>

## 🖼️ UAV Prototype

<table>
  <tr>
    <td align="center">
      <img src="https://github.com/user-attachments/assets/fda18613-69da-4717-908f-0935936f2a6b" alt="UAV Prototype" width="450"><br>
      <sub>UAV Prototype - Base structure </sub>
    </td>
    <td align="center">
      <img src="https://github.com/user-attachments/assets/8f7ca1dd-dca9-4222-b1d9-626072373875" alt="UAV Prototype PCB assembled" width="400"><br>
      <sub>PCB assembled on UAV - Version 4 </sub>
    </td>
    <td align="center">
      <img src="https://github.com/user-attachments/assets/183a825f-f665-487a-9a31-e490cf9b1156" alt="UAV Setup" width="400"><br>
      <sub>UAV Setup overview</sub>
    </td>
  </tr>
</table>

## 🔗 Supporting / Experimental Projects

Additional boards and experiments developed during system validation:

- 🧭 Dual IMU Evaluation Board  
  Test platform used to characterize BMI088 and ICM-42605 sensors before integration into the Flight Control Stage  
  https://github.com/CrissCCL/UAV_Dual_IMU

- Digital Control — Anti-Windup (simulation examples)  
  https://github.com/CrissCCL/Digital_ControlAntiWindup


## 🤝 Support projects
 Support me on Patreon [https://www.patreon.com/c/CrissCCL](https://www.patreon.com/c/CrissCCL)

## 📜 License
MIT License
