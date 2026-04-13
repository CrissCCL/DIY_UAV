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


## 🧩 System Architecture — Hardware Update (Next Revision)

The UAV platform follows a **modular hardware stack**, where control and power electronics are implemented on **independent PCBs**.

This figure illustrates the **next-generation hardware stack**, highlighting the physical separation between control and power electronics.

- 🧠 Flight Control Stage (sensing + control + I/O)
- ⚡ Power Stage (distribution + regulation)
- 🚁 UAV frame integration

<p align="center">
  <img src="https://github.com/user-attachments/assets/e5505aba-81e0-4fd2-8924-9d0b97acb467" width="600">
</p>
<p align="center">
  <sub>
   New modular electronics stack — improved noise isolation, maintainability and hardware scalability
  </sub>
</p>

## 🧩 Hardware Modules

🚀 **Current development focus:** Modular electronics redesign and flight-control hardware integration.

The UAV electronics follow a **modular architecture**, where control and power subsystems are implemented on **independent PCBs**.  
This separation improves reliability, reduces electrical noise coupling, simplifies maintenance, and allows faster hardware iteration.

| Module | Description |
|------------|---------------------------------------------|
| 🛩️ **Flight Control Stage** | Teensy-based control PCB with dual-IMU redundancy, real-time control and PWM actuation |
| ⚡ **Power Stage** | Power distribution and regulation for ESCs and control electronics |

### Repositories

- 🛩️ Flight Control Stage → https://github.com/CrissCCL/UAV_FlightControl_Stage
- ⚡ Power Stage → https://github.com/CrissCCL/UAV_PowerStage  


## 🔬 Propulsion Dynamics Identification

The motor–ESC dynamic lag was experimentally identified using acoustic spectral ridge tracking.

Repository:
👉 https://github.com/CrissCCL/DIY_UAV_Motor_System_Identification

This module provides:
- Motor time constant (τ = 0.17 s)
- Rotational axis plant models (Gp, Gq, Gr)
- Bandwidth constraints for rate loop tuning

## 🔗 ROS 2 Field Tuning Integration

This UAV project also includes a dedicated ROS 2 integration repository for **telemetry monitoring**, **runtime PID tuning**, and **remote field operation** during experimental tests.

👉 **ROS 2 UAV PID Tuning Bridge**  
🔗 https://github.com/CrissCCL/UAV_ControlTunning_ROS_integration

This repository documents the use of a **Raspberry Pi** as a ROS 2 bridge communicating over **UART** with the **Teensy-based flight controller**, enabling remote operation from an **iPad via SSH** while both the iPad and the Raspberry Pi are connected to the **phone hotspot**.

Main documented elements:
- ROS 2 UART bridge for telemetry and PID parameter exchange
- Runtime PID tuning without reflashing firmware
- Remote SSH workflow from iPad
- Architecture diagram and Raspberry Pi ↔ Teensy wiring schematic


## 📊 Project Status
| Component                  | Status                     |
|-----------------------------|----------------------------|
| Brushless Motors Setup      | ✅ Completed               |
| Teensy PID Control Loop     | ✅ Completed              |
| Model Development          |   ✅ Completed                 |
| Sensor Signal improvement   |  ✅ Completed                 |
| Dual IMU testing    | ✅ Completed                 |
| PCB Design with Improved structure and connections  | ✅ Completed            |
| Pitch, Roll & Yaw Angular Velocity Control| ⚙️ In Progress          |
| Pitch & Roll Stabilization  | ⚙️ In Progress             |
| Prototype Flight Testing    | ⚠️ Not Started             |



## 🌐 YouTube
📺 [Test 1](https://youtube.com/shorts/LwX8zSV23eY?feature=share)

📺 [Test 2](https://youtube.com/shorts/dZo7ZcapqBg?feature=share)

📺 [Test 3](https://youtube.com/shorts/Z043N4uVOiI?feature=share)

📺 [Upgrade Power Stage](https://youtube.com/shorts/vSKApaUKDLE?feature=share)


## ⚙️ System Description

- **Controller:** Teensy 4.0 microcontroller (high-speed MCU for improved stability and processing)   
- **Sensors:** 3-axis accelerometer and gyroscope (IMU module) - Dual-IMU configuration (BMI088 + ICM-42605)
- **Actuators:** Brushless motors with ESCs  
- **Control Strategy:** PID control for Pitch, Roll (angular position) and Yaw (angular velocity)  
- **Estimation filter:** 1D Kalman filter applied to angular measurements (per-axis)  
- **Sampling period:** **0.005 seconds (200 Hz)**  
- **Data transmission:** UART link to a **Raspberry Pi 4B** for telemetry and logging  
- **Data logging:** Raspberry Pi stores telemetry streams to `.csv` files for model validation and analysis  
- **Visualization:** Offline monitoring and plotting via Matlab


# 🔄 Control Architecture (Cascaded)

<p align="center">
<img width="500" alt="UAV Control Loop" src="https://github.com/user-attachments/assets/9d342de3-f207-44a3-a338-7d241f2a026d" />
</p>

The UAV implements a **cascaded digital control architecture** running at:

- **Sampling period:** Ts = 0.005 s  
- **Frequency:** 200 Hz  
- **MCU:** Teensy 4.0 (FPU enabled)

## 🎯 Controlled Variables

| Loop  | Variable          | Type             |
|-------|-------------------|------------------|
| Outer | Roll, Pitch       | Angular Position |
| Inner | Roll, Pitch, Yaw  | Angular Velocity |

---

# Control Architecture Overview

The flight controller follows a cascaded structure:

$$
\theta_{cmd}
\rightarrow
P_{angle}
\rightarrow
\omega_{ref}^{raw}
\rightarrow
\mathrm{slew\_limit}
\rightarrow
\omega_{ref}
\rightarrow
LPF
\rightarrow
\omega_{ref}^{f}
\rightarrow
PID_{rate} + FF
\rightarrow
Mixer
\rightarrow
Motors
$$

Where:

- **Angle controller** generates the angular-rate reference
- **Rate PID** stabilizes angular velocity
- **Feedforward** improves transient response
- **Mixer** converts control actions into motor commands


# 🟦 Outer Loop — Proportional Angle Controller

The outer loop converts the angular position error (Roll and Pitch) into a desired angular-rate reference for the inner loop.

Angle error:

$$
e_\theta(n) = \theta_{cmd}(n) - \theta(n)
$$

A proportional controller generates the raw rate reference:

$$
\omega_{ref}^{raw}(n) = K_p e_\theta(n)
$$

The reference rate is limited to avoid excessive angular velocity:

$$
\omega_{ref}^{raw}(n) =
\mathrm{clamp}\left(\omega_{ref}^{raw}(n), -\omega_{max}, \omega_{max}\right)
$$

## Rate Target Slew Limiting

To avoid abrupt changes in the commanded angular rate, the reference is rate-limited:

$$
\omega_{ref}(n) =
\mathrm{slew\_limit}\left(\omega_{ref}(n-1), \omega_{ref}^{raw}(n)\right)
$$

This prevents sudden jumps in angular velocity caused by aggressive stick inputs.

## Target Rate Filtering

The commanded rate is further smoothed using a first-order filter:

$$
\omega_{ref}^{f}(n) =
\alpha_t \cdot \omega_{ref}^{f}(n-1)
+
(1-\alpha_t)\cdot \omega_{ref}(n)
$$

Where:

- $$\omega_{ref}^{f}(n)$$ is the filtered angular-rate reference
- $$\alpha_t$$ is the target smoothing coefficient

The filtered reference $$\omega_{ref}^{f}(n)$$ is used by the inner loop controller.


# 🟥 Inner Loop — Rate PID with Feedforward

## Angular Rate Filtering

Before entering the rate controller, the measured angular velocities are filtered to attenuate structural vibration and high-frequency sensor noise.

The filtering chain implemented in the firmware is:

$$
\omega_f(n) = H_{LPF}(z) H_{notch}(z) \omega(n)
$$

Where:

- $$\omega(n)$$ is the raw gyroscope measurement
- $$H_{notch}(z)$$ is a digital notch filter used to reject narrow-band vibration
- $$H_{LPF}(z)$$ is a biquad low-pass filter used to attenuate high-frequency noise
- $$\omega_f(n)$$ is the filtered angular-rate signal used by the controller

This filtering stage is especially important in UAV applications, where propeller–motor vibration can propagate into the IMU and degrade rate feedback, derivative action and overall closed-loop stability.

### Notch Filter

The notch filter is implemented as a second-order digital biquad in the form:

$$
H_{notch}(z) =
\frac{b_0 + b_1 z^{-1} + b_2 z^{-2}}
{1 + a_1 z^{-1} + a_2 z^{-2}}
$$

Using the RBJ notch design, the coefficients are computed from the center frequency $$f_0$$, quality factor $$Q$$ and sampling frequency $$f_s$$.

The normalized digital frequency is:

$$
\omega_0 = 2\pi \frac{f_0}{f_s}
$$

and

$$
\alpha = \frac{\sin(\omega_0)}{2Q}
$$

The unnormalized coefficients are:

$$
b_0 = 1
$$

$$
b_1 = -2\cos(\omega_0)
$$

$$
b_2 = 1
$$

$$
a_0 = 1 + \alpha
$$

$$
a_1 = -2\cos(\omega_0)
$$

$$
a_2 = 1 - \alpha
$$

The implemented normalized coefficients are therefore:

$$
\tilde b_0 = \frac{b_0}{a_0}, \quad
\tilde b_1 = \frac{b_1}{a_0}, \quad
\tilde b_2 = \frac{b_2}{a_0}
$$

$$
\tilde a_1 = \frac{a_1}{a_0}, \quad
\tilde a_2 = \frac{a_2}{a_0}
$$

A higher $$Q$$ produces a narrower notch, while a lower $$Q$$ increases attenuation bandwidth around the target vibration frequency.

### Firmware Implementation

```cpp
// RBJ notch
static inline void biquad_notch_init(Biquad &q, float f0_hz, float Q, float fs_hz) {
  float w0    = 2.0f * 3.1415926f * (f0_hz / fs_hz);
  float cosw0 = cosf(w0);
  float sinw0 = sinf(w0);
  float alpha = sinw0 / (2.0f * Q);

  float b0 = 1.0f;
  float b1 = -2.0f * cosw0;
  float b2 = 1.0f;
  float a0 = 1.0f + alpha;
  float a1 = -2.0f * cosw0;
  float a2 = 1.0f - alpha;

  q.b0 = b0 / a0;
  q.b1 = b1 / a0;
  q.b2 = b2 / a0;
  q.a1 = a1 / a0;
  q.a2 = a2 / a0;
  q.reset();
}
```

### Rate Filtering Pipeline

In the flight controller, the notch filter is applied before the low-pass stage:

```cpp
float rr = RateRoll;
float rp = RatePitch;
float ry = RateYaw;

if (USE_NOTCH) {
    rr = notch_r.step(rr);
    rp = notch_p.step(rp);
    ry = notch_y.step(ry);
}

RateRoll_f  = lpf_r.step(rr);
RatePitch_f = lpf_p.step(rp);
RateYaw_f   = lpf_y.step(ry);
```

Thus, the controller uses filtered angular-rate measurements of the form:

$$
\omega_f(n) = LPF(Notch(\omega(n)))
$$

This improves robustness of the rate loop, reduces noise amplification in the derivative term, and helps preserve stable control authority under vibration-rich operating conditions.

## Rate Error

The inner loop regulates angular velocity (Roll, Pitch, Yaw) using a positional discrete-time PID structure.

$$
e(n) = \omega_{ref}^{f}(n) - \omega_f(n)
$$

## Control Law

$$
U(n) = P(n) + I(n) + D_f(n) + FF(n)
$$

## Proportional Term

$$
P(n) = K_p e(n)
$$

## Integral Term

$$
I(n) = I(n-1) + K_i T_s e(n)
$$

with

$$
K_i = \frac{K_p}{T_i}
$$

## Conditional Anti-Windup

If actuator saturation occurs and the integral update would worsen saturation, the integrator is frozen:

$$
I(n) = I(n-1)
$$

This behaviour is implemented in firmware using:

```cpp
allow_integrator_update(...)
```

## Derivative Term (Derivative on Measurement)

The derivative action is applied to the measured angular rate, which reduces derivative kick during abrupt reference changes.

The discrete derivative of the filtered rate is

$$
\dot{\omega}_f(n) =
\frac{\omega_f(n) - \omega_f(n-1)}{T_s}
$$

The raw derivative term is

$$
D(n) = -K_p T_d \dot{\omega}_f(n)
$$

The negative sign indicates **derivative on measurement**.

## D-Term Filtering

The derivative term is filtered using a first-order IIR low-pass filter:

$$
D_f(n) =
\alpha \cdot D_f(n-1)
+
(1-\alpha)\cdot D(n)
$$

Where:

- $$D_f(n)$$ is the filtered derivative contribution
- $$\alpha$$ corresponds to `D_ALPHA` in firmware
- $$0 < \alpha < 1$$

Firmware implementation:

```cpp
float Droll  = -(Kpr * Tdr) * dRateRoll;
float Dpitch = -(Kpp * Tdp) * dRatePitch;
float Dyaw   = -(Kpy * Tdy) * dRateYaw;

Droll_f  = D_ALPHA * Droll_f  + (1.0f - D_ALPHA) * Droll;
Dpitch_f = D_ALPHA * Dpitch_f + (1.0f - D_ALPHA) * Dpitch;
Dyaw_f   = D_ALPHA * Dyaw_f   + (1.0f - D_ALPHA) * Dyaw;
```

## Feedforward Term

A feedforward contribution improves the response to commanded angular velocity:

$$
FF(n) = K_{ff} \cdot \omega_{ref}^{f}(n)
$$

This term anticipates the control effort required to track the filtered rate reference.

## Final Rate Controller

$$
U(n) = K_p e(n) + I(n) + D_f(n) + FF(n)
$$

Applied independently to:

- Roll rate
- Pitch rate
- Yaw rate

Controller parameters per axis:

$$
K_p,\quad T_i,\quad T_d,\quad D_{ALPHA},\quad K_{ff}
$$


# 🔧 Motor Mixing Matrix (X Configuration)

The control torques generated by the rate controller are converted into individual motor commands using an **X-configuration mixer**.

The mixer combines collective thrust with roll, pitch and yaw control actions.

Motor commands:

$$
M_1 = U_{PWR} - U_{Roll} - U_{Pitch} - U_{Yaw}
$$

$$
M_2 = U_{PWR} - U_{Roll} + U_{Pitch} + U_{Yaw}
$$

$$
M_3 = U_{PWR} + U_{Roll} + U_{Pitch} - U_{Yaw}
$$

$$
M_4 = U_{PWR} + U_{Roll} - U_{Pitch} + U_{Yaw}
$$

Where:

- $$U_{PWR}$$ is the collective throttle command
- $$U_{Roll}$$ is the roll control action
- $$U_{Pitch}$$ is the pitch control action
- $$U_{Yaw}$$ is the yaw control action

## Motor Layout

The quadrotor follows the standard **X configuration**:

| Motor | Position |
|------|----------|
| M1 | Front Right |
| M2 | Rear Right |
| M3 | Rear Left |
| M4 | Front Left |

The mixer distributes the control effort such that:

- **Roll control** generates opposite thrust between left and right motors
- **Pitch control** generates opposite thrust between front and rear motors
- **Yaw control** is generated through differential torque from the counter-rotating propellers

## Firmware Implementation

The mixer implemented in the firmware corresponds to:

```cpp
motor_1 = clampf(PwR_prop - Uroll_cmd - Upitch_cmd - Uyaw_cmd, MOTOR_MIN, MOTOR_MAX);
motor_2 = clampf(PwR_prop - Uroll_cmd + Upitch_cmd + Uyaw_cmd, MOTOR_MIN, MOTOR_MAX);
motor_3 = clampf(PwR_prop + Uroll_cmd + Upitch_cmd - Uyaw_cmd, MOTOR_MIN, MOTOR_MAX);
motor_4 = clampf(PwR_prop + Uroll_cmd - Upitch_cmd + Uyaw_cmd, MOTOR_MIN, MOTOR_MAX);
```

The outputs are then converted to PWM signals and sent to the ESCs.


# ⏱ Real-Time Execution

- Control frequency: **200 Hz**
- ESC PWM synchronized
- Float arithmetic (Teensy 4.0 hardware FPU)
- Telemetry via UART → Raspberry Pi 4B
- Offline validation in MATLAB


# 🧠 Design Summary

- Cascaded architecture improves disturbance rejection.
- The **outer loop uses a proportional controller** to generate the angular-rate reference.
- The commanded rate is shaped using **slew limiting** and **first-order filtering**.
- The **inner loop uses a positional PID with feedforward**.
- The derivative action is implemented as **derivative on measurement**, reducing derivative kick.
- The D-term includes a **first-order low-pass filter** to reduce noise amplification.
- Angular-rate measurements are conditioned using **notch + biquad low-pass filtering**.
- Anti-windup is implemented using **conditional integrator update** during saturation.
- The mixer converts roll, pitch and yaw control actions into motor-level commands for the X quadrotor configuration.
- Control design is aligned with the identified motor lag

$$
\tau \approx 0.17 s
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


## :triangular_ruler: Connection Diagram (update flight control stage)
<p align="center">
<img width="600" alt="Esquema de conexiones" src="https://github.com/user-attachments/assets/b7067729-2eee-4299-a9fc-034612cc47d8" />
</p>


## 🖼️ 3D PCB Render
The following renders show the **next hardware update**, where the control PCB and the power stage have been fully modularized into separate boards.
<table>
  <tr>
    <td align="center">
      <img  alt="control_dron_v5" src="https://github.com/user-attachments/assets/ba6c4783-f66f-44c5-8a2b-d1f296775a4a" width="550"><br>
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
