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


## 🔄 Control Hardware Update (In Progress)

As part of the ongoing system upgrade, the control stage is being redesigned to include a **dual-IMU architecture**, integrating:

- **BMI088** – robust IMU with excellent vibration tolerance
- **ICM-4205** – high-bandwidth IMU for improved dynamic response

The use of two different IMUs aims to:
- Improve sensor redundancy
- Compare noise characteristics and dynamic behavior
- Enhance state estimation robustness under high vibration conditions

This update is currently under development and will be validated through bench testing and flight experiments.


# 🔄 Control Architecture (Cascaded)

<p align="center">
<img width="500" alt="UAV Control Loop" src="https://github.com/user-attachments/assets/9d342de3-f207-44a3-a338-7d241f2a026d" />
</p>

The UAV implements a **cascaded digital control architecture** running at:

- **Sampling period:** Ts = 0.005 s  
- **Frequency:** 200 Hz  
- **MCU:** Teensy 4.0 (FPU enabled)


## 🎯 Controlled Variables

| Loop  | Variable              | Type               |
|-------|----------------------|--------------------|
| Outer | Roll, Pitch          | Angular Position   |
| Inner | Roll, Pitch, Yaw     | Angular Velocity   |


# 🟦 Outer Loop — Proportional Angle Controller (Angle → Rate)

The outer loop regulates the angular position (Roll and Pitch) and generates the reference angular rate for the inner loop.

Angle error:

$$
e_\theta(n) = \theta_{cmd}(n) - \theta(n)
$$

A **proportional controller** converts the angle error into a desired angular rate:

$$
\omega_{ref}^{raw}(n) = K_p\ e_\theta(n)
$$

The reference rate is limited to avoid excessive angular velocity:

$$
\omega_{ref}^{raw}(n) =
\mathrm{clamp}\big(\omega_{ref}^{raw}(n), -\omega_{max}, \omega_{max}\big)
$$


## Rate Target Slew Limiting

To avoid abrupt changes in the commanded angular rate, the reference is rate-limited:

$$
\omega_{ref}(n) =
\mathrm{slew\_limit}\big(\omega_{ref}(n-1), \omega_{ref}^{raw}(n)\big)
$$

This prevents sudden jumps in angular velocity caused by abrupt stick inputs.

## Target Rate Filtering

The rate reference is additionally smoothed using a first-order filter:

$$
\omega_{ref}^{f}(n) =
\alpha_t\,\omega_{ref}^{f}(n-1)
+
(1-\alpha_t)\,\omega_{ref}(n)
$$

Where

- $$\omega_{ref}^{f}$$ is the **filtered angular rate reference**
- $$\alpha_t$$ is the smoothing coefficient (`TARGET_ALPHA`)

The filtered reference is used by the inner rate controller.

## Control Architecture Diagram

The implemented flight controller follows a cascaded structure:

$$
\theta_{cmd}
\rightarrow
P_{angle}
\rightarrow
\omega_{ref}^{f}
\rightarrow
PID_{rate} + FF
\rightarrow
Mixer
\rightarrow
Motors
$$

Where

- $$P_{angle}$$ generates the angular rate reference
- $$PID_{rate}$$ stabilizes angular velocity
- $$FF$$ improves reference tracking
- the mixer converts control torques into motor commands


## Angular Rate Filtering

Before entering the rate controller, the measured angular velocities are filtered to reduce vibration and high-frequency noise.

The filtering chain implemented in the firmware is:

$$
\omega_f(n) = LPF\big(Notch(\omega(n))\big)
$$

Where

- $$\omega(n)$$ is the raw gyroscope measurement  
- $$Notch(\cdot)$$ removes narrow-band vibration components  
- $$LPF(\cdot)$$ is a biquad low-pass filter used to attenuate high-frequency noise  
- $$\omega_f(n)$$ is the filtered angular rate used by the controller

This filtering stage improves controller robustness and prevents noise amplification in the derivative term.

In the firmware implementation this corresponds to:

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

# 🟥 Inner Loop — Rate PID with Feedforward

Rate error:

$$
e(n) = \omega_{ref}^{f}(n) - \omega_f(n)
$$

Control law:

$$
U(n) = K_p e(n) + I(n) + D_f(n) + FF(n)
$$

Where

- $$\omega_f$$ is the filtered angular rate
- $$D_f$$ is the filtered derivative contribution


## Feedforward Term

A feedforward contribution improves the dynamic response:

$$
FF(n) = K_{ff}\,\omega_{ref}^{f}(n)
$$

This anticipates the control effort required to track the commanded angular velocity.

---

## Final Inner Loop Expression

$$
U(n) = K_p e(n) + I(n) + D_f(n) + FF(n)
$$

Applied independently to:

- Roll rate
- Pitch rate
- Yaw rate

Parameters per axis:

$$
K_p,\quad T_i,\quad T_d,\quad D_{ALPHA},\quad K_{ff}
$$

## 🔹 Integral Term + Conditional Anti-Windup

The integral state is updated only when the saturation condition allows integration.  
This prevents integrator windup while preserving integral action when useful.

Discrete integral update:

$$
I(n) = I(n-1) + K_i T_s e(n)
$$

with:

$$
K_i = \frac{K_p}{T_i}
$$

If the actuator is saturated and the update would worsen saturation, the integral action is frozen:

$$
I(n) = I(n-1)
$$

## 🔹 Derivative Term (Derivative on Measurement)

The derivative action is applied on the measured angular rate rather than on the control error, reducing derivative kick during abrupt reference changes.

Let the discrete derivative of the measured rate be:

$$
\dot{\omega}_f(n) \approx \frac{\omega_f(n) - \omega_f(n-1)}{T_s}
$$

Then the raw derivative term is:

$$
D(n) = -K_p T_d \dot{\omega}_f(n)
$$

The negative sign indicates **derivative on measurement**.

## 🔹 D-Term First-Order Low-Pass Filter

A first-order IIR low-pass filter is applied to the raw derivative term:

$$
D_f(n) = \alpha D_f(n-1) + (1-\alpha)D(n)
$$

Where:

- $$D_f(n)$$ is the filtered derivative contribution
- $$\alpha$$ corresponds to `D_ALPHA` in firmware
- $$0 < \alpha < 1$$

### Embedded Implementation (Reference)

```cpp
// D-term (negative because derivative on measurement)
float Droll  = -(Kpr * Tdr) * dRateRoll;
float Dpitch = -(Kpp * Tdp) * dRatePitch;
float Dyaw   = -(Kpy * Tdy) * dRateYaw;

// D low-pass filter
Droll_f  = D_ALPHA * Droll_f  + (1.0f - D_ALPHA) * Droll;
Dpitch_f = D_ALPHA * Dpitch_f + (1.0f - D_ALPHA) * Dpitch;
Dyaw_f   = D_ALPHA * Dyaw_f   + (1.0f - D_ALPHA) * Dyaw;
```
## 🔹 Feedforward Term

A feedforward contribution is added from the **filtered target angular rate** to improve dynamic response and reduce tracking lag.

$$
FF(n) = K_{ff}\,\omega_{ref,f}(n)
$$

This anticipates the control effort required to track the commanded angular rate and reduces the burden on the feedback controller.

Feedforward is applied independently for **Roll, Pitch and Yaw**.

---

## ✅ Final Inner Loop Expression

The complete rate control law implemented in the firmware is:

$$
U(n) = K_p e(n) + I(n) + D_f(n) + FF(n)
$$

Where

$$
e(n) = \omega_{ref,f}(n) - \omega_f(n)
$$

Applied independently to:

- Roll rate  
- Pitch rate  
- Yaw rate  

Each axis has independently tuned parameters:

$$
K_p,\quad T_i,\quad T_d,\quad D\_ALPHA,\quad K_{ff}
$$

---

# 🔧 Motor Mixing Matrix (X Configuration)

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

### Motor Layout

- **M1:** Front Right  
- **M4:** Front Left  
- **M2:** Rear Right  
- **M3:** Rear Left  

---

# ⏱ Real-Time Execution

- Control frequency: **200 Hz**
- ESC PWM synchronized
- Float arithmetic (Teensy 4.0 hardware FPU)
- Telemetry via UART → Raspberry Pi 4B
- Offline validation in MATLAB

---

# 🧠 Design Summary

- Cascaded architecture improves disturbance rejection.
- **Outer loop uses a proportional controller (P)** to generate the angular rate reference.
- Inner loop positional PID uses **derivative on measurement** to reduce derivative kick.
- The D-term includes a **first-order low-pass filter** to reduce noise amplification.
- Angular-rate measurements are conditioned using **notch + biquad low-pass filtering**.
- A **feedforward term based on the filtered rate reference** improves transient tracking.
- Anti-windup is implemented using **conditional integrator update (`allow_integrator_update`)**, preventing integrator growth during actuator saturation.
- Control design is aligned with the identified motor lag

$$
\tau \approx 0.17\,s
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
