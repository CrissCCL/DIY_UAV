# 🚁 UAV with PID Control ⚠️ *Work in Progress*

## 📖 Overview
This DIY UAV (Unmanned Aerial Vehicle) project is being developed with an automatic control loop using PID controllers.  
The system controls angular position (Pitch and Roll) and angular velocity (Yaw) to achieve stable and precise flight.  

During development, the signal-to-noise ratio (S/N) of measurements was improved, and the stability of the electronic circuitry was enhanced. Brushless motors are used for propulsion, and a Teensy microcontroller handles the control loop in real time. Feedback is provided by gyroscopes and accelerometers, enabling precise stabilization and responsiveness to disturbances.

> ⚠️ **Note:** This project is currently under development. Features and hardware are subject to change.

## 📊 Project Status
| Component                  | Status                     |
|-----------------------------|----------------------------|
| Brushless Motors Setup      | ✅ Completed               |
| Sensor Signal Calibration   | ✅ Completed               |
| Teensy PID Control Loop     | ⚙️ In Progress             |
| Pitch, Roll & Yaw Angular Velocity Control| ⚙️ In Progress             |
| Pitch & Roll Stabilization  | ⚙️ In Progress             |
| Model Development          | ⚙️ In Progress               |
| Prototype Flight Testing    | ⚠️ Not Started             |


## 📂 Contents
- `/docs` → photos and setup diagrams.

## 🌐 YouTube
📺 [Short Video 1](https://youtube.com/shorts/LwX8zSV23eY?feature=share)

📺 [Short Video 2](https://youtube.com/shorts/dZo7ZcapqBg?feature=share)

📺 [Short Video 3](https://youtube.com/shorts/Z043N4uVOiI?feature=share)

## 🔄 Control Loops
<p align="center">
<img src="docs/diagrama uav.png" alt="UAV Control Loop" width="500">
</p>

### Controlled Variables
- **Pitch & Roll** → Angular position control (stabilization)
- **Pitch, Roll & Yaw** → Angular velocity control (rotation rate)

## 📐 Digital PID Control

The Module uses a discrete PI and P controller implemented on a Arduino microcontroller.  

Digital P controller in outer loop is implemented for Roll and Pitch angles,

$$
error_{posRoll}=Ref_{Roll}-Angle_{Roll}
$$
$$
error_{posPitch}=Ref_{Pitch}-Angle_{Pitch}
$$
$$
Ref_{rateRoll}=K_{Roll} error_{posRoll}
$$
$$
error_{posPitch}=K_{Pitch} error_{posPitch}
$$


### Parameters P Controller:

The parameters are adjusted for each of the angles,

$$
K_{Roll}
$$

$$
K_{Pitch}
$$

The control law for a PI controller in the digital domain is expressed as:

$$
u(n) = u(n-1) + K_0 e(n) + K_1 e(n-1)
$$

Digital PI controller in inner loop is implemented for Roll, Pitch and Yaw rates,

$$
error_{RateRoll}=Ref_{rateRoll}-RateRoll
$$
$$
error_{RatePitch}=Ref_{ratePitch}-RatePitch
$$
$$
error_{RateYaw}=Ref_{rateYaw}-RateYaw
$$

$$
u_{RollRate}(n) = u_{RollRate}(n-1) + K_0 error_{RateRoll}(n) + K_1 error_{RateRoll}(n-1)
$$
$$
u_{PitchRate}(n) = u_{PitchRate}(n-1) + K_0 error_{RatePitch}(n) + K_1 error_{RatePitch}(n-1)
$$
$$
u_{YawRate}(n) = u_{YawRate}(n-1) + K_0 error_{RateYaw}(n) + K_1 error_{RateYaw}(n-1)
$$

### Parameters PI controller:

The parameters are adjusted for each of the angular rates,

$$
K_0 = K_p + \frac{K_p}{2T_i} T_s
$$

$$
K_1 = -K_p + \frac{K_p}{2T_i} T_s
$$

### Control Signal Inner Loop:

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


## 🖼️ 3D PCB Render Version 4
<p align="center">
<img src="docs/control_dron_v4.jpg" alt="PCB Render" width="500">
</p>

## 🖼️ UAV Prototype
<p align="center">
<img src="docs/dron1.jpg" alt="UAV Prototype" width="400">
</p>

<p align="center">
<img src="docs/pcb5.jpg" alt="UAV Prototype PCB ensambled" width="400">
</p>
<p align="center">
<img src="docs/dron5.jpg" alt="UAV Setup" width="400">
</p>

## 📜 License
MIT License
