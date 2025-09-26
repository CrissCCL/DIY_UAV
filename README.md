# 🚁 DIY UAV with PID Control ⚠️ *Work in Progress*

## 📖 Overview
This DIY UAV (Unmanned Aerial Vehicle) project is being developed with an automatic control loop using PID controllers.  
The system controls angular position (Pitch and Roll) and angular velocity (Yaw) to achieve stable and precise flight.  

During development, the signal-to-noise ratio (S/N) of measurements was improved, and the stability of the electronic circuitry was enhanced. Brushless motors are used for propulsion, and a Teensy microcontroller handles the control loop in real time. Feedback is provided by gyroscopes and accelerometers, enabling precise stabilization and responsiveness to disturbances.

> ⚠️ **Note:** This project is currently under development. Features and hardware are subject to change.

## 📂 Contents
- `/Hardware` → schematics, PCB layouts, and component lists.
- `/control_uav` → C/C++ code implementing PID control for angular stabilization.
- `/docs` → photos, setup diagrams, and videos.

## 🔄 Control Loops
<p align="center">
<img src="docs/control_loop_uav.png" alt="UAV Control Loop" width="500">
</p>

### Controlled Variables
- **Pitch & Roll** → Angular position control (stabilization)
- **Yaw** → Angular velocity control (rotation rate)

## 🖼️ UAV Prototype
<p align="center">
<img src="docs/uav_prototype.jpg" alt="UAV Prototype" width="400">
</p>
<p align="center">
<img src="docs/uav_setup.jpg" alt="UAV Setup" width="400">
</p>

## 📜 License
MIT License
