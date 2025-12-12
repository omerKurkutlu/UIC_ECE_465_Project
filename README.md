# IMU-Based On-Board Localization  
**UIC – ECE 465 Course Project**
[Omer Kurkutlu UIN:655968526], [Sam Sheikh-Ahmadi UIN:677862407]

This repository contains the **Arduino** and **Python** implementation of a lightweight, IMU-only localization system designed for on-board deployment in tiny robots and drones.  
The project focuses on estimating **orientation** and **relative position** using only IMU sensor data—without GPS, motion-capture systems, or external tracking.

---

## 📌 Project Overview
The goal of this project is to explore whether **IMU-only localization** can be used for on-board navigation in resource-constrained robotic systems.

This repo includes:

- **Arduino Nano 33 BLE** code for:
  - Reading 9-axis IMU data (accelerometer, gyroscope, magnetometer)
  - Running the Madgwick filter on-board
  - Estimating orientation (quaternions → roll/pitch/yaw)
  - Computing relative position via double integration

- **Python visualization tools** for:
  - Receiving BLE-transmitted pose data
  - Real-time 3D visualization of orientation and position

---

## 🎥 Project Presentation  
Project presentation link:  
🔗 https://docs.google.com/presentation/d/1BtVPBkaQrPdXGqAYNNXBZE2Dg3rO5JJO/edit?usp=sharing&ouid=107822084700445451411&rtpof=true&sd=true


