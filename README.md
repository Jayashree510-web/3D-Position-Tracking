# 3D Position Tracking using MPU9250 IMU

## Overview
This project utilizes the **MPU9250** Inertial Measurement Unit (IMU) to perform **3D position tracking** by integrating accelerometer, gyroscope, and magnetometer data. The data is processed using **Arduino, Python, and MATLAB** to track motion and estimate orientation in real time.

## Components of MPU9250
The **MPU9250** is a 9-axis motion tracking device that integrates:
- **3-Axis Gyroscope**: Measures angular velocity (rotation rate) around the x, y, and z axes.
- **3-Axis Accelerometer**: Measures acceleration, useful for detecting linear movement and orientation.
- **3-Axis Magnetometer**: Measures magnetic fields, providing compass-like functionality to enhance orientation sensing.

### 🔹 Key Features
✔ **9 Degrees of Freedom (DoF)** – Combines gyroscope, accelerometer, and magnetometer data for precise motion tracking.  
✔ **Digital Motion Processor (DMP)** – Built-in sensor fusion algorithms simplify obtaining orientation (pitch, roll, yaw).  

![MPU9250](https://github.com/user-attachments/assets/fe995a2d-018e-439b-8422-4bd9270df85b)

---

## 🚀 Implementation
The project integrates **Arduino, Python, and MATLAB** for real-time 3D motion tracking. Below is an overview of each stage:

### 1️⃣ **Arduino Code**
- The **Arduino** interfaces with the **MPU9250** to collect motion data.
- Performs **sensor calibration** by applying offsets to raw accelerometer data.
- Sends **adjusted accelerometer & gyroscope values** via **serial port**.


![Arduino Connection Diagram](https://github.com/user-attachments/assets/afaeff0e-e748-4c4c-ae5e-efe99eb0fa25)

---

### 2️⃣ **Python Code**
- Reads real-time data from the **serial port**.
- Stores sensor readings into a **CSV file** for further processing.
- Enables **continuous data collection** until stopped.

---

### 3️⃣ **MATLAB Code**
The **MATLAB script** processes raw sensor data to estimate the **3D position** of the object.

#### 📌 Key Functionalities:
✅ **Data Import** – Reads accelerometer and gyroscope data from the CSV file.  
✅ **Plotting** – Generates separate plots for **accelerometer & gyroscope** readings.  
✅ **Sensor Fusion (AHRS Algorithm)** – Computes orientation using accelerometer and gyroscope data.  
✅ **Tilt-Compensated Acceleration** – Accounts for orientation changes for improved accuracy.  
✅ **Linear Acceleration, Velocity & Position** – Uses **integration & high-pass filtering** to estimate the object's movement.  
✅ **3D Animation** – A visual representation of the object’s real-time motion.  

**MATLAB Visuals:**  
![MATLAB Plot](https://github.com/user-attachments/assets/d92c72cb-fc9f-4a85-b8c5-f899f78a1e2e)  
![3D Animation](https://github.com/user-attachments/assets/e3f3c878-dcfe-463f-bdcf-d8f3d16c09f6)

---

