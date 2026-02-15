# 🏎️ RDK X5 + STM32: Heterogeneous Autonomous Robot
# 基于 RDK X5 (边缘AI) 与 STM32 的异构视觉循迹机器人

![Demo](2月15日.gif)
> 📺 **Video Demo on Bilibili**: [【深圳大学AutoLeader】基于RDKX5+STM32的视觉循迹与避障小车](https://www.bilibili.com/video/BV141ZtBBEUr/)

## 📖 Introduction (项目简介)
This project implements a **heterogeneous computing architecture** for autonomous driving, developed by a team of 4 students from Shenzhen University

- **Upper Computer**: Horizon **RDK X5** (Ubuntu 20.04/ROS2) - Handles high-performance AI inference (Lane detection) and path planning.
- **Lower Controller**: **STM32F103** - Handles real-time motion control (PID) and sensor data acquisition.
- **Communication**: Custom UART protocol for high-speed command transmission.

## 👥 Team & Roles (团队分工)
| Member | Role | Responsibilities |
| :--- | :--- | :--- |
| 廖宏商 | System Integration & Control | PID Tuning, RDK X5 Integration. | Mechanical Design | Chassis Modeling (SolidWorks), 3D Printing, Structural Optimization. |
| 彭林海 | **Vision & Full Stack** | **Lane Detection (AI), Web Dashboard Design, Real-time Web-Client Communication (WebSocket/HTTP).** |
| 蔡锐潜 | Hardware Engineer | Circuit Design, PCB Soldering, Power Management, Sensor Calibration. |
| 种雨佳 | Model Training & Tuning, Project Documentation, Visual Design |

## 🛠️ Tech Stack (技术栈)
- **Edge AI Platform**: Horizon RDK X5 (BPU Acceleration)
- **Embedded Control**: STM32F103C8T6 (Keil MDK)
- **Hardware Design**: **Custom PCB Layout** (EasyEDA/Altium) & Power Management
- **Vision Algorithm**: OpenCV & Deep Learning (BPU deployment)
- **Control Algorithm**: Incremental PID Control
- **Mechanical Design**: Custom 3D printed chassis (SolidWorks)

## ⚙️ System Architecture (系统架构)
`[Camera] -> [RDK X5 BPU] -> (UART) -> [STM32] -> (PWM) -> [Motors]`

## 💻 My Key Contributions (我的核心工作)
As the member responsible for **Control & Integration**, I focused on bridging the gap between AI and Motion:
1.  **Protocol Design**: Designed a robust frame header/footer protocol to prevent data packet loss between Linux (RDK X5) and MCU (STM32).
2.  **Driver Optimization**: Rewrote the STM32 serial interrupt handler to support high-frequency control commands (100Hz).
3.  **Closed-loop Control**: Implemented and tuned the PID algorithm to ensure the car tracks the lane smoothly without oscillation.
4.  **System Debugging**: Solved the signal interference issues during the hardware integration phase.
5.  **Mechanical Design (SolidWorks)**:- Designed a **modular chassis** to securely mount the RDK X5 computing unit and STM32 controller.
    
  

---
**Project Status**: Completed (Winter 2026)
