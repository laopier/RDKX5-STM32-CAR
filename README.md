# 🏎️ RDK X5 + STM32: Heterogeneous Autonomous Robot
# 基于 RDK X5 (边缘AI) 与 STM32 的异构视觉循迹机器人

![Demo](2月15日.gif)

> 📺 **Video Demo on Bilibili**: [【深圳大学AutoLeader】基于RDKX5+STM32的视觉循迹与避障小车](https://www.bilibili.com/video/BV141ZtBBEUr/)
> 
> [![Bilibili](https://img.shields.io/badge/Bilibili-Video-blue?logo=bilibili&logoColor=white)](https://www.bilibili.com/video/BV141ZtBBEUr/) 👈 点击观看详细演示视频

## 📖 Introduction (项目简介)
本项目由 **深圳大学 AutoLeader 团队** 开发，实现了一套用于自动驾驶的**异构计算架构**。本项目核心在于打通 Linux 高性能计算平台与微控制器实时控制系统之间的壁垒。

- **Upper Computer**: Horizon **RDK X5** (Ubuntu 20.04/ROS2) - 负责车道线感知与高层路径规划。
- **Lower Controller**: **STM32F103** - 负责底层硬件控制与 PID 实时闭环。
- **Communication**: 自定义 UART 协议，实现上位机与下位机的高速指令传输。

## 👥 Team & Roles (团队分工)
| Member | Role | Responsibilities |
| :--- | :--- | :--- |
| 彭林海 | Team Leader & Vision | Project Management, Lane Detection Algorithm, Web Dashboard Design. |
| 廖宏商 | Lead Embedded & Systems | STM32 Firmware, UART Protocol Design, PID Control, System Integration & Chassis Design. |
| 蔡锐潜 | Hardware Engineer | Circuit Design, PCB Layout & Soldering, Power Management. |
| 种雨佳 | AI & Design | Model Training & Tuning, Logo Design, Project Documentation. |

## 🛠️ Tech Stack (技术栈)
- **Edge AI Platform**: Horizon RDK X5 (BPU Acceleration)
- **Embedded Control**: STM32F103C8T6 (Keil MDK)
- **Hardware Design**: **Custom PCB Layout** (嘉立创EDA) & Power Management
- **Vision Algorithm**: OpenCV & Deep Learning (BPU deployment)
- **Control Algorithm**: Incremental PID Control
- **Mechanical Design**: Custom 3D printed chassis (SolidWorks)

## 💻 My Key Contributions (廖宏商的核心工作)
作为项目**嵌入式与系统集成核心负责人**，我主导了跨平台数据链路与控制系统的开发：

1.  **System Integration & Protocol (系统集成与协议)**: 
    - 针对 Linux (RDK X5) 与 MCU (STM32) 的异构通信，设计了健壮的 **UART 帧头帧尾协议**，有效解决了高频通信下的数据丢包问题。
2.  **Firmware Development (固件开发)**: 
    - 重写了 STM32 **串口中断处理逻辑**，支持来自上位机 100Hz 的实时控制指令，显著降低了系统控制延时。
3.  **Advanced Control (闭环控制)**: 
    - 实现了**增量式 PID 算法**。针对小车在视觉循迹中的动态响应，独立完成了参数整定，确保小车在循迹过程中平滑过弯。
4.  **Mechanical Design (机械设计)**:
    - 基于 SolidWorks 设计了**模块化底盘**，确保了 RDK X5 与 STM32 的稳固安装，并兼顾了传感器视野与散热需求。
5.  **System Debugging (系统调试)**: 
    - 解决了硬件集成过程中的信号干扰与电平转换问题，确保了系统的全链路稳定性。

---

## 👤 Author (作者信息)
- **Name**: **廖宏商**
- **Institution**: 深圳大学机电与控制工程学院 (2025级本科生)
- **GitHub**: [@laopier](https://github.com/laopier)
- **Email**: [2998272181@qq.com]

> **Project Status**: Completed (Winter 2026)
