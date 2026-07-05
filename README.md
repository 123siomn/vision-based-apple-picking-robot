\# 基于视觉定位与手眼标定的移动式苹果采摘机器人控制系统设计



本项目基于 YOLO、树莓派、STM32、深度相机、机械臂和移动底盘，设计面向苹果采摘场景的视觉定位与抓取控制系统。



\## Project Name



vision-based-apple-picking-robot



\## 项目总体架构



\- 树莓派：负责 YOLO 视觉识别、深度相机数据读取、目标定位、任务决策和串口通信

\- 底盘 STM32：负责循迹巡逻、编码器测速、PID 控制和超声波避障

\- 机械臂 STM32：负责接收目标位姿信息，执行逆运动学解算、舵机控制和抓取动作



\## 当前进度



\- 已完成 YOLO 基础检测与目标锁定

\- 已完成 YOLO 与 STM32 的串口通信

\- 已完成机械臂 STM32 视觉数据帧解析与姿态调整

\- 已完成底盘循迹、编码器控制和超声波避障

\- 下一步：树莓派双串口通信框架搭建



\## 目录结构



```text

vision-based-apple-picking-robot/

├── raspberry\_pi/

│   ├── main.py

│   ├── vision.py

│   ├── arm\_uart.py

│   ├── base\_uart.py

│   └── protocol.py

├── stm32\_base/

├── stm32\_arm/

└── docs/

