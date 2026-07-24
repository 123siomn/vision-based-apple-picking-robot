# 树莓派工具目录

本目录存放不参与正式运行、但后续标定和排障仍可能需要的工具。

- `calibration/depth_offset_scan.py`：检查 RGB 图像坐标与 Depth 坐标的像素偏移。
- `calibration/measure_grasp_depth.py`：记录固定抓取位置的目标深度参考值。
- `../serial/base_command_tool.py`：底盘串口协议诊断。
- `../serial/arm_command_tool.py`：机械臂统一数据帧诊断。
- `pi_speed_web.py`：底盘 PI 速度网页工具。
- `rgb_detection_preview.py`：纯 RGB 红色目标检测网页，用于调整相机角度；不连接串口或深度 SDK。

正式控制程序位于 `../vision/robot_controller.py`。
