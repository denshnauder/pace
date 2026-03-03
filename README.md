# PACE Team

## 项目简介
本仓库包含我在同济大学 PACE 车队算法组学习和参与开发的代码。项目主要分为两部分：底层的 STM32 固件驱动控制，以及上层的 ROS2 导航与控制节点。

这是我在智能驾驶领域的入门实践，主要目的是通过实际的软硬件协同开发，理解车辆的底层控制逻辑与上层决策机制。

## 目录结构
* `firmware/`: 包含 A 板和 C 板的底层嵌入式 C/C++ 驱动代码，基于 STM32 HAL 库开发。
* `pace_ws/`: 基于 ROS2 的工作空间。
  * `fishbot_cartographer/`: Cartographer 2D 建图与定位配置。
  * `pace_control/`: 将 cmd_vel 速度指令转化为串口下发指令的控制节点。

## 技术栈
* C/C++
* ROS2 (Robot Operating System 2)
* STM32 HAL 库 / FreeRTOS

## 当前学习与负责进展
* 熟悉并配置了底层硬件的串口（UART）、CAN 总线及 DMA 传输。
* 在 ROS2 环境下编写了基础的底盘通讯节点，实现速度指令的下发。
* 正在逐步学习并尝试引入行为树（Behavior Trees）来优化复杂环境下的决策逻辑。

## 运行说明
固件部分使用 CMake 和 arm-none-eabi-gcc 工具链编译，或者通过 STM32CubeMX 配合相应 IDE 导入。
ROS2 部分在 Ubuntu 22.04 (Humble) 下使用 colcon build 进行编译。
