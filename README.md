# PACE 项目

## 项目简介

PACE (Precision Autonomous Control Engineering) 是一个基于 STM32 底层固件和 ROS 2 上位机的智能控制系统项目。该项目采用 Monorepo (单体仓库) 结构，实现了 STM32 底层固件与 ROS 2 上位机节点的物理隔离，提高了代码的可维护性和可扩展性。

## 目录结构

```text
pace/  <-- GitHub 仓库根目录
│
├── firmware/              <-- 【下位机】所有 STM32 代码
│   ├── a_board/           (原 A1 工程) - 底盘控制板
│   ├── c_board/           (原 C_TO_A1_A2 工程) - 遥控器接收板
│   └── README.md          (记录烧录方式与通讯协议)
│
├── pace_ws/               <-- 【上位机】ROS 2 工作空间
│   └── src/               <-- colcon build 编译区
│       │
│       ├── pace_control/  <-- [核心节点] 车辆控制节点
│       │   ├── package.xml
│       │   ├── CMakeLists.txt
│       │   ├── include/
│       │   └── src/
│       │
│       └── pace_interfaces/ <-- [公共区] 自定义 msg/srv 消息
│
├── .gitignore             (全局忽略：build/, install/, log/, *.bag)
└── README.md              (项目总说明)
```

## 项目架构

### 下位机战区 (firmware/)
- **a_board**：底盘控制板，基于 STM32F427，负责底盘电机控制和状态反馈
- **c_board**：遥控器接收板，基于 STM32F407，负责接收遥控器信号并转发

### 上位机战区 (pace_ws/)
- **pace_control**：核心控制节点，负责接收上位机指令，处理数据，发送控制命令到底盘
- **pace_interfaces**：公共接口包，定义了上位机与下位机之间的通讯消息格式

## 核心功能

1. **底盘控制**：实现底盘电机的速度控制、位置控制和状态反馈
2. **遥控器接收**：接收遥控器信号并转换为控制指令
3. **上位机集成**：通过 ROS 2 提供统一的控制接口和数据可视化
4. **通讯协议**：基于 CAN 总线和串口的可靠通讯
5. **串口转 ROS 2**：将串口命令转换为 ROS 2 话题消息

## 开发环境

### 下位机开发环境
- **IDE**：STM32CubeIDE
- **编译器**：ARM GCC
- **构建系统**：CMake
- **调试工具**：ST-Link V2
- **操作系统**：Windows / Linux

### 上位机开发环境
- **操作系统**：Ubuntu 22.04 LTS
- **ROS 2 版本**：Humble Hawksbill
- **构建工具**：colcon
- **依赖管理**：rosdep
- **编译器**：GCC 11.3

## 使用方法

### 1. 下位机固件编译与烧录

#### 底盘控制板 (a_board)
1. 进入目录：`cd firmware/a_board`
2. 创建构建目录：`mkdir build && cd build`
3. 配置编译环境：`cmake ..`
4. 编译代码：`cmake --build .`
5. 烧录固件：使用 ST-Link 或 OpenOCD 进行烧录
   ```bash
   openocd -f openocd.cfg -c "program build/DiPan.elf verify reset exit"
   ```

#### 遥控器接收板 (c_board)
1. 进入目录：`cd firmware/c_board`
2. 创建构建目录：`mkdir build && cd build`
3. 配置编译环境：`cmake ..`
4. 编译代码：`cmake --build .`
5. 烧录固件：使用 ST-Link 或 OpenOCD 进行烧录
   ```bash
   openocd -f openocd.cfg -c "program build/RC_C_Com.elf verify reset exit"
   ```

### 2. 上位机 ROS 2 节点编译与运行

#### 编译上位机代码
1. 进入 ROS 2 工作空间：`cd pace_ws`
2. 安装依赖：`rosdep install --from-paths src --ignore-src -r -y`
3. 编译代码：`colcon build`
4. 激活环境：`source install/setup.bash`

#### 运行核心控制节点

##### cmd_vel_to_serial_node 节点
该节点负责将 ROS 2 的 `cmd_vel` 话题消息转换为串口命令，发送给底层控制板。

**使用方法**：

1. **默认参数运行**：
   ```bash
   ros2 run pace_control cmd_vel_to_serial_node
   ```

2. **自定义参数运行**：
   ```bash
   ros2 run pace_control cmd_vel_to_serial_node --ros-args \
     -p serial_port:=/dev/ttyUSB0 \
     -p baud_rate:=115200 \
     -p linear_threshold:=0.05 \
     -p angular_threshold:=0.05 \
     -p high_speed_threshold:=0.5
   ```

**参数说明**：
- `serial_port`：串口设备路径，默认 `/dev/ttyUSB0`
- `baud_rate`：串口波特率，默认 115200
- `linear_threshold`：线速度阈值，默认 0.05
- `angular_threshold`：角速度阈值，默认 0.05
- `high_speed_threshold`：高速档阈值，默认 0.5

### 3. 系统启动流程

1. **启动下位机**：
   - 烧录两个控制板的固件（a_board 和 c_board）
   - 连接 ST-Link 调试器
   - 给开发板供电

2. **启动上位机**：
   - 启动 ROS 2 系统：`source /opt/ros/humble/setup.bash`
   - 激活工作空间：`source pace_ws/install/setup.bash`
   - 运行控制节点：`ros2 run pace_control cmd_vel_to_serial_node`

3. **测试系统**：
   - 发布速度命令：`ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"`
   - 查看节点状态：`ros2 node info /cmd_vel_to_serial_node`
   - 查看话题信息：`ros2 topic info /cmd_vel`

## 通讯协议

### 底盘控制板 (a_board) 通讯
- **接口**：CAN 总线
- **波特率**：500Kbps
- **消息格式**：
  - 电机控制指令：ID 0x100，数据长度 8 字节
  - 电机状态反馈：ID 0x200，数据长度 8 字节

### 遥控器接收板 (c_board) 通讯
- **接口**：UART (串口)
- **波特率**：115200
- **消息格式**：
  - 遥控器数据：`RC::<ch1>,<ch2>,<ch3>,<ch4>*`
  - 状态反馈：`RC_OK*`

## 故障排除

### 常见问题

1. **串口连接失败**
   - 检查串口设备是否正确连接
   - 确认串口权限是否正确
   - 检查串口参数设置是否匹配

2. **电机无响应**
   - 检查 CAN 总线连接是否正确
   - 确认电机驱动是否正常
   - 检查控制指令是否正确发送

3. **ROS 2 节点启动失败**
   - 检查依赖是否正确安装
   - 确认工作空间是否正确激活
   - 查看节点日志获取详细错误信息

### 调试命令

```bash
# 查看串口设备
ls /dev/ttyUSB*

# 查看节点列表
ros2 node list

# 查看话题列表
ros2 topic list

# 查看节点信息
ros2 node info /cmd_vel_to_serial_node

# 查看话题数据
ros2 topic echo /cmd_vel
```

## 贡献指南

1. **代码风格**：遵循 ROS 2 代码风格指南
2. **提交规范**：使用 Conventional Commits 规范
3. **分支管理**：
   - `main`：主分支，稳定版本
   - `dev`：开发分支，新功能开发
   - `feature/*`：特性分支，特定功能开发
4. **PR 流程**：
   - 从 `dev` 分支创建特性分支
   - 完成开发后提交 PR 到 `dev` 分支
   - 代码审查通过后合并到 `dev` 分支
   - 定期从 `dev` 分支合并到 `main` 分支

## 许可证

本项目采用 MIT 许可证，详见 [LICENSE](LICENSE) 文件。

## 联系方式

- **项目维护者**：PACE 开发团队
- **邮箱**：pace-dev@example.com
- **GitHub**：https://github.com/pace-robotics/pace

## 固件更新日志

### 底盘控制板 (a_board)
- **v1.0**：初始版本，实现基本电机控制功能
- **v1.1**：优化 PID 控制参数，提高电机响应速度
- **v1.2**：添加故障检测与保护机制

### 遥控器接收板 (c_board)
- **v1.0**：初始版本，实现遥控器信号接收
- **v1.1**：添加信号滤波功能
- **v1.2**：优化通讯协议

### 上位机 (pace_ws)
- **v1.0**：初始版本，实现基本 ROS 2 节点
- **v1.1**：添加 cmd_vel_to_serial_node 节点
- **v1.2**：优化节点参数配置和错误处理
