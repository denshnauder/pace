# PACE 

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
