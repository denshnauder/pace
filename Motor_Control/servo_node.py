import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import serial
import time

class ServoNode(Node):
    def __init__(self):
        super().__init__('stm32_servo_driver')
        
        # ---------------------------------------------------------
        # ⚠️ 关键修改：把这里的 'COM5' 改成你刚才查到的端口号！
        # 如果是 WSL/Linux，可能是 '/dev/ttyUSB0' 或 '/dev/ttyS5'
        # ---------------------------------------------------------
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1) 
            self.get_logger().info(f'成功连接到串口: {self.ser.port}')
        except Exception as e:
            self.get_logger().error(f'无法打开串口，请检查连线或端口号! 错误: {e}')
            return
        
        # 订阅 /cmd_angle 话题，接收整数角度
        self.subscription = self.create_subscription(
            Int32,
            'cmd_angle',
            self.listener_callback,
            10)
        
        # 定时检查 STM32 发回来的反馈
        self.timer = self.create_timer(0.05, self.read_serial_feedback)

    def listener_callback(self, msg):
        angle = msg.data
        if 0 <= angle <= 180:
            self.get_logger().info(f'>>> 发送指令: 转到 {angle} 度')
            # 转换成字节发送 (例如 90 -> b'\x5A')
            self.ser.write(bytes([angle]))
        else:
            self.get_logger().warn('角度必须在 0~180 之间！')

    def read_serial_feedback(self):
        if hasattr(self, 'ser') and self.ser.isOpen() and self.ser.in_waiting > 0:
            try:
                # 读取 STM32 发回来的 "STM32 Get: 90"
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    self.get_logger().info(f'<<< STM32反馈: {line}')
            except:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = ServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'ser') and node.ser.isOpen():
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()