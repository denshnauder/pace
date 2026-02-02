#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string>
#include <cmath>

class CmdVelToSerialNode : public rclcpp::Node {
public:
    CmdVelToSerialNode() : Node("cmd_vel_to_serial_node") {
    
        serial_port_ = open("/dev/ttyUSB0", O_RDWR);
        if (serial_port_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "无法打开串口 /dev/ttyUSB0，请确保 socat 正在运行");
        } else {
            configure_serial();
            RCLCPP_INFO(this->get_logger(), "已成功连接到串口: /dev/ttyUSB0");
        }

        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&CmdVelToSerialNode::cmd_vel_callback, this, std::placeholders::_1));
    }

    ~CmdVelToSerialNode() { if (serial_port_ >= 0) close(serial_port_); }

private:
    void configure_serial() {
        struct termios tty;
        if (tcgetattr(serial_port_, &tty) != 0) return;
        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tcsetattr(serial_port_, TCSANOW, &tty);
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::string buffer = "00000000"; // 默认刹车

        // 高低速档 (buffer[4])
        buffer[4] = (std::abs(msg->linear.x) > 0.5 || std::abs(msg->linear.y) > 0.5) ? '1' : '0';

        // 运动逻辑映射 (严格匹配 ca_com.c)
        if (std::abs(msg->angular.z) > 0.05) { // 自旋
            buffer[0] = '1';
            buffer[1] = (msg->angular.z > 0) ? '0' : '1'; // 0-左转, 1-右转
        } 
        else if (std::abs(msg->linear.x) > 0.05) { // 前后
            buffer[0] = '1';
            buffer[1] = '2'; // 不转向模式
            buffer[2] = (msg->linear.x > 0) ? '0' : '1'; // 0-前进, 1-后退
        } 
        else if (std::abs(msg->linear.y) > 0.05) { // 左右平移
            buffer[0] = '1';
            buffer[1] = '2';
            buffer[2] = (msg->linear.y > 0) ? '2' : '3'; // 2-左移, 3-右移
        }

        if (serial_port_ >= 0) {
            write(serial_port_, buffer.c_str(), 8);
        }
    }

    int serial_port_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelToSerialNode>());
    rclcpp::shutdown();
    return 0;
}