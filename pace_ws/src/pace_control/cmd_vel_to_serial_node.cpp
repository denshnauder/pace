#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string>
#include <cmath>
#include <stdexcept>
#include <chrono>

class CmdVelToSerialNode : public rclcpp::Node {
public:
    CmdVelToSerialNode() : Node("cmd_vel_to_serial_node") {
        // 声明参数
        this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
        this->declare_parameter<int>("baud_rate", 115200);
        this->declare_parameter<double>("linear_threshold", 0.05);
        this->declare_parameter<double>("angular_threshold", 0.05);
        this->declare_parameter<double>("high_speed_threshold", 0.5);

        // 获取参数
        std::string serial_port = this->get_parameter("serial_port").as_string();
        int baud_rate = this->get_parameter("baud_rate").as_int();
        linear_threshold_ = this->get_parameter("linear_threshold").as_double();
        angular_threshold_ = this->get_parameter("angular_threshold").as_double();
        high_speed_threshold_ = this->get_parameter("high_speed_threshold").as_double();

        // 打开串口
        try {
            open_serial(serial_port, baud_rate);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "串口初始化失败: %s", e.what());
        }

        // 创建订阅
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&CmdVelToSerialNode::cmd_vel_callback, this, std::placeholders::_1));

        // 创建定时器，用于定期检查串口状态
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&CmdVelToSerialNode::check_serial_status, this));

        RCLCPP_INFO(this->get_logger(), "cmd_vel_to_serial_node 已初始化");
    }

    ~CmdVelToSerialNode() {
        if (serial_port_ >= 0) {
            close(serial_port_);
            RCLCPP_INFO(this->get_logger(), "串口已关闭");
        }
    }

private:
    void open_serial(const std::string& port, int baud_rate) {
        serial_port_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_port_ < 0) {
            throw std::runtime_error("无法打开串口: " + port);
        }

        // 配置串口
        struct termios tty;
        if (tcgetattr(serial_port_, &tty) != 0) {
            close(serial_port_);
            serial_port_ = -1;
            throw std::runtime_error("无法获取串口属性");
        }

        // 设置波特率
        speed_t baud = B115200;
        switch (baud_rate) {
            case 9600: baud = B9600; break;
            case 19200: baud = B19200; break;
            case 38400: baud = B38400; break;
            case 57600: baud = B57600; break;
            case 115200: baud = B115200; break;
            default: RCLCPP_WARN(this->get_logger(), "不支持的波特率，使用默认值 115200");
        }

        cfsetospeed(&tty, baud);
        cfsetispeed(&tty, baud);

        // 配置串口参数
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_oflag &= ~OPOST;
        tty.c_cc[VTIME] = 0;
        tty.c_cc[VMIN] = 0;

        if (tcsetattr(serial_port_, TCSANOW, &tty) != 0) {
            close(serial_port_);
            serial_port_ = -1;
            throw std::runtime_error("无法设置串口属性");
        }

        RCLCPP_INFO(this->get_logger(), "已成功连接到串口: %s, 波特率: %d", port.c_str(), baud_rate);
    }

    void check_serial_status() {
        if (serial_port_ < 0) {
            RCLCPP_WARN(this->get_logger(), "串口未连接，请检查设备是否正确连接");
        }
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (serial_port_ < 0) {
            RCLCPP_WARN(this->get_logger(), "串口未连接，无法发送命令");
            return;
        }

        // 生成控制命令
        std::string command = generate_control_command(msg);

        // 发送命令
        ssize_t bytes_written = write(serial_port_, command.c_str(), command.size());
        if (bytes_written != static_cast<ssize_t>(command.size())) {
            RCLCPP_ERROR(this->get_logger(), "串口写入失败，发送了 %zd 字节，期望发送 %zu 字节", bytes_written, command.size());
        } else {
            RCLCPP_DEBUG(this->get_logger(), "发送命令: %s", command.c_str());
        }
    }

    std::string generate_control_command(const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::string buffer = "00000000"; // 默认刹车

        // 高低速档 (buffer[4])
        buffer[4] = (std::abs(msg->linear.x) > high_speed_threshold_ || std::abs(msg->linear.y) > high_speed_threshold_) ? '1' : '0';

        // 运动逻辑映射
        if (std::abs(msg->angular.z) > angular_threshold_) { // 自旋
            buffer[0] = '1';
            buffer[1] = (msg->angular.z > 0) ? '0' : '1'; // 0-左转, 1-右转
            RCLCPP_DEBUG(this->get_logger(), "自旋命令: %s, 角速度: %.2f", (msg->angular.z > 0 ? "左转" : "右转"), std::abs(msg->angular.z));
        } 
        else if (std::abs(msg->linear.x) > linear_threshold_) { // 前后
            buffer[0] = '1';
            buffer[1] = '2'; // 不转向模式
            buffer[2] = (msg->linear.x > 0) ? '0' : '1'; // 0-前进, 1-后退
            RCLCPP_DEBUG(this->get_logger(), "前后命令: %s, 线速度: %.2f", (msg->linear.x > 0 ? "前进" : "后退"), std::abs(msg->linear.x));
        } 
        else if (std::abs(msg->linear.y) > linear_threshold_) { // 左右平移
            buffer[0] = '1';
            buffer[1] = '2';
            buffer[2] = (msg->linear.y > 0) ? '2' : '3'; // 2-左移, 3-右移
            RCLCPP_DEBUG(this->get_logger(), "平移命令: %s, 线速度: %.2f", (msg->linear.y > 0 ? "左移" : "右移"), std::abs(msg->linear.y));
        } else {
            RCLCPP_DEBUG(this->get_logger(), "停止命令");
        }

        return buffer;
    }

    int serial_port_ = -1;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    double linear_threshold_;
    double angular_threshold_;
    double high_speed_threshold_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CmdVelToSerialNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}