#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <mutex>
#include <chrono>
#include <cstring>
#include <cmath>

using namespace std::chrono_literals;

class CmdVelSerialNode:public rclcpp::Node
{
    public:
        CmdVelToSerialNode():Node("cmd_vel_to_serial_node"),seria_fd_(-1)
        {
            //1.声明节点参数
            this->declare_parameter<std::string>("port_name","/dev/ttyUSB0");//初始化串口，注意在运行前检查串口名称
            this->declare_parameter<int>("baud_rate",115200);//声明波特率
            this->declare_parameter<double>("timeout_sec",0.5);//看门狗超时时间（秒）
            this->declare_parameter<int>("publish_rate_hz",50);//串口下发频率

            this->get_parameter("port_name",port_name_);
            this->get_parameter("baud_rate",baud_rate_);
            this->get_parameter("timeout_sec",timeout_sec_);
            int rate_hz;
            this->get_parameter("publish_rate_hz",rate_hz);
            
            //2.初始化串口
            init_serial_port();

            //3.订阅控制信号话题
            cmd_vel_sub_=this->creat_subscription<geometry_msgs::msg::TWist>("订阅话题名称"，10,std::bind(&CmdVelToSerialNode::cmd_vel_callback,this,std::placeholders::_1));
            
        }
}