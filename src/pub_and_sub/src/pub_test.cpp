//必要的库
#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <chrono>         // 时间库
#include <functional>     // 函数库,在ROS2中回调函数经常通过function和bind来绑定成员函数，比如订阅消息时的回调
#include <memory>         // 智能指针库,在ROS2中节点通常使用智能指针来管理资源，避免内存泄漏
#include <string>         // 字符串库,在ROS2中节点名称、话题名称等都是字符串，所以需要基本的字符串处理功能。

#include <geometry_msgs/msg/twist.hpp> 
#include <std_msgs/msg/bool.hpp>            
#include <std_msgs/msg/float32.hpp>    
#include <std_msgs/msg/string.hpp>   
#include <std_msgs/msg/int16.hpp>




class PubTest: public rclcpp::Node
{
public:
    PubTest() : Node("pub_test_node")
    {
        RCLCPP_INFO(this->get_logger(), "PubTest node has been started.");

        // 创建发布者
        cmd_pub = this->create_publisher<std_msgs::msg::Int16>("cmd_aa", 1); 
        // 创建定时器，定期发布消息
        timer_ = this->create_wall_timer(
            std::chrono::seconds(5), // 每5秒发布一次
            std::bind(&PubTest::publish_messages, this)
        );
    }

    ~PubTest() 
    {

    }

private:
    void publish_messages()
    {
        // 创建msg对象
        auto msg_aa = std_msgs::msg::Int16();

        // %设置msg_led内容并发布
        // msg_led.open_effect = true;
        // 发布消息
        cmd_pub->publish(msg_aa);
        RCLCPP_INFO(this->get_logger(), "Published RobotLed message.");
    }


private:
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr cmd_pub;   //发布者
    rclcpp::TimerBase::SharedPtr timer_{nullptr};


};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto pub_test = std::make_shared<PubTest>();
    rclcpp::spin(pub_test);
    rclcpp::shutdown();
    return 0;
}







