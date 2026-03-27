#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>       //手柄输入消息，对应sensor_msgs::msg::Joy
#include <geometry_msgs/msg/twist.hpp>



class JOY_control : public rclcpp::Node
{
    public:
    JOY_control() : Node("joy_control")
    {
        //声明参数
        this->declare_parameter("max_linear_speed", 0.5);
        this->declare_parameter("max_rotation_speed", 0.5);
        this->declare_parameter("accel_linear_speed_per", 0.1);
        //获取参数
        this->get_parameter("max_linear_speed", max_linear_speed);
        this->get_parameter("max_rotation_speed", max_rotation_speed);
        this->get_parameter("accel_linear_speed_per", accel_linear_speed_per);

        //订阅和发布话题创建
        cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        joy_sub = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&JOY_control::joy_callback, this, std::placeholders::_1));


    }
    ~JOY_control()
    {
        RCLCPP_INFO(this->get_logger(),"JOY_control node is shutting down.");
    }
    
    //订阅回调函数
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        set_linear_speed_X=msg->axes[1]*max_linear_speed;//左杆上下是1，左右是0
        now_rotation_speed_z=msg->axes[3]*max_rotation_speed;//右杆上下是4，左右是3
        // RCLCPP_INFO(this->get_logger(),"set:linear_x is %0.2f, set:axes is %d",set_linear_speed_X, msg->axes[1]);
    }

    void run()
    {
        rclcpp::Rate loop_rate(30); //30Hz的循环频率
        geometry_msgs::msg::Twist cmd_pub_msg;//要发送的速度消息
        while(rclcpp::ok())
        {
            rclcpp::spin_some(shared_from_this());//单次回调循环。类内调用类对象，指针赋权
            if(fabs(set_linear_speed_X) > 0.05) //设定速度不为0
            {
                if(now_linear_speed_x < set_linear_speed_X)
                    now_linear_speed_x += accel_linear_speed_per;
                else
                    now_linear_speed_x -= accel_linear_speed_per;
            }
            else
                now_linear_speed_x = set_linear_speed_X;

            //RCLCPP_INFO(this->get_logger(),"pub:x is %0.2f",now_linear_speed_x);
            is_zero = (fabs(now_linear_speed_x) < 1e-5) && (fabs(now_rotation_speed_z) < 1e-5);
 
            //%发布速度消息
            cmd_pub_msg.linear.x = now_linear_speed_x;
            cmd_pub_msg.angular.z = now_rotation_speed_z;

            if(is_zero && already_send_zero)
            {
                loop_rate.sleep();
                continue;
            }
            cmd_vel_pub->publish(cmd_pub_msg);
            already_send_zero = is_zero;            
            loop_rate.sleep();
        }

    }

    private:
    double now_linear_speed_x;  //当前要发布的速度大小
    double now_rotation_speed_z;
    double set_linear_speed_X;
    double max_linear_speed;
    double max_rotation_speed;
    double accel_linear_speed_per;

    bool already_send_zero = false;
    bool is_zero = false;

    //publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
    //subscriber
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;

};

int main(int argc, char *argv[]) 
{
    rclcpp::init(argc, argv);
    auto joycontrol = std::make_shared<JOY_control>();
    joycontrol->run();
    rclcpp::shutdown();
    return 0;
}