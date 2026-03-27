#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <limits>   //数值极限
#include <string.h>
#include <cmath> //M_PI
#include <chrono>//时间库
#include <algorithm>//算法库,sort排序,find查找
//激光点云消息需要的头文件
#include <laser_geometry/laser_geometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>


//tf2需要的头文件
#include "tf2/exceptions.h"     //tf2异常库
#include "tf2_ros/transform_listener.h" //tf2监听器库
#include "tf2_ros/buffer.h"     //tf2缓冲区库

//动态参数需要的头文件
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/floating_point_range.hpp"


//PCL需要的头文件
////#include "pcl_ros/point_cloud.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp> //ROS2标准点云类型
#include <pcl_conversions/pcl_conversions.h> //PCL与ROS2点云转换函数

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.hpp>

using namespace std;
using namespace pcl;

class LaserscanMerger : public rclcpp::Node
{
    public:
        LaserscanMerger() ;
        void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan , std::string topic);
        void pointcloud_to_laserscan(Eigen::MatrixXf points, pcl::PCLPointCloud2 *merged_cloud);
        rcl_interfaces::msg::SetParametersResult dynamic_param_callback(const std::vector<rclcpp::Parameter> &parameters);//动态参数回调函数

    private:
        laser_geometry::LaserProjection projector_; //激光投影器

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_; //合并的点云发布
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_publisher_;    //合并的源激光消息发布
        std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> scan_subscribers; //订阅激光扫描消息,放在容器中
        std::vector<bool> cloud_modified; //用于标记每个激光扫描消息是否被修改

        std::vector<pcl::PCLPointCloud2> clouds; //存储点云数据,cloud2,pcl库点云类型
        std::vector<string> input_topics;   //存储每个激光扫描消息的主题名称
        void laserscan_topic_parser();  //解析laserscan话题
       
        //动态参数回调句柄
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handler_; 

        //承载参数的变量
        double angle_min;
        double angle_max;
        double angle_increment;
        double time_increment;
        double scan_time;
        double range_min;
        double range_max;

        std::string destination_frame;
        std::string cloud_destination_topic;
        std::string scan_destination_topic;
        std::string laserscan_topics;   //laserscan话题列表，这里写的谁，就会订阅谁
        std::vector<bool> clouds_modified;

        //tf相关定义
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};//类中定义tf监听器
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;    //类中定义tf缓冲区
};       


LaserscanMerger::LaserscanMerger() : Node("laser_multi_merger")
{
    //%动态参数声明
    //angle_min参数
    rcl_interfaces::msg::ParameterDescriptor  angle_min_Descriptor;//创建一个参数描述对象
    angle_min_Descriptor.name = "angle_min";    //唯一对应的参数名
    angle_min_Descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;//参数类型
    angle_min_Descriptor.description = "angle_min dynamic_parameter"; //参数描述
    angle_min_Descriptor.floating_point_range.resize(1);//设置参数调节的范围只有1个
    auto &range_angle_min = angle_min_Descriptor.floating_point_range[0];//获取第一个取值范围的对象。auto &是C++的自动类型推导和引用声明，避免拷贝对象
    range_angle_min.from_value = -M_PI; //最小值
    range_angle_min.to_value = M_PI; //最大值
    range_angle_min.step = 0; //步长，如果不设置，或者设置为0，则无限制。
    this->declare_parameter<double>("angle_min", -3.14, angle_min_Descriptor); //声明参数，默认值为-3.14

    //angle_max参数
    rcl_interfaces::msg::ParameterDescriptor  angle_max_Descriptor;//创建一个参数描述对象
    angle_max_Descriptor.name = "angle_max";    //唯一对应的参数名
    angle_max_Descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;//参数类型
    angle_max_Descriptor.description = "angle_max dynamic_parameter"; //参数描述
    angle_max_Descriptor.floating_point_range.resize(1);//设置参数调节的范围只有1个
    auto &range_angle_max = angle_max_Descriptor.floating_point_range[0];//获取第一个取值范围的对象。auto &是C++的自动类型推导和引用声明，避免拷贝对象
    range_angle_max.from_value = -M_PI; //最小值
    range_angle_max.to_value = M_PI; //最大值
    range_angle_max.step = 0; //步长，如果不设置，或者设置为0，则无限制。
    this->declare_parameter<double>("angle_max", 3.14, angle_max_Descriptor); //声明参数，默认值为3.14

    //angle_increment参数
    rcl_interfaces::msg::ParameterDescriptor  angle_increment_Descriptor;//创建一个参数描述对象
    angle_increment_Descriptor.name = "angle_increment";    //唯一对应的参数名
    angle_increment_Descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;//参数类型
    angle_increment_Descriptor.description = "angle_increment dynamic_parameter"; //参数描述
    angle_increment_Descriptor.floating_point_range.resize(1);//设置参数调节的范围只有1个
    auto &range_angle_increment = angle_increment_Descriptor.floating_point_range[0];//获取第一个取值范围的对象。
    range_angle_increment.from_value = 0.0; //最小值
    range_angle_increment.to_value = M_PI; //最大值
    range_angle_increment.step = 0; //步长，如果不设置，或者设置为0，则无限制。
    this->declare_parameter<double>("angle_increment", 0.00613592332229, angle_increment_Descriptor); //声明参数，默认值为0.0061

    //time_increment参数
    rcl_interfaces::msg::ParameterDescriptor  time_increment_Descriptor;//创建一个参数描述对象
    time_increment_Descriptor.name = "time_increment";    //唯一对应的参数名
    time_increment_Descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;//参数类型
    time_increment_Descriptor.description = "time_increment dynamic_parameter"; //参数描述
    time_increment_Descriptor.floating_point_range.resize(1);//设置参数调节的范围只有1个
    auto &range_time_increment = time_increment_Descriptor.floating_point_range[0];//获取第一个取值范围的对象。
    range_time_increment.from_value = 0.0; //最小值
    range_time_increment.to_value = 1.0; //最大值
    range_time_increment.step = 0; //步长，如果不设置，或者设置为0，则无限制。
    this->declare_parameter<double>("time_increment", 0.0, time_increment_Descriptor); //声明参数，默认值为0

    //scan_time参数
    rcl_interfaces::msg::ParameterDescriptor  scan_time_Descriptor;//创建一个参数描述对象
    scan_time_Descriptor.name = "scan_time";    //唯一对应的参数名
    scan_time_Descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;//参数类型
    scan_time_Descriptor.description = "scan_time dynamic_parameter"; //参数描述
    scan_time_Descriptor.floating_point_range.resize(1);//设置参数调节的范围只有1个
    auto &range_scan_time = scan_time_Descriptor.floating_point_range[0];//获取第一个取值范围的对象。
    range_scan_time.from_value = 0.0; //最小值
    range_scan_time.to_value = 1.0; //最大值
    range_scan_time.step = 0; //步长，如果不设置，或者设置为0，则无限制。
    this->declare_parameter<double>("scan_time", 0.1, scan_time_Descriptor); //声明参数，默认值为0.1

    //range_min  0.1, 0, 50
    rcl_interfaces::msg::ParameterDescriptor  range_min_Descriptor;//创建一个参数描述对象
    range_min_Descriptor.name = "range_min";    //唯一对应的参数名
    range_min_Descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;//参数类型
    range_min_Descriptor.description = "range_min dynamic_parameter"; //参数描述
    range_min_Descriptor.floating_point_range.resize(1);//设置参数调节的范围只有1个
    auto &range_range_min = range_min_Descriptor.floating_point_range[0];//获取第一个取值范围的对象。
    range_range_min.from_value = 0.0; //最小值
    range_range_min.to_value = 50.0; //最大值
    range_range_min.step = 0; //步长，如果不设置，或者设置为0，则无限制。
    this->declare_parameter<double>("range_min", 0.1, range_min_Descriptor); //声明参数，默认值为0.1

    //range_max  19.59, 0, 50
    rcl_interfaces::msg::ParameterDescriptor  range_max_Descriptor;//创建一个参数描述对象
    range_max_Descriptor.name = "range_max";    //唯一对应的参数名
    range_max_Descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;//参数类型
    range_max_Descriptor.description = "range_max dynamic_parameter"; //参数描述
    range_max_Descriptor.floating_point_range.resize(1);//设置参数调节的范围只有1个
    auto &range_range_max = range_max_Descriptor.floating_point_range[0];//获取第一个取值范围的对象。
    range_range_max.from_value = 0.0; //最小值
    range_range_max.to_value = 50.0; //最大值
    range_range_max.step = 0; //步长，如果不设置，或者设置为0，则无限制。
    this->declare_parameter<double>("range_max", 19.59, range_max_Descriptor); //声明参数，默认值为0.1
  
    //%声明其他参数
    this->declare_parameter<std::string>("destination_frame", "cart_frame");   //声明参数，默认值为cart_frame
    this->declare_parameter<std::string>("cloud_destination_topic", "/merged_cloud"); 
    this->declare_parameter<std::string>("scan_destination_topic", "/scan_multi"); 
    this->declare_parameter<std::string>("laserscan_topics", "/front_scan_filtered /back_scan_filtered"); 

    //%变量赋值

    destination_frame = this->get_parameter("destination_frame").as_string();
    cloud_destination_topic = this->get_parameter("cloud_destination_topic").as_string();
    scan_destination_topic = this->get_parameter("scan_destination_topic").as_string();
    laserscan_topics = this->get_parameter("laserscan_topics").as_string();
    this->get_parameter("angle_min",angle_min);
    this->get_parameter("angle_max",angle_max);
    this->get_parameter("angle_increment",angle_increment);
    this->get_parameter("time_increment",time_increment);
    this->get_parameter("scan_time",scan_time);
    this->get_parameter("range_min",range_min);
    this->get_parameter("range_max",range_max);
    RCLCPP_INFO(this->get_logger(), "angle_increment is: %f", angle_increment);


    //%动态参数回调
    param_callback_handler_ = this->add_on_set_parameters_callback(
        std::bind(&LaserscanMerger::dynamic_param_callback, this, std::placeholders::_1)
    );

    //%tf2相关对象初始化
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());//创建保存坐标变换信息的缓冲区对象
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);//创建tf-listener对象

    this->laserscan_topic_parser();
    point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_destination_topic.c_str(), 1);
    laser_scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(scan_destination_topic.c_str(), 1);
}


rcl_interfaces::msg::SetParametersResult LaserscanMerger::dynamic_param_callback(const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result; //创建参数变更结果对象
    result.successful = true; //默认设置成功
    for(const auto &param : parameters)
    {
        if(param.get_name() == "angle_min")
        {
            int angle_min_new = param.as_double();//获取参数值
            //验证动态参数是否在范围内
            if(angle_min_new < -M_PI || angle_min_new > M_PI)
            {
                result.successful = false; //如果值不在范围内，设置失败
                result.reason = "angle_min must be between -pi and pi";
            }
            else
            {
                this->angle_min = angle_min_new; //如果值在范围内，更新参数值。这里写this不是必须的，只是为了代码的可读性。
                RCLCPP_INFO(this->get_logger(), "angle_min changed to: %d", angle_min_new); //打印参数值
            }
        }
        else if(param.get_name() == "angle_max")
        {
            int angle_max_new = param.as_double();//获取参数值
            //验证动态参数是否在范围内
            if(angle_max_new < -M_PI || angle_max_new > M_PI)
            {
                result.successful = false; //如果值不在范围内，设置失败
                result.reason = "angle_max must be between -pi and pi";
            }
            else
            {
                this->angle_max = angle_max_new; //如果值在范围内，更新参数值
                RCLCPP_INFO(this->get_logger(), "angle_max changed to: %d", angle_max_new); //打印参数值
            }
        }
        else if(param.get_name() == "angle_increment")
        {
            int angle_increment_new = param.as_double();//获取参数值
            //验证动态参数是否在范围内
            if(angle_increment_new < 0 || angle_increment_new > M_PI)
            {
                result.successful = false; //如果值不在范围内，设置失败
                result.reason = "angle_increment must be between 0.0 and pi";                
            }
            else
            {
                this->angle_increment = angle_increment_new; //如果值在范围内，更新参数值
                RCLCPP_INFO(this->get_logger(), "angle_increment changed to: %d", angle_increment_new); //打印参数值
            }
        }
        else if(param.get_name() == "time_increment")
        {
            int time_increment_new = param.as_double();//获取参数值
            //验证动态参数是否在范围内
            if(time_increment_new < 0 || time_increment_new > 1)
            {
                result.successful = false; //如果值不在范围内，设置失败
                result.reason = "time_increment must be between 0.0 and 1.0";
            }
            else
            {
                this->time_increment = time_increment_new; //如果值在范围内，更新参数值
                RCLCPP_INFO(this->get_logger(), "time_increment changed to: %d", time_increment_new); //打印参数值
            }
        }
        else if(param.get_name() == "scan_time")
        {
            int scan_time_new = param.as_double();//获取参数值
            //验证动态参数是否在范围内
            if(scan_time_new < 0 || scan_time_new > 1)
            {
                result.successful = false; //如果值不在范围内，设置失败
                result.reason = "scan_time must be between 0.0 and 1.0";
            }
            else
            {
                this->scan_time = scan_time_new; //如果值在范围内，更新参数值
                RCLCPP_INFO(this->get_logger(), "scan_time changed to: %d", scan_time_new); //打印参数值
            }
        }
        else if(param.get_name() == "range_min")
        {
            int range_min_new = param.as_double();//获取参数值
            //验证动态参数是否在范围内
            if(range_min_new < 0 || range_min_new > 50)
            {
                result.successful = false; //如果值不在范围内，设置失败
                result.reason = "range_min must be between 0.0 and 50.0";
            }
            else
            {
                this->range_min = range_min_new; //如果值在范围内，更新参数值
                RCLCPP_INFO(this->get_logger(), "range_min changed to: %d", range_min_new); //打印参数值
            }
        }
        else if(param.get_name() == "range_max")
        {
            int range_max_new = param.as_double();//获取参数值
            //验证动态参数是否在范围内
            if(range_max_new < 0 || range_max_new > 50)
            {
                result.successful = false; //如果值不在范围内，设置失败
                result.reason = "range_max must be between 0.0 and 50.0";
            }
            else
            {
                this->range_max = range_max_new; //如果值在范围内，更新参数值
                RCLCPP_INFO(this->get_logger(), "range_max changed to: %d", range_max_new); //打印参数值
            }
        }
        else
        {
            result.successful = false; //如果参数名不匹配，设置失败
            result.reason = "Unsupported parameter";
            RCLCPP_INFO(this->get_logger(), "Unsupported parameter occured!!!");
        }
    }
    return result;
}

//#解析laserscan话题
void LaserscanMerger::laserscan_topic_parser()
{
    //%获取要订阅的 LaserScan 话题列表
    std::istringstream iss(laserscan_topics);  // 将laserscan_topics参数里的字符串转为输入流
    std::set<std::string> tokens;              // 用于存储分割后的字符串
    //从输入流中逐个读取字符串, 并将其插入到 tokens 集合中
    std::copy(std::istream_iterator<std::string>(iss),  //copy的初始位置即为输入流
            std::istream_iterator<std::string>(),       //尾后迭代器，它表示流的结束位置，作为copy的结束位置
            std::inserter(tokens, tokens.begin()));     //插入迭代器，用于将元素插入到容器中，从头插入

    std::vector<std::string> tmp_input_topics;

    //%从话题列表中提取出与输入流字符匹配的LaserScan话题
    while (!tokens.empty())
    {
        RCLCPP_INFO(this->get_logger(), "Waiting for topics ...");
        auto topics = this->get_topic_names_and_types();//topics数据类型为std::map<std::string, std::vector<std::string>>
        rclcpp::sleep_for(std::chrono::seconds(1));//休眠1秒

        for(const auto& topic : topics)
        {
            for(const auto& type : topic.second)
            {
                if(type == "sensor_msgs/msg/LaserScan" && tokens.erase(topic.first) > 0)//如果话题类型为LaserScan, 且话题名称在tokens集合中有存在(同时删除该话题名，返回为1代表删除数量，set和map最多为1)
                {
                    tmp_input_topics.push_back(topic.first);
                }
            }
        }
    }

    //%经典三步走，sort-unique-erase，用于去除序列容器重复元素。这样获得的相关话题名唯一
    std::sort(tmp_input_topics.begin(), tmp_input_topics.end());//对tmp_input_topics进行排序
    std::vector<std::string>::iterator last= std::unique(tmp_input_topics.begin(), tmp_input_topics.end());
    tmp_input_topics.erase(last, tmp_input_topics.end());//删除重复元素

    //%如果话题数量不相等或者话题名称不相等，重新订阅
    if ((tmp_input_topics.size() != input_topics.size()) ||         //如果话题数量不相等
        !std::equal(tmp_input_topics.begin(), tmp_input_topics.end(), input_topics.begin())) //或者话题名称不相等
    {
        
        scan_subscribers.clear(); //关闭容器中所有的订阅者
        input_topics = tmp_input_topics;//更新input_topics
        if (input_topics.size() > 0)
        {
            scan_subscribers.resize(input_topics.size());
            clouds_modified.resize(input_topics.size());
            clouds.resize(input_topics.size());
            RCLCPP_INFO(this->get_logger(), "Subscribing to topics: %ld", scan_subscribers.size());//zu修饰unsigned size_t类型
            for (size_t i = 0; i < input_topics.size(); ++i)//根据laserscan_topics参数里的话题名，重新创建订阅
            {
                scan_subscribers[i] = this->create_subscription<sensor_msgs::msg::LaserScan>( 
                    input_topics[i], 1, 
                    [this, i](const sensor_msgs::msg::LaserScan::SharedPtr scan)
                    {
                        this->scanCallback(scan, input_topics[i]);
                    }
                );
                clouds_modified[i] = false; //初始化clouds_modified
                RCLCPP_INFO(this->get_logger(), "subscribe: %s ", input_topics[i].c_str()); //打印话题名称
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Not subscribed to any topic.");
        }
    }
}

//#点云话题订阅回调函数
void LaserscanMerger::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan , std::string topic)
{
    sensor_msgs::msg::PointCloud2 tmp_cloud;

    try 
    {
        //%获取base_link坐标系到laser坐标系的坐标变换
        //这里主要是为了等待tf_buffer_更新
        geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
            destination_frame,          //目标坐标系，base_link坐标系，在launch中定义  
            scan->header.frame_id,      //源坐标系，/front_scan_filtered话题数据的坐标系
            scan->header.stamp,         //查询的时刻，这里使用的是tf2::TimePointZero表示使用最新的变换
            tf2::durationFromSec(1.0)   //等待时间，或者写rclcpp::Duration::from_seconds(1.0)
        );
        //%将激光数据转为点云，并转到指定坐标系base_link下
        projector_.transformLaserScanToPointCloud(
            destination_frame,//指定坐标系
            *scan,            //输入的激光数据
            tmp_cloud,        //输出的点云数据，cloud2类型
            *tf_buffer_,       //tf坐标转换关系缓存区
            -1.0,             //截断值。这里不截断
            laser_geometry::channel_option::Distance//指定输出点云应包含哪些额外通道数据，这里是原始距离值
        );
    }
    catch(tf2::TransformException &ex)
    {
        RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
        return;
    }

    for(size_t i = 0; i < input_topics.size(); i++)
    {
        if (topic.compare(input_topics[i]) == 0)
        {
            pcl_conversions::toPCL(tmp_cloud, clouds[i]);//将ROS2标准点云类型转换为PCL库点云类型,输入各点云话题都转
            clouds_modified[i] = true;
        }
    }

    //% Count how many scans we have
    size_t totalClouds = 0;
    for (size_t i = 0; i < clouds_modified.size(); i++)
    {
        if (clouds_modified[i])
            totalClouds++;
    }

    //% Go ahead only if all subscribed scans have arrived
    if (totalClouds == clouds_modified.size())
    {
        pcl::PCLPointCloud2 merged_cloud = clouds[0];
        sensor_msgs::msg::PointCloud2 ros_merged_cloud;
        clouds_modified[0] = false;
        for(size_t i = 1;i < clouds_modified.size(); i++)
        {
            #if PCL_VERSION_COMPARE(>=, 1, 10, 0)   //PCL版本号比较，>=1.10.0 ROS2安装1.14.0
                pcl::concatenate(merged_cloud, clouds[i], merged_cloud);
            #else
                pcl::concatenatePointCloud(merged_cloud, clouds[i], merged_cloud);//从cloud0开始，拼接点云到merged_cloud
            #endif
            clouds_modified[i] = false;
        }
        pcl_conversions::fromPCL(merged_cloud, ros_merged_cloud);  //将PCL库cloud2类型拼接点云转为ROS2标准点云类型
        point_cloud_publisher_->publish(ros_merged_cloud);//发布点云，只能发布ROS2点云类型
        Eigen::MatrixXf points; //动态大小的eigen矩阵，float类型
        pcl::getPointCloudAsEigen(merged_cloud, points);//将cloud2点云转为Eigen矩阵。应为N行4列(x,y,z,distance)，每个点一行
        pointcloud_to_laserscan(points, &merged_cloud); //将点云转为激光类型数据
    }
}

//%将合并的点云转为源激光数据
void LaserscanMerger::pointcloud_to_laserscan(Eigen::MatrixXf points, pcl::PCLPointCloud2 *merged_cloud)
{
    auto output = std::make_shared<sensor_msgs::msg::LaserScan>();
    output->header = pcl_conversions::fromPCL(merged_cloud->header);//不仅能点云信息直接转，还能将头部header、头部内容stamp、PCL点云图image、每个点的字段pointfeild等单独转换
    output->header.frame_id = this->destination_frame;
    // output->header.stamp = merged_cloud->header.stamp; // ROS2 时间获取
    output->angle_min = this->angle_min;
    output->angle_max = this->angle_max;
    output->angle_increment = this->angle_increment;
    output->time_increment = this->time_increment;
    output->scan_time = this->scan_time;
    output->range_min = this->range_min;
    output->range_max = this->range_max;
    uint32_t ranges_size = std::ceil((output->angle_max - output->angle_min) / output->angle_increment);
    output->ranges.assign(ranges_size, std::numeric_limits<float>::infinity());

    for (int i = 0; i < points.cols(); ++i)
    {
        const float &x = points(0, i);
        const float &y = points(1, i);
        const float &z = points(2, i);

        if (isnan(x) || isnan(y) || isnan(z))
        {
            RCLCPP_DEBUG(this->get_logger(), "rejected for nan in point(%f, %f, %f)", x, y, z);
            continue;
        }

        double range_sq = pow(y, 2) + pow(x, 2);
        double range_min_sq_ = output->range_min * output->range_min;
        if (range_sq < range_min_sq_)
        {
            RCLCPP_DEBUG(this->get_logger(), "rejected for range %f below minimum value %f. Point: (%f, %f, %f)",
                        range_sq, range_min_sq_, x, y, z);
            continue;
        }

        double angle = atan2(y, x);
        if (angle < output->angle_min || angle > output->angle_max)
        {
            RCLCPP_DEBUG(this->get_logger(), "rejected for angle %f not in range (%f, %f)", angle, output->angle_min, output->angle_max);
            continue;
        }
        int index = (angle - output->angle_min) / output->angle_increment;

        if (output->ranges[index] * output->ranges[index] > range_sq)
            output->ranges[index] = sqrt(range_sq);
    }
    laser_scan_publisher_->publish(*output);//合并后的源激光消息发布。注意合并后的点云数据也发布了
}



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    this_thread::sleep_for(chrono::seconds(5)); //休眠5秒，等待tf2初始化
    rclcpp::spin(std::make_shared<LaserscanMerger>());
    rclcpp::shutdown();
    return 0;
}

















