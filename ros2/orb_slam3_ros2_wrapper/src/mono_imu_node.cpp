#include "orb_slam3_ros2_wrapper/SystemWrapper.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    // 实例化 Mono-IMU 模式的 Wrapper
    auto node = std::make_shared<SystemWrapper>("orb_slam3_mono_imu_node", ORB_SLAM3::System::MONO_IMU);
    
    // 订阅图像话题 (低频)
    node->img_sub_ = node->create_subscription<sensor_msgs::msg::Image>(
        "image_topic", 10, std::bind(&SystemWrapper::mono_imu_callback, node.get(), std::placeholders::_1));
        
    // 订阅 IMU 话题 (高频)
    // 队列深度应设置更高，以容纳更高频的 IMU 数据
    node->imu_sub_ = node->create_subscription<sensor_msgs::msg::Imu>(
        "imu_topic", 100, std::bind(&SystemWrapper::imu_callback, node.get(), std::placeholders::_1));
    
    // 创建姿态发布器
    node->pose_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("slam_pose", 10);
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}