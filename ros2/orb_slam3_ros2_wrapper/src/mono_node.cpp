#include "orb_slam3_ros2_wrapper/SystemWrapper.hpp"


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SystemWrapper>("orb_slam3_mono_node", ORB_SLAM3::System::MONOCULAR);
    
    // 订阅图像话题
    node->img_sub_ = node->create_subscription<sensor_msgs::msg::Image>(
        "image_topic", 10, std::bind(&SystemWrapper::mono_callback, node.get(), std::placeholders::_1));
    
    // 创建发布器给 Nav2 (例如发布 PoseStamped)
    node->pose_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("slam_pose", 10);
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}