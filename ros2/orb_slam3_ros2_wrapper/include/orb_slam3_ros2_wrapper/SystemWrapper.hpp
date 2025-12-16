#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>

// 引入 ORB-SLAM3 的核心头文件
#include <System.h> 

class SystemWrapper : public rclcpp::Node
{
public:
    // 构造函数：初始化 ORB-SLAM3 系统
    SystemWrapper(const std::string& node_name, ORB_SLAM3::System::eSensor sensor_type);
    
    // 核心处理函数：单目图像回调
    void mono_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    
    // 核心处理函数：单目+IMU 图像回调
    void mono_imu_callback(const sensor_msgs::msg::Image::SharedPtr img_msg);

    // IMU 数据回调 (用于缓存和同步)
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg);

private:
    ORB_SLAM3::System::eSensor sensor_type_;
    std::unique_ptr<ORB_SLAM3::System> slam_system_;
    
    // ROS 2 订阅器和发布器
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // IMU 队列用于同步 (Mono-IMU 模式需要)
    std::list<sensor_msgs::msg::Imu::SharedPtr> imu_data_buffer_;
    std::mutex imu_mutex_;

    // ORB-SLAM3 初始化参数
    std::string voc_file_;
    std::string settings_file_;

    // 姿态发布函数
    void publish_pose(const ORB_SLAM3::System::KeyFrame* kf);
    void publish_tf(const ORB_SLAM3::System::KeyFrame* kf);

    // 内部同步和处理函数
    std::vector<ORB_SLAM3::IMU::Point> get_synced_imu_data(const double timestamp);
};