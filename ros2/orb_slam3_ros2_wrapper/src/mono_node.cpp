#include "orb_slam3_ros2_wrapper/SystemWrapper.hpp"

// 构造函数实现 (SystemWrapper::SystemWrapper)
// ... [在此处实现 ORB_SLAM3::System 的初始化和参数读取]

// 单目回调函数实现
void SystemWrapper::mono_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // 1. 转换图像格式
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    
    // 2. 调用 ORB-SLAM3 核心函数
    // 注意：时间戳转换
    double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    cv::Mat Tcw = slam_system_->TrackMonocular(cv_ptr->image, timestamp);

    // 3. 发布结果 (如果跟踪成功)
    if (!Tcw.empty()) {
        // Tcw 是世界坐标系到相机的变换矩阵，需要转换为 ROS 的 Pose 或 TF
        // this->publish_pose(Tcw); 
        // this->publish_tf(Tcw);
    }
}

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