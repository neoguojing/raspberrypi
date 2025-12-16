#include "orb_slam3_ros2_wrapper/SystemWrapper.hpp"

#include <chrono>
#include <opencv2/core/eigen.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using std::placeholders::_1;

// 构造函数
SystemWrapper::SystemWrapper(const std::string& node_name, ORB_SLAM3::System::eSensor sensor_type)
    : Node(node_name), sensor_type_(sensor_type)
{
    // 1. 声明并获取参数
    this->declare_parameter("voc_file", "");
    this->declare_parameter("settings_file", "");
    this->declare_parameter("image_topic", "/camera/image_raw");
    this->declare_parameter("imu_topic", "/imu/data");
    this->declare_parameter("world_frame_id", "map");
    this->declare_parameter("odom_frame_id", "odom");
    this->declare_parameter("robot_base_frame_id", "base_link");

    voc_file_ = this->get_parameter("voc_file").as_string();
    settings_file_ = this->get_parameter("settings_file").as_string();
    std::string img_topic = this->get_parameter("image_topic").as_string();
    std::string imu_topic = this->get_parameter("imu_topic").as_string();

    if (voc_file_.empty() || settings_file_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Please provide voc_file and settings_file parameters.");
        rclcpp::shutdown();
        return;
    }

    // 2. 初始化 ORB-SLAM3 系统
    // 注意：这可能需要几秒钟，如果在主线程中做，可能会阻塞 ROS 节点的启动。
    // 实际工程中建议放在单独的线程或生命周期管理的 on_configure 中。
    RCLCPP_INFO(this->get_logger(), "Loading ORB-SLAM3 System...");
    slam_system_ = std::make_unique<ORB_SLAM3::System>(
        voc_file_, settings_file_, sensor_type_, true);
    RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 System Loaded.");

    // 3. 创建发布器
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("slam_pose", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // 4. 创建订阅器
    // 根据传感器类型订阅不同的话题
    if (sensor_type_ == ORB_SLAM3::System::MONOCULAR) {
        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            img_topic, 10, std::bind(&SystemWrapper::mono_callback, this, _1));
    } 
    else if (sensor_type_ == ORB_SLAM3::System::MONO_IMU) {
        // IMU 订阅 (QoS 设为 SensorData 以获得最佳实时性)
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, rclcpp::SensorDataQoS(), std::bind(&SystemWrapper::imu_callback, this, _1));
        
        // 图像订阅
        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            img_topic, 10, std::bind(&SystemWrapper::mono_imu_callback, this, _1));
    }
    
    RCLCPP_INFO(this->get_logger(), "SystemWrapper initialized.");
}

// IMU 回调：缓存数据
void SystemWrapper::imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
    std::lock_guard<std::mutex> lock(imu_mutex_);
    // 防止内存无限增长，虽然 ORB-SLAM3 处理很快，但安全起见可以加限制
    if (imu_data_buffer_.size() > 2000) {
        imu_data_buffer_.pop_front();
    }
    imu_data_buffer_.push_back(imu_msg);
}

// 辅助函数：获取同步的 IMU 数据
std::vector<ORB_SLAM3::IMU::Point> SystemWrapper::get_synced_imu_data(const double timestamp)
{
    std::vector<ORB_SLAM3::IMU::Point> imu_vector;
    std::lock_guard<std::mutex> lock(imu_mutex_);

    auto it = imu_data_buffer_.begin();
    while (it != imu_data_buffer_.end()) {
        const auto& msg = *it;
        double t_imu = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

        // 取出图像时间戳之前的所有 IMU 数据
        if (t_imu <= timestamp) {
            ORB_SLAM3::IMU::Point p;
            // 注意：ORB-SLAM3 期望的单位通常是 rad/s 和 m/s^2，与 ROS 一致
            p.a = cv::Point3f(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
            p.w = cv::Point3f(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
            p.t = t_imu;
            imu_vector.push_back(p);
            
            // 从 buffer 中移除，避免重复处理
            it = imu_data_buffer_.erase(it);
        } else {
            // 后续数据是未来的，跳出
            break; 
        }
    }
    return imu_vector;
}

// 单目回调
void SystemWrapper::mono_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    if (!slam_system_) return;

    // 1. 转换图像
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // 2. 运行 SLAM
    double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    
    // Tcw: World 到 Camera 的变换矩阵 (4x4)
    cv::Mat Tcw = slam_system_->TrackMonocular(cv_ptr->image, timestamp);

    // 3. 处理结果 (状态判断)
    int state = slam_system_->GetTrackingState();
    if (state == ORB_SLAM3::Tracking::OK || state == ORB_SLAM3::Tracking::RECENTLY_LOST) {
        if (!Tcw.empty()) {
            // 这里我们需要手动调用发布逻辑，因为 TrackMonocular 返回的是 cv::Mat，不是 KeyFrame*
            // 我们在内部实现一个基于 cv::Mat 的 helper (不修改头文件的情况下)
            
            // 将 cv::Mat 转换为 ROS 格式并发布
            // 为了代码复用，我们在下面实现具体的转换逻辑
            
            // 获取 Twc (Camera 到 World，即相机的 Pose)
            cv::Mat Twc = Tcw.inv(); 
            
            // --- 发布 Pose 和 TF ---
            // 这里为了适配 Nav2，我们需要发布 map -> odom 的变换
            // 假设 SLAM 输出的是 map -> camera 的位姿
            
            // 构造 PoseStamped
            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header = msg->header;
            pose_msg.header.frame_id = this->get_parameter("world_frame_id").as_string();
            
            // 旋转矩阵转四元数
            cv::Mat Rwc = Twc.rowRange(0, 3).colRange(0, 3);
            cv::Mat twc = Twc.rowRange(0, 3).col(3);
            
            Eigen::Matrix3d R_eigen;
            Eigen::Vector3d t_eigen;
            cv::cv2eigen(Rwc, R_eigen);
            cv::cv2eigen(twc, t_eigen);
            
            Eigen::Quaterniond q(R_eigen);
            
            // 坐标系修正：ORB-SLAM (Camera Z-forward) -> ROS Base (X-forward)
            // 如果标定文件中已经处理了 T_body_cam，这里可能不需要。
            // 但通常 VSLAM 算的是相机 Pose。Nav2 需要的是 base_link Pose。
            // 简单的做法是：发布 map->camera_link 的 TF，利用 URDF 里的 camera_link->base_link 静态变换。
            
            pose_msg.pose.position.x = t_eigen.x();
            pose_msg.pose.position.y = t_eigen.y();
            pose_msg.pose.position.z = t_eigen.z();
            pose_msg.pose.orientation.x = q.x();
            pose_msg.pose.orientation.y = q.y();
            pose_msg.pose.orientation.z = q.z();
            pose_msg.pose.orientation.w = q.w();

            pose_pub_->publish(pose_msg);

            // 发布 TF: map -> odom (简化版：直接发 map -> camera_link/header.frame_id)
            // 在 Nav2 标准架构中，你应该计算 map -> odom = (map->base) * (odom->base)^-1
            // 这里为了简化，我们假设这是一个提供绝对定位的源，发布 map -> header.frame_id
            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header = pose_msg.header;
            tf_msg.child_frame_id = msg->header.frame_id; // camera link
            tf_msg.transform.translation.x = pose_msg.pose.position.x;
            tf_msg.transform.translation.y = pose_msg.pose.position.y;
            tf_msg.transform.translation.z = pose_msg.pose.position.z;
            tf_msg.transform.rotation = pose_msg.pose.orientation;
            
            tf_broadcaster_->sendTransform(tf_msg);
        }
    }
}

// 单目+IMU 回调
void SystemWrapper::mono_imu_callback(const sensor_msgs::msg::Image::SharedPtr img_msg)
{
    if (!slam_system_) return;

    double timestamp = img_msg->header.stamp.sec + img_msg->header.stamp.nanosec * 1e-9;
    
    // 1. 获取同步 IMU 数据
    std::vector<ORB_SLAM3::IMU::Point> imu_vector = this->get_synced_imu_data(timestamp);
    
    if (imu_vector.empty()) {
        // 如果没有 IMU 数据，等待一下或跳过（视策略而定）
        // RCLCPP_WARN(this->get_logger(), "Waiting for IMU data...");
        return;
    }

    // 2. 转换图像
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // 3. 运行 Mono-IMU SLAM
    cv::Mat Tcw = slam_system_->TrackMonocular(cv_ptr->image, timestamp, imu_vector);

    // 4. 处理结果 (逻辑同 mono_callback)
    // 为了代码整洁，你可以将 mono_callback 中的发布逻辑提取为一个私有函数
    int state = slam_system_->GetTrackingState();
    if (state == ORB_SLAM3::Tracking::OK || state == ORB_SLAM3::Tracking::RECENTLY_LOST) {
        if (!Tcw.empty()) {
             // ... 重复 mono_callback 中的发布代码 ...
             // 建议封装为 publish_ros_data(Tcw, img_msg->header);
             
             // 快速实现发布：
             cv::Mat Twc = Tcw.inv();
             cv::Mat Rwc = Twc.rowRange(0, 3).colRange(0, 3);
             cv::Mat twc = Twc.rowRange(0, 3).col(3);
             Eigen::Matrix3d R_eigen;
             Eigen::Vector3d t_eigen;
             cv::cv2eigen(Rwc, R_eigen);
             cv::cv2eigen(twc, t_eigen);
             Eigen::Quaterniond q(R_eigen);
             
             geometry_msgs::msg::PoseStamped pose_msg;
             pose_msg.header = img_msg->header;
             pose_msg.header.frame_id = this->get_parameter("world_frame_id").as_string();
             pose_msg.pose.position.x = t_eigen.x();
             pose_msg.pose.position.y = t_eigen.y();
             pose_msg.pose.position.z = t_eigen.z();
             pose_msg.pose.orientation.x = q.x();
             pose_msg.pose.orientation.y = q.y();
             pose_msg.pose.orientation.z = q.z();
             pose_msg.pose.orientation.w = q.w();
             pose_pub_->publish(pose_msg);
             
             geometry_msgs::msg::TransformStamped tf_msg;
             tf_msg.header = pose_msg.header;
             tf_msg.child_frame_id = img_msg->header.frame_id;
             tf_msg.transform.translation.x = t_eigen.x();
             tf_msg.transform.translation.y = t_eigen.y();
             tf_msg.transform.translation.z = t_eigen.z();
             tf_msg.transform.rotation = pose_msg.pose.orientation;
             tf_broadcaster_->sendTransform(tf_msg);
        }
    }
}

// ------------------------------------------------------------------
// 满足头文件定义的接口实现
// 注意：实际高频跟踪中我们使用的是 TrackMonocular 返回的 cv::Mat
// 这个函数通常用于从 KeyFrame 对象中提取数据，或者用于可视化/回环后的优化更新
// ------------------------------------------------------------------

void SystemWrapper::publish_pose(const ORB_SLAM3::System::KeyFrame* kf)
{
    if (!kf) return;
    
    // 从 KeyFrame 获取位姿 (Tcw)
    cv::Mat Tcw = kf->GetPose();
    cv::Mat Twc = Tcw.inv();
    
    // ... 转换逻辑同上 ...
    // 此处省略重复的转换代码，实际项目中建议封装 convertToPoseMsg(cv::Mat, header)
}

void SystemWrapper::publish_tf(const ORB_SLAM3::System::KeyFrame* kf)
{
    if (!kf) return;
    cv::Mat Tcw = kf->GetPose();
    // ... 转换逻辑同上 ...
}