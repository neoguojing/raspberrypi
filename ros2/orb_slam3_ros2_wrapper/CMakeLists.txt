cmake_minimum_required(VERSION 3.8)
project(orb_slam3_ros2_wrapper)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# 查找 ROS 2 依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)

# --------------------------------------------------------
# ORB-SLAM3 依赖配置 (关键！)
# --------------------------------------------------------

# 假设 ORB-SLAM3 的路径设置在环境变量中或硬编码
# 最佳实践：使用环境变量 ORBSLAM3_ROOT_DIR
if(NOT DEFINED ENV{ORBSLAM3_ROOT_DIR})
    message(FATAL_ERROR "Please set ORBSLAM3_ROOT_DIR environment variable to the ORB-SLAM3 installation path.")
endif()

set(ORB_SLAM3_DIR $ENV{ORBSLAM3_ROOT_DIR})

# 包含 ORB-SLAM3 库的头文件
include_directories(
    include
    ${ORB_SLAM3_DIR}/include
    ${ORB_SLAM3_DIR}/Thirdparty/DBoW2/include
    ${ORB_SLAM3_DIR}/Thirdparty/g2o/include
)

# 查找 ORB-SLAM3 编译生成的库文件
# 注意：库路径可能因您的安装而异
link_directories(
    ${ORB_SLAM3_DIR}/lib
)

# --------------------------------------------------------
# 编译可执行文件
# --------------------------------------------------------

# 编译核心 Wrapper 源文件
add_library(ORBSLAM3Wrapper
    src/SystemWrapper.cpp
)

# 编译单目节点
add_executable(mono_node
    src/mono_node.cpp
)
target_link_libraries(mono_node
    ORBSLAM3Wrapper
    rclcpp
    sensor_msgs
    nav_msgs
    geometry_msgs
    cv_bridge
    tf2_ros
    ${OpenCV_LIBS}
    # ORB-SLAM3 库和它的第三方依赖
    ${ORB_SLAM3_DIR}/lib/libORB_SLAM3.so # 主库
    ${ORB_SLAM3_DIR}/Thirdparty/g2o/lib/libg2o.so
    ${ORB_SLAM3_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
    ${EIGEN3_LIBS}
    ${Pangolin_LIBRARIES} # 如果需要 Pangolin 可视化
)

# 编译单目+IMU节点 (mono_imu_node) (略)

# --------------------------------------------------------
# 安装和导出
# --------------------------------------------------------

# 安装头文件
install(DIRECTORY include/
  DESTINATION include/
)

# 安装可执行文件
install(TARGETS mono_node mono_imu_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装配置文件和启动文件
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME})

ament_export_targets(mono_node mono_imu_node HAS_ALL)
ament_export_dependencies(rclcpp sensor_msgs ...)
ament_package()