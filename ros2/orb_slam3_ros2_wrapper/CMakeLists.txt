cmake_minimum_required(VERSION 3.8)
project(orb_slam3_ros2_wrapper)

# 设置 C++ 标准，ORB-SLAM3 通常需要 C++14 或 C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# --------------------------------------------------------
# 1. 查找依赖
# --------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)         # 关键：用于 Eigen 和 TF 转换
find_package(tf2_geometry_msgs REQUIRED) # 关键：用于 Pose 转换
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)            # 关键：矩阵运算

# --------------------------------------------------------
# 2. 配置 ORB-SLAM3 路径
# --------------------------------------------------------
# 最佳实践：使用环境变量 ORBSLAM3_ROOT_DIR
if(NOT DEFINED ENV{ORBSLAM3_ROOT_DIR})
    message(FATAL_ERROR "Please set ORBSLAM3_ROOT_DIR environment variable to the ORB-SLAM3 installation path.")
endif()

set(ORB_SLAM3_DIR $ENV{ORBSLAM3_ROOT_DIR})
message(STATUS "ORB-SLAM3 DIR: ${ORB_SLAM3_DIR}")

# --------------------------------------------------------
# 3. 包含头文件
# --------------------------------------------------------
include_directories(
    include
    ${ORB_SLAM3_DIR}
    ${ORB_SLAM3_DIR}/include
    ${ORB_SLAM3_DIR}/Thirdparty/DBoW2/include
    ${ORB_SLAM3_DIR}/Thirdparty/g2o/include
    ${EIGEN3_INCLUDE_DIR}
)

# --------------------------------------------------------
# 4. 编译核心 Wrapper 库
# --------------------------------------------------------
# 将核心逻辑编译为库，方便多个节点调用
add_library(ORBSLAM3Wrapper
    src/SystemWrapper.cpp
)

# 链接 ROS 2 依赖到库
ament_target_dependencies(ORBSLAM3Wrapper
    rclcpp
    std_msgs
    sensor_msgs
    nav_msgs
    geometry_msgs
    cv_bridge
    tf2
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
)

# 链接 非ROS 依赖 (OpenCV, Eigen, ORB-SLAM3)
target_link_libraries(ORBSLAM3Wrapper
    ${OpenCV_LIBS}
    Eigen3::Eigen
    ${ORB_SLAM3_DIR}/lib/libORB_SLAM3.so
    ${ORB_SLAM3_DIR}/Thirdparty/g2o/lib/libg2o.so
    ${ORB_SLAM3_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
)

# --------------------------------------------------------
# 5. 编译可执行节点
# --------------------------------------------------------

# --- A. 单目节点 (Mono Node) ---
add_executable(mono_node
    src/mono_node.cpp
)
target_link_libraries(mono_node
    ORBSLAM3Wrapper
)
# 确保节点也能找到 ORB-SLAM3 的动态库（运行时需要）
ament_target_dependencies(mono_node rclcpp)

# --- B. 单目+IMU 节点 (Mono-IMU Node) ---
add_executable(mono_imu_node
    src/mono_imu_node.cpp
)
target_link_libraries(mono_imu_node
    ORBSLAM3Wrapper
)
ament_target_dependencies(mono_imu_node rclcpp)

# --------------------------------------------------------
# 6. 安装指令 (Install)
# --------------------------------------------------------

# 安装目标文件 (库和可执行文件)
install(TARGETS 
    ORBSLAM3Wrapper
    mono_node 
    mono_imu_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include/
)

# 安装配置文件夹 (YAML 参数文件)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# 安装 Launch 文件夹 (Python 启动脚本)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# --------------------------------------------------------
# 7. 导出依赖 (Export)
# --------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips the linter which checks for cpplint
  # comment the line when this style is not used
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 导出所有必要的依赖，以便其他包可以 find_package 找到此包
ament_export_include_directories(include)
ament_export_libraries(ORBSLAM3Wrapper)
ament_export_dependencies(
    rclcpp
    std_msgs
    sensor_msgs
    nav_msgs
    geometry_msgs
    cv_bridge
    tf2_ros
    tf2_eigen
    OpenCV
    Eigen3
)

ament_package()