#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import SaveMap  # 用于调用地图保存服务

import numpy as np
import cv2
import threading
import time
import math
from collections import deque

# 导入 TF 相关库
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class FinalExploreNode(Node):
    def __init__(self):
        super().__init__('final_explore_node')
        
        self.declare_parameter('save_map_service', '/map_saver/save_map')

        # 初始化 Nav2 简单导航接口
        self.navigator = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )
        
        # 2. 初始化 TF 监听器 (替代 getRobotPose)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- 核心工程参数 ---
        self.SAFE_OFFSET = 0.45       # 安全退避距离：目标点会从边界向自由区回缩 45cm，防止撞墙
        self.NAV_TIMEOUT = 60.0      # 导航超时：防止局部路径规划死循环
        self.FINISH_THRESHOLD = 3     # 终止判定：连续 3 次扫描不到有效边界则认为地图已扫完
        self.UNKNOWN_THRESHOLD = 0.05  # 如果未知区域比例低于 5%，则认为完成
        self.MAP_SAVE_PATH = "auto_map_result" # 保存的文件名‘’
        self.MIN_GOAL_DISTANCE = 1.5  # 至少 80cm 远
        
        # --- 状态控制 ---
        self.map_msg = None           # 实时地图缓存
        self.failed_goals = deque(maxlen=60) # 失败点黑名单，防止机器人反复尝试不可达区域
        self.no_frontier_count = 0    # 空边界计数器
        
        # --- 服务客户端：调用 map_server 保存地图 ---
        self.save_map_cli = self.create_client(
            SaveMap,
            self.get_parameter('save_map_service').value
        )
        
        # 订阅 SLAM 发布的地图
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        
        # 延迟启动计时器：给 SLAM 和 Nav2 预留初始化时间
        self.timer = self.create_timer(5.0, self._start_logic)
        self.started = False

        self.nav_lock = threading.Lock()
        self.goal_handle = None
        self.result_future = None
        self.nav_status = 'IDLE'  # IDLE / NAVIGATING
        
        self.global_costmap = None
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.costmap_callback,
            10
        )

    def get_current_pose(self):
        try:
            # 修复点：添加 0.1s 的等待时间，处理 TF 发布延迟
            now = rclpy.time.Time()
            t = self.tf_buffer.lookup_transform(
                'map', 
                'base_link', 
                now, 
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            return t.transform.translation.x, t.transform.translation.y
        except Exception as ex:
            # 失败时尝试查找最近的一次有效变换
            try:
                t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time(seconds=0))
                return t.transform.translation.x, t.transform.translation.y
            except:
                return None, None

    def costmap_callback(self, msg):
        """接收全局代价地图，用于目标点安全性验证"""
        self.global_costmap = msg
        
    def map_callback(self, msg):
        """地图回调：不断更新本地地图快照"""
        self.map_msg = msg

    def _start_logic(self):
        """启动逻辑：仅执行一次，开启独立的计算线程"""
        if self.started or self.map_msg is None: return
        self.started = True
        self.timer.cancel()
        
        # 记录起点坐标
        rx, ry = self.get_current_pose()
        if rx is not None:
            self.start_pose_x, self.start_pose_y = rx, ry

        # 开启后台线程处理探索逻辑，避免阻塞 ROS2 节点的 spin 回调
        thread = threading.Thread(target=self.exploration_loop)
        thread.daemon = True
        thread.start()

    def _is_costmap_safe(self, wx, wy, safe_threshold=60):
        """
        检查世界坐标 (wx, wy) 在全局代价地图中是否安全。
        :param wx, wy: 世界坐标 (m)
        :param safe_threshold: cost 阈值，低于此值认为安全（推荐 60～80）
        :return: bool
        """
        if self.global_costmap is None:
            return False  # 代价地图未加载，保守返回不安全

        ox = self.global_costmap.info.origin.position.x
        oy = self.global_costmap.info.origin.position.y
        res = self.global_costmap.info.resolution
        width = self.global_costmap.info.width
        height = self.global_costmap.info.height

        # 转换为栅格坐标
        mx = int((wx - ox) / res)
        my = int((wy - oy) / res)

        # 边界检查
        if mx < 0 or mx >= width or my < 0 or my >= height:
            return False

        index = my * width + mx
        if index >= len(self.global_costmap.data):
            return False

        cost = self.global_costmap.data[index]
        
        # cost == 0: free, 1～252: 可通行但有代价, 253～255: lethal
        # 我们要求 cost < safe_threshold 才认为安全
        return cost < safe_threshold

    # ---------------- 核心算法：边界提取与评估 ----------------

    def get_best_frontier(self):
        """
        核心算法：
        1. 像素分析提取自由区与未知区的交界
        2. 将像素坐标精准映射到物理坐标（修复 Y 轴翻转）
        3. 评分筛选出最优目标
        """
        msg = self.map_msg
        w, h = msg.info.width, msg.info.height
        res = msg.info.resolution
        ox, oy = msg.info.origin.position.x, msg.info.origin.position.y
        
        # 获取机器人实时位置用于计算距离得分
        rx, ry = self.get_current_pose()
        if rx is None: return None

        # 将栅格地图转换为 OpenCV 格式 (0:障碍, 127:未知, 255:自由)
        data_np = np.array(msg.data).reshape((h, w))
        img = np.full((h, w), 127, dtype=np.uint8)
        img[data_np == 0] = 255
        img[data_np > 0] = 0

        # 图像处理提取边界线：寻找自由区边缘且紧邻未知区的点
        free_mask = cv2.inRange(img, 250, 255)
        unknown_mask = cv2.inRange(img, 120, 135)
        dilated_free = cv2.dilate(free_mask, np.ones((3,3), np.uint8), iterations=1)
        frontier_mask = cv2.bitwise_and(dilated_free, unknown_mask)

        # 连通域聚类：将离散的边界点聚集为块，并计算其中心点
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(frontier_mask)
        best_goal = None
        max_score = -float('inf')

        for i in range(1, num_labels):
            if stats[i, cv2.CC_STAT_AREA] < 10: continue # 忽略面积过小的噪点边界
            
            cx, cy = centroids[i]
            
            # --- 重要：坐标系转换修复 ---
            # 物理世界 X = 像素 X * 分辨率 + 原点 X
            # 物理世界 Y = (图像高 - 像素 Y - 1) * 分辨率 + 原点 Y (修复镜像问题)
            wx_raw = cx * res + ox
            wy_raw = (h - cy - 1) * res + oy

            # 黑名单过滤：如果该点距离之前的失败点太近，则跳过
            if any(math.dist((wx_raw, wy_raw), f) < 0.7 for f in self.failed_goals):
                continue

            # 评分公式：Utility = 边界面积 - 距离权重 * 机器人到该点的距离
            dist = math.sqrt((wx_raw - rx)**2 + (wy_raw - ry)**2)
            if dist < self.MIN_GOAL_DISTANCE:
                continue 
            
            # score = stats[i, cv2.CC_STAT_AREA] - (dist * 2.2)
            score = stats[i, cv2.CC_STAT_AREA] * 10 - dist  # 面积权重 ×10

            if score > max_score:
                # --- 安全退避逻辑 (Vector Back-off) ---
                # 为了防止导航目标点刚好落在未知区或墙上，我们计算机器人到边界的向量
                # 将目标点沿着这个向量向机器人方向回缩一段距离，确保目标在“已探测到的安全区”
                angle = math.atan2(wy_raw - ry, wx_raw - rx)
                wx_safe = wx_raw - self.SAFE_OFFSET * math.cos(angle)
                wy_safe = wy_raw - self.SAFE_OFFSET * math.sin(angle)
                
                # ✅ 新增：检查退避后的点是否在 costmap 安全区
                if not self._is_costmap_safe(wx_safe, wy_safe, safe_threshold=80):
                    continue  # 跳过这个不安全的目标点
                
                max_score = score
                best_goal = (wx_safe, wy_safe, angle)
        
        return best_goal

    # ---------------- 任务执行逻辑 ----------------
    def save_current_map(self):
        self.get_logger().info(f"正在保存地图...")
        if not self.save_map_cli.wait_for_service(timeout_sec=2.0):
            return

        req = SaveMap.Request()
        req.map_url = self.MAP_SAVE_PATH
        # 使用同步调用在独立线程中是安全的，或者坚持用异步但确保 loop 能够处理
        self.save_map_cli.call_async(req)

    def wait_for_nav2_ready(self):
        """
        手动检查 Nav2 核心 Action Server 是否就绪，而不依赖 AMCL
        """
        self.get_logger().info("正在等待 Nav2 核心控制器 (controller_server)...")
        
        # 1. 等待最关键的导航 Action Server
        # 这是 Nav2 执行移动的核心接口
        while not self.navigator.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("Nav2 导航服务尚未启动，继续等待...")
            if not rclpy.ok():
                return False
                
        # 2. (可选) 等待地图话题有数据发布
        self.get_logger().info("检测到导航服务，正在等待 SLAM 发布初始地图...")
        while self.map_msg is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if not rclpy.ok():
                return False
                
        self.get_logger().info("Nav2 与地图环境已就绪！")
        return True

    def exploration_loop(self):
        """探索主线程状态机"""
        
        # 预热：等待定位数据（TF map -> base_link）生效
        while rclpy.ok():
            rx, ry = self.get_current_pose()
            if rx is not None and self.wait_for_nav2_ready():
                break
            self.get_logger().info("等待机器人定位 / Nav2 就绪...")
            time.sleep(1.0)

        while rclpy.ok():
            # 获取地图统计数据
            unknown_ratio, known_count = self.get_unknown_ratio()
            self.get_logger().info(f"当前地图未知区域占比: {unknown_ratio:.2%},{known_count} 已知像素点")
            if known_count < 500: 
                self.get_logger().warn(f"SLAM 未初始化或地图太小 (已知像素: {known_count})，等待中...", once=True)
                time.sleep(1.0)
                continue
            if self.nav_status == 'IDLE':
                # 1. 计算当前最优边界点
                target = self.get_best_frontier()
                if target:
                    self.no_frontier_count = 0 # 重置结束计数器
                    wx, wy, yaw = target
                    # 发送目标位姿
                    self.send_nav_goal(self._make_pose(wx, wy, yaw))
                    self.nav_start_time = time.time()
                    self.current_goal = (wx, wy)
                    self.get_logger().info(f"✅ 新目标点确定: ({wx:.2f}, {wy:.2f}) | 机器人位置: ({rx:.2f}, {ry:.2f})")
                else:
                    # 双重判定：
                    # 1. 找不到显著边界
                    # 2. 或者未知区域已经非常少（例如仅剩 3%）
                    if target is None or unknown_ratio < self.UNKNOWN_THRESHOLD:
                        self.no_frontier_count += 1
                        if self.no_frontier_count >= self.FINISH_THRESHOLD:
                            self.get_logger().info("判定依据：未知区域比例达标或边界消失。")
                            break
                    time.sleep(2.0)
                    continue

            # 2. 监控当前导航任务
            if self.nav_status == 'IDLE':
                time.sleep(1.0) # 冷却防止频繁计算
                
            # 3. 超时强制处理
            elif self.goal_handle and (time.time() - self.nav_start_time) > self.NAV_TIMEOUT:
                self.get_logger().warning(f"⏰ 导航超时 ({self.NAV_TIMEOUT}s) - 目标点: ({self.current_goal[0]:.2f}, {self.current_goal[1]:.2f})")
                self.goal_handle.cancel_goal_async()
                with self.nav_lock:
                    self.nav_status = 'IDLE'
                self.failed_goals.append(self.current_goal)
            
            time.sleep(0.5)

        # --- 探索结束后的收尾工作 ---
        
        # 第一步：保存当前地图
        self.save_current_map()
        
        # 第二步：回到机器人起始坐标点 (0,0)
        self.get_logger().info("探索完成，正在指令机器人回到起始点...")
        self.send_nav_goal(self._make_pose(self.start_pose_x, self.start_pose_y, 0.0))
        
        # 等待回航结束
        while rclpy.ok() and self.nav_status != 'IDLE':
            time.sleep(1.0)
        
        self.get_logger().info("任务结束：地图已保存，机器人已停稳。")

    def get_unknown_ratio(self):
        """计算未知区域占比"""
        data = np.array(self.map_msg.data)
        # 只统计有效范围内的点（忽略地图边缘可能存在的巨大空白区，如果需要）
        unknown = np.count_nonzero(data == -1)
        known = np.count_nonzero(data != -1)
        if (unknown + known) == 0: return 1.0
        return unknown / (unknown + known), known

    def _make_pose(self, x, y, yaw):
        """快捷生成 PoseStamped 消息"""
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = x
        p.pose.position.y = y
        # 将角度转为四元数 Z/W
        p.pose.orientation.z = math.sin(yaw/2)
        p.pose.orientation.w = math.cos(yaw/2)
        return p
    
    def send_nav_goal(self, pose: PoseStamped):
        goal = NavigateToPose.Goal()
        goal.pose = pose
        with self.nav_lock:
            self.nav_status = 'NAVIGATING'
        self.nav_start_time = time.time()

        self.goal_future = self.navigator.send_goal_async(
            goal,
            feedback_callback=self._feedback_cb
        )
        self.goal_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        self.goal_handle = future.result()

        if not self.goal_handle.accepted:
            self.get_logger().warn('导航目标被拒绝')
            with self.nav_lock:
                self.nav_status = 'IDLE'
            return
        self.get_logger().info("✅ 导航目标已接受，开始规划路径")
        self.result_future = self.goal_handle.get_result_async()
        self.result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        status = future.result().status
        with self.nav_lock:
            self.nav_status = 'IDLE'

        if status != 4:  # STATUS_SUCCEEDED = 4
            self.get_logger().warn('导航失败，加入黑名单')
            self.failed_goals.append(self.current_goal)
            # time.sleep(3.0)
        else:
            self.get_logger().info("✅ 导航成功到达目标点")

    def _feedback_cb(self, feedback_msg):
        pass




def main():
    rclpy.init()
    node = FinalExploreNode()
    
    # 使用多线程执行器
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin() # 使用 executor 替代 rclpy.spin
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()