#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import SaveMap  # 用于调用地图保存服务

import numpy as np
import cv2
import threading
import time
import math
import random
from collections import deque

# 导入 TF 相关库
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class FinalExploreNode(Node):
    def __init__(self):
        super().__init__('final_explore_node')

        # --- 可调参数 ---
        self.declare_parameter('save_map_service', '/map_saver/save_map')
        self.declare_parameter('initial_spin_duration', 6.0)  # 启动时原地旋转搜周围
        self.declare_parameter('stuck_timeout', 30)         # 导航时无移动判定为卡住
        self.declare_parameter('stuck_min_move', 0.05)      # 判定为“移动”的最小距离 (m)
        self.declare_parameter('recovery_backoff_time', 1.0)
        self.declare_parameter('recovery_rotate_time', 2.0)
        self.declare_parameter('recovery_backoff_speed', 0.1)
        self.declare_parameter('recovery_rotate_speed', 0.1)

        # 初始化 Nav2 简单导航接口 (ActionClient)
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
        self.FINISH_THRESHOLD = 5     # 终止判定：连续 FINISH_THRESHOLD 次扫描不到有效边界则认为地图已扫完
        self.UNKNOWN_THRESHOLD = 0.05  # 如果未知区域比例低于 5%，则认为完成
        self.MAP_SAVE_PATH = "auto_map_result" # 保存的文件名前缀
        self.MIN_GOAL_DISTANCE = 0.2  # 至少 80cm 远

        # --- 状态控制 ---
        self.map_msg = None           # 实时地图缓存
        self.failed_goals = deque(maxlen=120) # 失败点黑名单，防止机器人反复尝试不可达区域
        self.no_frontier_count = 0    # 空边界计数器

        # --- 服务客户端：调用 map_server 保存地图 ---
        self.save_map_cli = self.create_client(
            SaveMap,
            self.get_parameter('save_map_service').value
        )

        # 订阅 SLAM 发布的地图
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

        # 延迟启动计时器：给 SLAM 和 Nav2 预留初始化时间
        self.timer = self.create_timer(1.0, self._start_logic)
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

        # 发布 cmd_vel 用于本地恢复动作（原地旋转、后退）
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 运动跟踪，用于判定是否卡住
        self.last_pose = (None, None)
        self.last_moved_time = time.time()

    def get_current_pose(self):
        try:
            # 尝试使用最新时间点查询 TF
            now = rclpy.time.Time()
            t = self.tf_buffer.lookup_transform(
                'map', 
                'base_link', 
                now, 
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            x = t.transform.translation.x
            y = t.transform.translation.y
            # 更新最后移动时间
            self._update_motion_track(x, y)
            return x, y
        except Exception:
            self.get_logger().warn("get_current_pose failed")
            # 失败时尝试查找最近的一次有效变换
            try:
                t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time(seconds=0))
                x = t.transform.translation.x
                y = t.transform.translation.y
                self._update_motion_track(x, y)
                return x, y
            except:
                self.get_logger().error("get_current_pose failed")
                return None, None

    def _update_motion_track(self, x, y):
        # 如果last_pose尚未初始化，直接赋值
        if self.last_pose[0] is None:
            self.last_pose = (x, y)
            self.last_moved_time = time.time()
            return

        lx, ly = self.last_pose
        dx = math.hypot(x - lx, y - ly)
        stuck_min_move = self.get_parameter('stuck_min_move').value
        if dx >= stuck_min_move:
            self.last_moved_time = time.time()
            self.last_pose = (x, y)

    def costmap_callback(self, msg):
        """接收全局代价地图，用于目标点安全性验证"""
        self.global_costmap = msg
        # 使用类变量确保只打印一次，避免日志刷屏
        if not hasattr(self, '_costmap_logged'):
            info = msg.info
            data_np = np.array(msg.data)
            
            # 计算代价统计
            max_cost = np.max(data_np)
            min_cost = np.min(data_np)
            avg_cost = np.mean(data_np)
            # 统计高代价点（通常 > 100 表示靠近障碍物）
            high_cost_count = np.count_nonzero(data_np > 100)
            lethal_count = np.count_nonzero(data_np >= 253)

            self.get_logger().info("==== 全局代价地图 (Global Costmap) 结构解析 ====")
            self.get_logger().info(f"数据类型: {type(msg.data)} | 长度: {len(msg.data)}")
            self.get_logger().info(f"地图尺寸: {info.width} x {info.height} (总像素: {info.width * info.height})")
            self.get_logger().info(f"分辨率: {info.resolution:.4f} m/pixel")
            self.get_logger().info(f"地图原点: x={info.origin.position.x:.2f}, y={info.origin.position.y:.2f}")
            self.get_logger().info(f"代价值范围: [{min_cost} ~ {max_cost}] | 平均值: {avg_cost:.2f}")
            self.get_logger().info(f"危险点统计: 较高代价(>100): {high_cost_count} | 致命障碍(>=253): {lethal_count}")
            self.get_logger().info("============================================")
            
            self._costmap_logged = True

    def map_callback(self, msg):
        """地图回调：不断更新本地地图快照"""
        self.map_msg = msg
        # 仅在收到地图的前几次打印格式信息，避免刷屏
        if not hasattr(self, '_map_logged_once'):
            data_np = np.array(msg.data)
            unique_values = np.unique(data_np)
            
            self.get_logger().info("--- 地图格式校验 ---")
            self.get_logger().info(f"地图分辨率: {msg.info.resolution:.3f} m/pixel")
            self.get_logger().info(f"地图尺寸: {msg.info.width}x{msg.info.height}")
            self.get_logger().info(f"原始数据数值范围: {unique_values}")
            
            # 统计分布
            unknown_count = np.count_nonzero(data_np == -1)
            free_count = np.count_nonzero(data_np == 0)
            obs_count = np.count_nonzero(data_np > 0)
            
            self.get_logger().info(f"像素统计 -> 未知(-1): {unknown_count}, 自由(0): {free_count}, 障碍(>0): {obs_count}")
            self.get_logger().info("-------------------")
            self._map_logged_once = True

    def _start_logic(self):
        """启动逻辑：仅执行一次，开启独立的计算线程"""
        if self.started or self.map_msg is None:
            return
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

    def _is_costmap_safe(self, wx, wy, safe_threshold=100):
        if self.global_costmap is None:
            self.get_logger().warn("安全检查失败：全局代价地图尚未收到")
            return False

        info = self.global_costmap.info
        ox, oy = info.origin.position.x, info.origin.position.y
        res = info.resolution
        w, h = info.width, info.height

        mx = int((wx - ox) / res)
        my = int((wy - oy) / res)

        # 检查是否在地图数组范围内
        if mx < 0 or mx >= w or my < 0 or my >= h:
            self.get_logger().warn(f"点 ({wx:.2f}, {wy:.2f}) 超出代价地图边界")
            return False

        # 检查中心点及周边小范围区域
        check_radius = max(1, int(0.15 / res)) 
        for dx in range(-check_radius, check_radius + 1):
            for dy in range(-check_radius, check_radius + 1):
                curr_mx, curr_my = mx + dx, my + dy
                
                if curr_mx < 0 or curr_mx >= w or curr_my < 0 or curr_my >= h:
                    continue
                    
                index = curr_my * w + curr_mx
                cost = self.global_costmap.data[index]

                # 关键判定日志
                if cost == -1 or cost == 255:
                    self.get_logger().debug(f"坐标({wx:.2f}, {wy:.2f}) 不安全: 落在未知区域(cost={cost})")
                    return False
                if cost >= 253:
                    self.get_logger().debug(f"坐标({wx:.2f}, {wy:.2f}) 不安全: 触碰致命障碍(cost={cost})")
                    return False
                if cost > safe_threshold:
                    self.get_logger().debug(f"坐标({wx:.2f}, {wy:.2f}) 不安全: 代价过高({cost} > {safe_threshold})")
                    return False

        return True

    # ---------------- 核心算法：边界提取与评估 ----------------
    def get_best_frontier(self):
        msg = self.map_msg
        if msg is None: return None

        w, h = msg.info.width, msg.info.height
        res = msg.info.resolution
        ox, oy = msg.info.origin.position.x, msg.info.origin.position.y
        rx, ry = self.get_current_pose()
        if rx is None: return None

        # 1. 预处理地图：区分自由、障碍、未知
        data_np = np.array(msg.data).reshape((h, w))
        
        # 建立掩码
        img = np.full((h, w), 127, dtype=np.uint8)  # 默认未知
        img[data_np == 0] = 255                    # 自由区域
        img[data_np > 0] = 0                       # 障碍物区域

        # --- 优化 A：对障碍物进行膨胀，防止选点离墙太近 ---
        kernel = np.ones((int(0.3/res), int(0.3/res)), np.uint8) # 30cm 膨胀
        obs_mask = cv2.inRange(img, 0, 10)
        dilated_obs = cv2.dilate(obs_mask, kernel, iterations=1)
        
        # 在自由区域中扣除掉靠近障碍物的部分
        safe_free_mask = cv2.bitwise_and(cv2.inRange(img, 250, 255), cv2.bitwise_not(dilated_obs))

        # 2. 提取边界 (Frontier)
        # 边界定义：在安全自由区内，且邻域内有未知区域
        unknown_mask = cv2.inRange(img, 120, 135)
        dilated_safe_free = cv2.dilate(safe_free_mask, np.ones((3,3), np.uint8), iterations=1)
        frontier_mask = cv2.bitwise_and(dilated_safe_free, unknown_mask)

        # 3. 连通域分析
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(frontier_mask)
        
        best_goal = None
        max_score = -float('inf')
        min_area_pixels = max(5, int(0.15 / res)) # 最小边界尺寸要求

        for i in range(1, num_labels):
            area = stats[i, cv2.CC_STAT_AREA]
            if area < min_area_pixels: continue

            # 边界中心的世界坐标
            cx, cy = centroids[i]
            # wx_raw = cx * res + ox
            # wy_raw = (h - cy - 1) * res + oy
            wx_raw = ox + (cx + 0.5) * res
            wy_raw = oy + (cy + 0.5) * res

            dist_to_robot = math.hypot(wx_raw - rx, wy_raw - ry)
            
            # 过滤过近或黑名单点
            if dist_to_robot < 0.4: continue
            if any(math.hypot(wx_raw - fx, wy_raw - fy) < 0.6 for fx, fy in self.failed_goals):
                continue

            # 计算朝向 (指向未知区域中心)
            angle = math.atan2(wy_raw - ry, wx_raw - rx)

            # --- 优化 B：安全的退避位置计算 ---
            # 尝试在机器人与边界点的连线上，找一个距离边界 0.45m 的点
            offset = self.SAFE_OFFSET 
            if dist_to_robot < offset + 0.2:
                offset = dist_to_robot * 0.5 # 距离太近时缩小退避距离

            wx_goal = wx_raw - offset * math.cos(angle)
            wy_goal = wy_raw - offset * math.sin(angle)

            # --- 优化 C：多重安全性检查 ---
            # 1. 检查目标点是否落在障碍物膨胀区
            if not self._is_costmap_safe(wx_goal, wy_goal, safe_threshold=100):
                # 如果不安全，尝试微调角度或缩小偏移量
                continue

            # 评分：面积大优先，距离中等优先（避免总是跑最远或者最近）
            # 使用高斯型距离评分，鼓励机器人去 2.0m - 5.0m 左右的点
            dist_score = 10.0 / (1.0 + abs(dist_to_robot - 3.0)) 
            score = area * 1.0 + dist_score * 5.0

            if score > max_score:
                max_score = score
                best_goal = (wx_goal, wy_goal, angle)

        return best_goal

    # ---------------- 恢复动作 ----------------
    def _publish_twist_for(self, linear_x=0.0, angular_z=0.0, duration=0.5):
        t_end = time.time() + duration
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        rate_hz = 10
        period = 1.0 / rate_hz
        while time.time() < t_end and rclpy.ok():
            self.cmd_vel_pub.publish(twist)
            time.sleep(period)
        # 停止
        stop = Twist()
        self.cmd_vel_pub.publish(stop)

    def recovery_behavior(self):
        """当检测到卡住或局部规划失败时调用：后退 + 原地旋转，尝试重新建立可行路径"""
        with self.nav_lock:
            if self.nav_status != 'IDLE':
                # 确保导航空闲后再发 cmd_vel
                self.get_logger().info("等待导航空闲以执行本地恢复动作...")
                start = time.time()
                while self.nav_status != 'IDLE' and time.time() - start < 1.0:
                    time.sleep(0.05)

        self.get_logger().warn("🆘 触发恢复动作：后退 + 旋转扫描")
        backoff_time = self.get_parameter('recovery_backoff_time').value
        backoff_speed = -abs(self.get_parameter('recovery_backoff_speed').value)
        rotate_time = self.get_parameter('recovery_rotate_time').value
        rotate_speed = self.get_parameter('recovery_rotate_speed').value

        # 1) 轻微后退
        self._publish_twist_for(linear_x=backoff_speed, duration=backoff_time)
        time.sleep(0.2)
        # 2) 随机方向原地旋转（扩大感知）
        direction = random.choice([-1.0, 1.0])
        self._publish_twist_for(angular_z=direction * rotate_speed, duration=rotate_time)
        time.sleep(0.1)

    # ---------------- 任务执行逻辑 ----------------
    def save_current_map(self):
        self.get_logger().info(f"正在保存地图...")
        if not self.save_map_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("保存地图服务不可用")
            return

        req = SaveMap.Request()
        req.map_url = self.MAP_SAVE_PATH
        # 使用异步调用
        self.save_map_cli.call_async(req)

    def wait_for_nav2_ready(self):
        """
        手动检查 Nav2 核心 Action Server 是否就绪，而不依赖 AMCL
        """
        self.get_logger().info("正在等待 Nav2 核心控制器 (controller_server)...")
        # 1. 等待最关键的导航 Action Server
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
        """
        改进后的探索主线程状态机：
        1. 强化结束判定：必须【找不到点】且【比例达标】才退出。
        2. 引入自救逻辑：找不到点但比例不达标时，清空黑名单重试。
        3. 状态监控：实时打印进度。
        4. 卡住检测与恢复动作（后退 + 旋转扫描）。
        """

        # --- 预热阶段 ---
        self.get_logger().info("等待系统预热：正在同步定位与导航服务...")
        while rclpy.ok():
            rx, ry = self.get_current_pose()
            if rx is not None and self.wait_for_nav2_ready():
                break
            time.sleep(1.0)

        # 启动时做一次原地旋转，帮助 SLAM 快速获取周围观测
        initial_spin = self.get_parameter('initial_spin_duration').value
        if initial_spin and initial_spin > 0:
            self.get_logger().info(f"🔄 启动扫描：原地旋转 {initial_spin}s")
            self._publish_twist_for(angular_z=0.6, duration=initial_spin)

        self.get_logger().info("🚀 探索正式开始！")

        stuck_timeout = self.get_parameter('stuck_timeout').value

        while rclpy.ok():
            # 获取地图统计数据
            if self.map_msg is None:
                time.sleep(0.2)
                continue

            unknown_ratio, known_count = self.get_unknown_ratio()
            progress = 1.0 - unknown_ratio

            # 每隔一段时间打印一次进度
            self.get_logger().info(f"📊 探索进度: {progress:.2%} | 已知像素: {known_count}")

            # 1. 如果当前没有导航任务，尝试寻找新目标
            if self.nav_status == 'IDLE':
                target = self.get_best_frontier()

                if target:
                    # 发现有效目标点
                    self.no_frontier_count = 0 
                    wx, wy, yaw = target
                    self.current_goal = (wx, wy)
                    self.send_nav_goal(self._make_pose(wx, wy, yaw))
                    self.get_logger().info(f"📍 前往新边界: ({wx:.2f}, {wy:.2f})")
                    time.sleep(1.0) # 给状态更新留一点时间

                else:
                    # --- 关键：判定是否真的结束 ---
                    if unknown_ratio < self.UNKNOWN_THRESHOLD:
                        # 情况 A：地图已经扫得差不多了
                        self.no_frontier_count += 1
                        self.get_logger().info(f"🧐 未发现新边界，进度已达标 ({progress:.2%})，确认中 {self.no_frontier_count}/{self.FINISH_THRESHOLD}")
                        
                        if self.no_frontier_count >= self.FINISH_THRESHOLD:
                            self.get_logger().info("✅ 地图探索完整，准备保存并回航！")
                            break
                    else:
                        # 情况 B：地图没扫完但没点可去了（被黑名单过滤或路径不通）
                        self.get_logger().warn("⚠️ 进度不达标但暂无有效路径！执行自救逻辑...")
                        
                        # 自救动作 1：清空黑名单，给之前失败的点一个重试的机会
                        if len(self.failed_goals) > 0:
                            self.get_logger().info("🧹 清空黑名单，准备重新扫描不可达区域...")
                            self.failed_goals.clear()
                        
                        # 自救动作 2：原地旋转再试
                        self._publish_twist_for(angular_z=0.6, duration=2.0)
                        time.sleep(1.0)
                        continue

            # 2. 如果正在导航中，检查超时与卡住情况
            elif self.nav_status == 'NAVIGATING':
                elapsed_time = time.time() - self.nav_start_time

                # 卡住检测：若在一段时间内机器人没有实际位移，触发恢复
                time_since_moved = time.time() - self.last_moved_time
                if time_since_moved > stuck_timeout:
                    self.get_logger().warning(f"⛔ 检测到机器人可能卡住 (未移动 {time_since_moved:.1f}s)，触发恢复")
                    # 取消当前导航目标
                    if self.goal_handle is not None:
                        try:
                            cancel_future = self.goal_handle.cancel_goal_async()
                            # 等待短时间让 action 确认取消
                            timeout = 1.0
                            start_c = time.time()
                            while not cancel_future.done() and time.time() - start_c < timeout:
                                time.sleep(0.05)
                        except Exception as e:
                            self.get_logger().warn(f"cancel goal exception: {e}")

                    # 执行恢复动作
                    self.recovery_behavior()

                    # 将该点记为失败并回到空闲状态
                    with self.nav_lock:
                        self.nav_status = 'IDLE'

                    if hasattr(self, 'current_goal'):
                        self.failed_goals.append(self.current_goal)
                    continue

                if elapsed_time > self.NAV_TIMEOUT:
                    self.get_logger().warning(f"⏰ 导航超时 ({self.NAV_TIMEOUT}s)，放弃当前点。")
                    if self.goal_handle:
                        try:
                            self.goal_handle.cancel_goal_async()
                        except Exception:
                            pass
                    with self.nav_lock:
                        self.nav_status = 'IDLE'
                    self.failed_goals.append(self.current_goal)

            # 循环频率控制
            time.sleep(0.4)

        # --- 任务收尾 ---
        self.get_logger().info("🏁 正在执行收尾流程...")
        
        # 保存地图
        try:
            self.save_current_map()
        except Exception as e:
            self.get_logger().error(f"地图保存失败: {e}")

        # 回航
        if hasattr(self, 'start_pose_x'):
            self.get_logger().info(f"🏠 正在回到起点: ({self.start_pose_x:.2f}, {self.start_pose_y:.2f})")
            self.send_nav_goal(self._make_pose(self.start_pose_x, self.start_pose_y, 0.0))
            
            # 等待机器人到家
            while rclpy.ok():
                if self.nav_status == 'IDLE':
                    break
                time.sleep(1.0)

        self.get_logger().info("🎮 任务全部完成，节点准备退出。")
        
    def get_unknown_ratio(self):
        data = np.array(self.map_msg.data)
        unknown = np.count_nonzero(data == -1)
        free = np.count_nonzero(data == 0)
        occupied = np.count_nonzero(data > 0)
        
        known = free + occupied
        if known == 0: return 1.0, 0
        
        # 返回 未知 / (未知 + 已知)
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
            # 避免在导航中再次发送
            if self.nav_status == 'NAVIGATING':
                self.get_logger().warn("尝试在正在导航时发送目标，已忽略")
                return
            self.nav_status = 'NAVIGATING'
            self.nav_start_time = time.time()

        self.goal_future = self.navigator.send_goal_async(
            goal,
            feedback_callback=self._feedback_cb
        )
        self.goal_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"_goal_response_cb exception: {e}")
            with self.nav_lock:
                self.nav_status = 'IDLE'
            return

        with self.nav_lock:
            self.goal_handle = goal_handle

        if not goal_handle.accepted:
            self.get_logger().warn('导航目标被拒绝')
            with self.nav_lock:
                self.nav_status = 'IDLE'
            return

        self.get_logger().info("✅ 导航目标已接受，开始规划路径")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)
        with self.nav_lock:
            self.result_future = result_future

    def _result_cb(self, future):
        try:
            result = future.result()
            status = result.status
        except Exception as e:
            self.get_logger().error(f"_result_cb exception: {e}")
            status = None

        with self.nav_lock:
            self.nav_status = 'IDLE'
            self.goal_handle = None
            self.result_future = None

        # STATUS_SUCCEEDED = 4
        if status != 4:
            self.get_logger().warn('导航失败或被取消，加入黑名单（若有当前目标）')
            if hasattr(self, 'current_goal'):
                self.failed_goals.append(self.current_goal)
        else:
            self.get_logger().info("✅ 导航成功到达目标点")

    def _feedback_cb(self, feedback_msg):
        # 可在这里检查局部规划状态 / 里程计等信息
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
