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
        self.declare_parameter('recovery_backoff_speed', 0.08)
        self.declare_parameter('recovery_rotate_speed', 0.6)

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
            # 失败时尝试查找最近的一次有效变换
            try:
                t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time(seconds=0))
                x = t.transform.translation.x
                y = t.transform.translation.y
                self._update_motion_track(x, y)
                return x, y
            except:
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

    def map_callback(self, msg):
        """地图回调：不断更新本地地图快照"""
        self.map_msg = msg

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

    def _is_costmap_safe(self, wx, wy, safe_threshold=60):
        """
        检查世界坐标 (wx, wy) 在全局代价地图中是否安全。
        :param wx, wy: 世界坐标 (m)
        :param safe_threshold: cost 阈值，低于此值认为安全（推荐 60～120）
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
        改进版：引入动态退避与最小距离校验，解决小车原地不动的问题。
        """
        msg = self.map_msg
        if msg is None:
            self.get_logger().warn("get_best_frontier(): map_msg is None")
            return None

        # 地图基本参数
        w, h = msg.info.width, msg.info.height
        res = msg.info.resolution
        ox, oy = msg.info.origin.position.x, msg.info.origin.position.y

        # 获取机器人当前位姿
        rx, ry = self.get_current_pose()
        if rx is None:
            self.get_logger().warn("get_best_frontier(): robot pose unavailable")
            return None

        # OccupancyGrid -> numpy 转换与图像处理
        data_np = np.array(msg.data).reshape((h, w))
        img = np.full((h, w), 127, dtype=np.uint8)
        img[data_np == 0] = 255
        img[data_np > 0] = 0

        free_mask = cv2.inRange(img, 250, 255)
        unknown_mask = cv2.inRange(img, 120, 135)

        dilated_free = cv2.dilate(free_mask, np.ones((3, 3), np.uint8), iterations=1)
        frontier_mask = cv2.bitwise_and(dilated_free, unknown_mask)

        # 连通域分析
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(frontier_mask)

        best_goal = None
        max_score = -float('inf')
        min_area_pixels = max(4, int(0.2 / res))

        self.get_logger().info(f"🔍 扫描边界块数量: {num_labels-1}")

        for i in range(1, num_labels):
            area = stats[i, cv2.CC_STAT_AREA]
            if area < min_area_pixels:
                continue

            cx, cy = centroids[i]
            wx_raw = cx * res + ox
            wy_raw = (h - cy - 1) * res + oy

            # 1. 计算原始距离
            dist_to_robot = math.hypot(wx_raw - rx, wy_raw - ry)

            # 2. 黑名单过滤
            if any(math.hypot(wx_raw - fx, wy_raw - fy) < 0.7 for fx, fy in self.failed_goals):
                self.get_logger().warn(f"黑名单过滤!!!")
                continue

            # 3. 评分函数：面积优先，距离惩罚
            score = area * 2.0 - dist_to_robot * 1.5

            if score > max_score:
                angle = math.atan2(wy_raw - ry, wx_raw - rx)

                # --- 核心改进：动态退避逻辑 ---
                # 如果点很近，退避距离不能超过原始距离的一半
                dynamic_offset = min(self.SAFE_OFFSET, dist_to_robot * 0.4)
                
                wx_safe = wx_raw - dynamic_offset * math.cos(angle)
                wy_safe = wy_raw - dynamic_offset * math.sin(angle)

                # --- 核心改进：防止“原地完成” ---
                dist_safe = math.hypot(wx_safe - rx, wy_safe - ry)
                if dist_safe < 0.25:
                    self.get_logger().warn(f"跳过过近目标: dist={dist_safe:.2f}m")
                    # 如果这是唯一的块，即便近也要试一下，不直接 continue
                    if num_labels > 2:
                        continue

                # 强制最小目标距离，防止导航认为已到达
                if dist_safe < self.MIN_GOAL_DISTANCE:
                    self.get_logger().info(f"目标太近 ( {dist_safe:.2f}m )，尝试选更远的块")
                    # 不立即丢弃，但优先级降低
                    adjusted_score = score - (self.MIN_GOAL_DISTANCE - dist_safe) * 5.0
                else:
                    adjusted_score = score

                # 4. 代价地图安全性校验
                # 使用稍微严格的阈值，防止蹭墙
                if not self._is_costmap_safe(wx_safe, wy_safe, safe_threshold=120):
                    self.get_logger().warn(f"点 ({wx_safe:.2f}, {wy_safe:.2f}) 代价过高，放弃")
                    continue

                max_score = adjusted_score
                best_goal = (wx_safe, wy_safe, angle)

        if best_goal:
            self.get_logger().info(f"🎯 选定目标: {best_goal[0]:.2f}, {best_goal[1]:.2f} (得分: {max_score:.2f})")
        else:
            self.get_logger().warn("⚠️ 本轮未找到符合安全条件的有效边界")

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
                    if self.goal_handle:
                        try:
                            self.goal_handle.cancel_goal_async()
                        except Exception:
                            pass

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

        # STATUS_SUCCEEDED = 4
        if status != 4:
            self.get_logger().warn('导航失败，加入黑名单')
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
