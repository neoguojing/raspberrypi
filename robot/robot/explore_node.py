#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import SaveMap  # 用于调用地图保存服务
from action_msgs.msg import GoalStatus
from std_msgs.msg import String

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

        # --- 参数化：方便在运行时微调 ---
        self.declare_parameter('save_map_service', '/map_saver/save_map')
        self.declare_parameter('initial_spin_duration', 6.0)
        self.declare_parameter('stuck_timeout', 30.0)
        self.declare_parameter('stuck_min_move', 0.05)
        self.declare_parameter('recovery_backoff_time', 1.0)
        self.declare_parameter('recovery_rotate_time', 2.0)
        self.declare_parameter('recovery_backoff_speed', 0.08)
        self.declare_parameter('recovery_rotate_speed', 0.6)

        # 探索策略相关参数
        self.declare_parameter('safe_offset', 0.45)
        self.declare_parameter('nav_timeout', 60.0)
        self.declare_parameter('finish_threshold', 5)
        self.declare_parameter('unknown_threshold', 0.05)
        self.declare_parameter('map_save_path', 'auto_map_result')
        self.declare_parameter('min_goal_distance', 0.2)
        self.declare_parameter('min_frontier_area_m2', 0.2)
        self.declare_parameter('costmap_safe_threshold', 120)
        self.declare_parameter('min_goal_retry_interval', 10.0)

        # 初始化 action client
        self.navigator = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 状态变量
        self.map_msg = None
        self.global_costmap = None
        self.failed_goals = deque(maxlen=200)
        self.failed_goal_timestamps = {}  # 记录失败时间用于重试间隔
        self.no_frontier_count = 0

        # 服务客户端
        self.save_map_cli = self.create_client(SaveMap, self.get_parameter('save_map_service').value)

        # 订阅
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.costmap_sub = self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.costmap_callback, 10)

        # cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # debug publishers：用于定位问题
        self.debug_goal_pub = self.create_publisher(PoseStamped, '/explore/debug_goal', 10)
        self.debug_info_pub = self.create_publisher(String, '/explore/debug_info', 10)

        # timer for start
        self.timer = self.create_timer(1.0, self._start_logic)
        self.started = False

        self.nav_lock = threading.Lock()
        self.goal_handle = None
        self.result_future = None
        self.nav_status = 'IDLE'

        # motion tracking
        self.last_pose = (None, None)
        self.last_moved_time = time.time()

        # keep track of nav start time
        self.nav_start_time = None

    # ---------------- TF and pose helpers ----------------
    def get_current_pose(self):
        try:
            # 使用可用的最新时间查询 transform
            now = rclpy.time.Time()
            # 尝试较短超时，避免阻塞
            t = self.tf_buffer.lookup_transform('map', 'base_link', now, timeout=rclpy.duration.Duration(seconds=0.2))
            x = t.transform.translation.x
            y = t.transform.translation.y
            self._update_motion_track(x, y)
            return x, y
        except TransformException:
            # 退而求其次，尝试获取最近可用的变换
            try:
                t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time(seconds=0))
                x = t.transform.translation.x
                y = t.transform.translation.y
                self._update_motion_track(x, y)
                return x, y
            except Exception:
                return None, None
        except Exception:
            return None, None

    def _update_motion_track(self, x, y):
        if x is None or y is None:
            return
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

    # ---------------- subscriptions ----------------
    def costmap_callback(self, msg: OccupancyGrid):
        self.global_costmap = msg

    def map_callback(self, msg: OccupancyGrid):
        self.map_msg = msg

    # ---------------- costmap safety ----------------
    def _is_costmap_safe(self, wx, wy, safe_threshold=None):
        if safe_threshold is None:
            safe_threshold = self.get_parameter('costmap_safe_threshold').value
        if self.global_costmap is None:
            return False
        ox = self.global_costmap.info.origin.position.x
        oy = self.global_costmap.info.origin.position.y
        res = self.global_costmap.info.resolution
        width = self.global_costmap.info.width
        height = self.global_costmap.info.height

        mx = int((wx - ox) / res)
        my = int((wy - oy) / res)

        if mx < 0 or mx >= width or my < 0 or my >= height:
            return False

        index = my * width + mx
        if index < 0 or index >= len(self.global_costmap.data):
            return False

        cost = self.global_costmap.data[index]
        # -1 代表未知，保守处理为不安全
        if cost < 0:
            return False
        # lethal or inflation high cost
        return cost < safe_threshold

    # ---------------- frontier detection ----------------
    def get_best_frontier(self):
        msg = self.map_msg
        if msg is None:
            self.get_logger().debug('get_best_frontier: map_msg is None')
            return None

        w, h = msg.info.width, msg.info.height
        res = msg.info.resolution
        ox, oy = msg.info.origin.position.x, msg.info.origin.position.y

        rx, ry = self.get_current_pose()
        if rx is None:
            self.get_logger().warn('get_best_frontier: robot pose unavailable')
            return None

        data_np = np.array(msg.data).reshape((h, w))
        # map: -1 unknown, 0 free, >0 occupied
        img = np.full((h, w), 127, dtype=np.uint8)
        img[data_np == 0] = 255
        img[data_np > 0] = 0

        free_mask = cv2.inRange(img, 250, 255)
        unknown_mask = cv2.inRange(img, 120, 135)

        dilated_free = cv2.dilate(free_mask, np.ones((3, 3), np.uint8), iterations=1)
        frontier_mask = cv2.bitwise_and(dilated_free, unknown_mask)

        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(frontier_mask)

        best_goal = None
        max_score = -float('inf')
        # convert min area from meters^2 to pixels
        min_area_m2 = self.get_parameter('min_frontier_area_m2').value
        min_area_pixels = max(4, int(min_area_m2 / (res * res)))

        self.get_logger().info(f'🔍 扫描边界块数量: {max(0, num_labels-1)}')

        for i in range(1, num_labels):
            area = stats[i, cv2.CC_STAT_AREA]
            if area < min_area_pixels:
                continue

            cx, cy = centroids[i]
            # centroids are in pixel coordinates with origin top-left
            # compute center of cell (add 0.5) to get more accurate world position
            wx_raw = (cx + 0.5) * res + ox
            wy_raw = (h - (cy + 0.5)) * res + oy  # keep original Y flip approach; depends on map origin

            dist_to_robot = math.hypot(wx_raw - rx, wy_raw - ry)

            # skip recently failed goals (allow retry after interval)
            retry_interval = self.get_parameter('min_goal_retry_interval').value
            recently_failed = False
            for fx, fy in self.failed_goals:
                if math.hypot(wx_raw - fx, wy_raw - fy) < 0.7:
                    ts = self.failed_goal_timestamps.get((fx, fy), 0)
                    if time.time() - ts < retry_interval:
                        recently_failed = True
                        break

            if recently_failed:
                continue

            # scoring
            score = area * 2.0 - dist_to_robot * 1.5

            # dynamic safe offset
            angle = math.atan2(wy_raw - ry, wx_raw - rx)
            safe_offset = min(self.get_parameter('safe_offset').value, dist_to_robot * 0.4)
            wx_safe = wx_raw - safe_offset * math.cos(angle)
            wy_safe = wy_raw - safe_offset * math.sin(angle)

            dist_safe = math.hypot(wx_safe - rx, wy_safe - ry)
            if dist_safe < 0.25 and num_labels > 2:
                # skip very near targets if there are other options
                continue

            adjusted_score = score
            if dist_safe < self.get_parameter('min_goal_distance').value:
                adjusted_score = score - (self.get_parameter('min_goal_distance').value - dist_safe) * 5.0

            if not self._is_costmap_safe(wx_safe, wy_safe, safe_threshold=self.get_parameter('costmap_safe_threshold').value):
                self.get_logger().debug(f'点 ({wx_safe:.2f},{wy_safe:.2f}) 代价过高，跳过')
                continue

            if adjusted_score > max_score:
                max_score = adjusted_score
                best_goal = (wx_safe, wy_safe, angle)

        if best_goal:
            # publish debug goal for visualization
            p = PoseStamped()
            p.header.frame_id = 'map'
            p.header.stamp = self.get_clock().now().to_msg()
            p.pose.position.x = best_goal[0]
            p.pose.position.y = best_goal[1]
            p.pose.orientation.z = math.sin(best_goal[2] / 2)
            p.pose.orientation.w = math.cos(best_goal[2] / 2)
            self.debug_goal_pub.publish(p)
            self.get_logger().info(f'🎯 选定目标: {best_goal[0]:.2f}, {best_goal[1]:.2f} (得分: {max_score:.2f})')
        else:
            self.get_logger().warn('⚠️ 本轮未找到符合安全条件的有效边界')

        return best_goal

    # ---------------- recovery ----------------
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
        stop = Twist()
        self.cmd_vel_pub.publish(stop)

    def recovery_behavior(self):
        self.get_logger().warn('🆘 触发恢复动作：后退 + 旋转扫描')
        backoff_time = self.get_parameter('recovery_backoff_time').value
        backoff_speed = -abs(self.get_parameter('recovery_backoff_speed').value)
        rotate_time = self.get_parameter('recovery_rotate_time').value
        rotate_speed = self.get_parameter('recovery_rotate_speed').value

        self._publish_twist_for(linear_x=backoff_speed, duration=backoff_time)
        time.sleep(0.15)
        direction = random.choice([-1.0, 1.0])
        self._publish_twist_for(angular_z=direction * rotate_speed, duration=rotate_time)
        time.sleep(0.05)

    # ---------------- nav2 readiness ----------------
    def wait_for_nav2_ready(self):
        self.get_logger().info('正在等待 Nav2 核心控制器...')
        timeout_count = 0
        while not self.navigator.wait_for_server(timeout_sec=1.0):
            timeout_count += 1
            self.get_logger().info('Nav2 导航服务尚未启动，继续等待...')
            if not rclpy.ok():
                return False
            if timeout_count > 120:
                self.get_logger().error('等待 Nav2 超时')
                return False
        # 等待地图数据
        while self.map_msg is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if not rclpy.ok():
                return False
        self.get_logger().info('Nav2 与地图环境已就绪！')
        return True

    # ---------------- exploration loop ----------------
    def exploration_loop(self):
        self.get_logger().info('等待系统预热：同步定位与导航服务...')
        while rclpy.ok():
            rx, ry = self.get_current_pose()
            if rx is not None and self.wait_for_nav2_ready():
                break
            time.sleep(1.0)

        initial_spin = self.get_parameter('initial_spin_duration').value
        if initial_spin and initial_spin > 0:
            self.get_logger().info(f'🔄 启动扫描：原地旋转 {initial_spin}s')
            self._publish_twist_for(angular_z=0.6, duration=initial_spin)

        self.get_logger().info('🚀 探索正式开始！')

        stuck_timeout = self.get_parameter('stuck_timeout').value

        try:
            while rclpy.ok():
                if self.map_msg is None:
                    time.sleep(0.2)
                    continue

                unknown_ratio, known = self.get_unknown_ratio()
                progress = 1.0 - unknown_ratio
                info = f'进度:{progress:.2%} 已知像素:{known} 失败点:{len(self.failed_goals)}'
                self.debug_info_pub.publish(String(data=info))
                self.get_logger().info(info)

                if self.nav_status == 'IDLE':
                    target = self.get_best_frontier()
                    if target:
                        self.no_frontier_count = 0
                        wx, wy, yaw = target
                        self.current_goal = (wx, wy)
                        self.send_nav_goal(self._make_pose(wx, wy, yaw))
                        self.get_logger().info(f'📍 前往新边界: ({wx:.2f}, {wy:.2f})')
                        time.sleep(0.5)
                    else:
                        if unknown_ratio < self.get_parameter('unknown_threshold').value:
                            self.no_frontier_count += 1
                            self.get_logger().info(f'🧐 未发现新边界，进度已达标 ({progress:.2%})，确认中 {self.no_frontier_count}/{self.get_parameter("finish_threshold").value}')
                            if self.no_frontier_count >= self.get_parameter('finish_threshold').value:
                                self.get_logger().info('✅ 地图探索完整，准备保存并回航！')
                                break
                        else:
                            self.get_logger().warn('⚠️ 进度不达标但暂无有效路径，尝试自救')
                            if len(self.failed_goals) > 0:
                                self.get_logger().info('🧹 清空黑名单，准备重新扫描不可达区域...')
                                self.failed_goals.clear()
                                self.failed_goal_timestamps.clear()
                            self._publish_twist_for(angular_z=0.6, duration=2.0)
                            time.sleep(0.5)
                            continue

                elif self.nav_status == 'NAVIGATING':
                    elapsed_time = time.time() - (self.nav_start_time or time.time())
                    time_since_moved = time.time() - self.last_moved_time
                    if time_since_moved > stuck_timeout:
                        self.get_logger().warning(f'⛔ 检测到可能卡住 (未移动 {time_since_moved:.1f}s)，触发恢复')
                        if self.goal_handle:
                            try:
                                self.goal_handle.cancel_goal_async()
                            except Exception:
                                pass
                        self.recovery_behavior()
                        with self.nav_lock:
                            self.nav_status = 'IDLE'
                        if hasattr(self, 'current_goal'):
                            self.failed_goals.append(self.current_goal)
                            self.failed_goal_timestamps[self.current_goal] = time.time()
                        continue

                    if elapsed_time > self.get_parameter('nav_timeout').value:
                        self.get_logger().warning(f'⏰ 导航超时 ({self.get_parameter("nav_timeout").value}s)，放弃当前点。')
                        if self.goal_handle:
                            try:
                                self.goal_handle.cancel_goal_async()
                            except Exception:
                                pass
                        with self.nav_lock:
                            self.nav_status = 'IDLE'
                        if hasattr(self, 'current_goal'):
                            self.failed_goals.append(self.current_goal)
                            self.failed_goal_timestamps[self.current_goal] = time.time()

                time.sleep(0.4)
        except Exception as e:
            self.get_logger().error(f'探索循环异常: {e}')

        # 收尾
        self.get_logger().info('🏁 正在执行收尾流程...')
        try:
            self.save_current_map()
        except Exception as e:
            self.get_logger().error(f'地图保存失败: {e}')

        if hasattr(self, 'start_pose_x'):
            self.get_logger().info(f'🏠 正在回到起点: ({self.start_pose_x:.2f}, {self.start_pose_y:.2f})')
            self.send_nav_goal(self._make_pose(self.start_pose_x, self.start_pose_y, 0.0))
            while rclpy.ok():
                if self.nav_status == 'IDLE':
                    break
                time.sleep(1.0)

        self.get_logger().info('🎮 任务全部完成，节点准备退出。')

    def get_unknown_ratio(self):
        data = np.array(self.map_msg.data)
        unknown = np.count_nonzero(data == -1)
        free = np.count_nonzero(data == 0)
        occupied = np.count_nonzero(data > 0)
        known = free + occupied
        if known == 0:
            return 1.0, 0
        return unknown / (unknown + known), known

    def _make_pose(self, x, y, yaw):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.orientation.z = math.sin(yaw/2)
        p.pose.orientation.w = math.cos(yaw/2)
        return p

    # ---------------- navigation ----------------
    def send_nav_goal(self, pose: PoseStamped):
        goal = NavigateToPose.Goal()
        goal.pose = pose
        with self.nav_lock:
            self.nav_status = 'NAVIGATING'
        self.nav_start_time = time.time()

        self.goal_future = self.navigator.send_goal_async(goal, feedback_callback=self._feedback_cb)
        self.goal_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        try:
            self.goal_handle = future.result()
            if not self.goal_handle.accepted:
                self.get_logger().warn('导航目标被拒绝')
                with self.nav_lock:
                    self.nav_status = 'IDLE'
                return
            self.get_logger().info('✅ 导航目标已接受，开始规划路径')
            self.result_future = self.goal_handle.get_result_async()
            self.result_future.add_done_callback(self._result_cb)
        except Exception as e:
            self.get_logger().error(f'目标响应回调异常: {e}')
            with self.nav_lock:
                self.nav_status = 'IDLE'

    def _result_cb(self, future):
        try:
            result = future.result()
            status = result.status
            with self.nav_lock:
                self.nav_status = 'IDLE'

            if status != GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().warn(f'导航失败，状态码: {status}，加入黑名单')
                if hasattr(self, 'current_goal'):
                    self.failed_goals.append(self.current_goal)
                    self.failed_goal_timestamps[self.current_goal] = time.time()
            else:
                self.get_logger().info('✅ 导航成功到达目标点')
        except Exception as e:
            self.get_logger().error(f'结果回调异常: {e}')
            with self.nav_lock:
                self.nav_status = 'IDLE'

    def _feedback_cb(self, feedback_msg):
        # 可扩展：将局部规划信息输出到 debug topic
        pass

    def save_current_map(self):
        self.get_logger().info('正在保存地图...')
        if not self.save_map_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('保存地图服务不可用')
            return
        req = SaveMap.Request()
        req.map_url = self.get_parameter('map_save_path').value
        future = self.save_map_cli.call_async(req)
        # 不阻塞，添加回调以记录结果
        def _save_cb(fut):
            try:
                res = fut.result()
                self.get_logger().info('地图保存请求已提交')
            except Exception as e:
                self.get_logger().error(f'地图保存调用异常: {e}')
        future.add_done_callback(_save_cb)


def main():
    rclpy.init()
    node = FinalExploreNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
