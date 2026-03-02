#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Pose
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus # 需要导入状态常量
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import SaveMap
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import Twist
import numpy as np
import math
import time
from enum import Enum

class RobotState(Enum):
    IDLE = 0          # 待命（找点中）
    NAVIGATING = 1    # 导航中（监控进度）
    RECOVERING = 2    # 自救中（底层接管）

# ================= 工具函数 =================

def yaw_from_quat(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )

def angle_diff(a, b):
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d

# ================= Explorer Node =================

class Explorer(Node):
    def __init__(self):
        super().__init__('explorer_node')

        # ---- ROS IO ----
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_cb, 1
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/seg/scan', self.scan_cb, 10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 监控进度用的状态变量
        self.state = RobotState.IDLE
        self.last_pose = None
        self.last_progress_time = 0.0
        self.stuck_threshold_dist = 0.05  # 5厘米
        self.stuck_threshold_time = 5.0   # 5秒

        # ---- 状态 ----
        self.map = None
        self.scan = None
        self.goal_handle = None
        self.current_goal = None
        self.goal_start_time = 0.0

        # ---- 参数（工程可调）----
        self.frontier_min_size = 5

        self.recompute_interval = 3.0
        self.last_compute = 0.0

        self.failed_goals = {}  # (x,y) -> time

        # --- 服务客户端：调用 map_server 保存地图 ---
        self.save_map_cli = self.create_client(
            SaveMap,
            '/map_saver_server/save_map'
        )
        self.map_url = "/home/ros_user/ros2_ws/my_map"
        self.map_save_timer = self.create_timer(30, self.save_current_map)
        # ---- 主循环 ----
        self.timer = self.create_timer(3, self.loop)
        self.get_logger().info("🚀 Explorer node started")

    # ================= ROS Callbacks =================

    def map_cb(self, msg):
        self.map = msg
        # self.get_logger().info(
        #     f"[MAP] received {msg.info.width}x{msg.info.height}, res={msg.info.resolution}"
        # )

    def scan_cb(self, msg):
        self.scan = msg

    # ================= 主循环 =================

    def save_current_map(self):
        self.get_logger().info(f"正在保存地图...")
        if not self.save_map_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("保存地图服务不可用")
            return

        req = SaveMap.Request()
        req.map_topic = "map"
        req.map_url = self.map_url
        req.image_format = "png"
        req.map_mode = "trinary"
        req.free_thresh = 0.0
        req.occupied_thresh = 0.0
        
        # 使用异步调用
        future = self.save_map_cli.call_async(req)
        return future

    def loop(self):
        if self.map is None:
            self.get_logger().debug("[WAIT] no map yet")
            return

        # self.validate_map_orientation()
        pose = self.get_robot_pose()
        if pose is None:
            self.get_logger().warn("[TF] cannot get robot pose")
            return

        self.cleanup_failed_goals()

        # 初始时全-1，主动触发前进1m
        if np.all(np.array(self.map.data) == -1):
            self.get_logger().info("[INIT] Map all unknown → move forward")
            self.drive_manually_non_blocking(vx=0.1,duration=1)
            return
        
        # 状态 A: 正在自救。不执行任何逻辑，直到自救完成切换回 IDLE
        if self.state == RobotState.RECOVERING:
            return

        # 状态 B: 正在导航。检查是否卡死。
        if self.state == RobotState.NAVIGATING:
            # 1. 检查物理停滞 (Stuck)
            moving = self.check_motion_status()
            # 2. 检查总耗时超时 (你的原逻辑: 60s)
            is_timeout = (time.time() - self.goal_start_time > 60.0)

            if not moving or is_timeout:
                reason = "停滞" if not moving else "超时"
                self.get_logger().error(f"[ALARM] 导航{reason}！切换至自救模式...")
                self.execute_hard_recovery() 
            return
    
        
        # 状态 C: 空闲。寻找新目标。
        if self.state == RobotState.IDLE:
            # 节流 frontier 计算
            now = time.time()
            if now - self.last_compute < self.recompute_interval:
                return
            self.last_compute = now

            frontiers = self.find_frontiers()
            self.get_logger().info(f"[FRONTIER] clusters={len(frontiers)}")

            if not frontiers:
                self.get_logger().warn("[FRONTIER] none found")
                return

            # 前沿点打分，分数越低优先级约高
            scored = self.score_frontiers(frontiers, pose)

            for score, cent, size in scored:
                grid_r, grid_c = self.world_to_index(cent[0], cent[1])
                key = (int(grid_r), int(grid_c)) # 显式转 int 确保 key 稳定
                if key in self.failed_goals and time.time() - self.failed_goals[key] < 30:
                    self.get_logger().debug(f"[SKIP] failed frontier {cent}")
                    continue

                self.send_goal(cent)
                return

    # ================= Pose =================

    def get_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.map.header.frame_id,
                'base_footprint',
                rclpy.time.Time()
            )
            x = t.transform.translation.x
            y = t.transform.translation.y
            yaw = yaw_from_quat(t.transform.rotation)
            return x, y, yaw
        except Exception:
            return None

    # ================= Frontier =================
    # frontier 是 未知区域的边缘，是机器人探索的目标点，而不是障碍点
    def find_frontiers(self):
        # ----------------------------
        # 将地图数据转换为 2D numpy 数组
        # occupancy grid 数据是一维数组，需要 reshape 成 (height, width)
        # ----------------------------
        data = np.array(self.map.data).reshape(
            self.map.info.height, self.map.info.width
        )
        h, w = data.shape

        # 创建一个np和data一样，初始都为false
        frontier = np.zeros_like(data, dtype=bool)
        # ----------------------------
        # 遍历地图格子，找到 frontier 点
        # frontier 条件：
        #   1. 当前格子未知 (data[r,c] == -1)
        #   2. 相邻的上下左右至少有一个空地 (0)
        # ----------------------------
        # for r in range(1, h - 1):
        #     for c in range(1, w - 1):
        #         if data[r, c] != -1:
        #             continue
        #         if (data[r+1, c] == 0 or data[r-1, c] == 0 or
        #             data[r, c+1] == 0 or data[r, c-1] == 0):
        #             frontier[r, c] = True
                    
        # === 核心修改：放宽 frontier 判定条件 ===
        for r in range(1, h - 1):
            for c in range(1, w - 1):
                # 1. 当前格子必须是未知区域
                if data[r, c] != -1:
                    continue
                
                # 2. 检查上下左右邻居
                neighbors = [data[r+1, c], data[r-1, c], data[r, c+1], data[r, c-1]]
                
                # 3. 改进逻辑：只要有一个邻居是“已知且非障碍”的
                # 这样即使地图上有噪声（如 5, 10, 20），只要没超过 50，都能触发探索
                is_frontier = False
                for n in neighbors:
                    if 0 <= n <= 50:  # 关键点：不再只盯着 0 看
                        is_frontier = True
                        break
                
                if is_frontier:
                    frontier[r, c] = True

        # ----------------------------
        # 聚类 frontier 点
        # 用 DFS 遍历相连的 frontier 点，将它们分为 cluster
        # ----------------------------
        visited = np.zeros_like(frontier, dtype=bool)
        clusters = []

        for r in range(h):
            for c in range(w):
                if frontier[r, c] and not visited[r, c]:
                    stack = [(r, c)]
                    visited[r, c] = True
                    cells = []

                    while stack:
                        rr, cc = stack.pop()
                        cells.append((rr, cc))
                        for dr, dc in [(1,0),(-1,0),(0,1),(0,-1)]:
                            nr, nc = rr+dr, cc+dc
                            if (0 <= nr < h and 0 <= nc < w and
                                frontier[nr, nc] and not visited[nr, nc]):
                                visited[nr, nc] = True
                                stack.append((nr, nc))

                    if len(cells) >= self.frontier_min_size:
                        ar = sum(p[0] for p in cells) / len(cells)
                        ac = sum(p[1] for p in cells) / len(cells)
                        mx, my = self.index_to_world(ar, ac)

                        # cells → 这个 cluster 的所有 frontier 点
                        # mx, my → cluster 的质心，作为探索目标
                        # len(cells) → cluster 的大小，可以用于评分（越大越值得去）
                        clusters.append({
                            "centroid": (mx, my),
                            "size": len(cells)
                        })
        return clusters
    
    def score_frontiers(self, frontiers, pose):
        """
        对 frontier 点集进行打分
        1. 距离：越近越优先
        2. 方向：和机器人当前朝向差越小越优先
        3. 尺寸：越大越优先（面积越大说明空间更大）
        4. 风险：障碍风险越小越优先
        5. 未知奖励：未知栅格越多越优先

        返回按 score 排序的列表 [(score, (cx, cy), size), ...]
        """
        rx, ry, ryaw = pose
        scored = []

        for f in frontiers:
            cx, cy = f["centroid"]
            size = f["size"]
            # 使用 log 压缩 size 的量级
            # +1 是为了防止 size 为 1 时 log 结果为 0
            size_factor = math.log(size + 1)

            # 1️⃣ 距离
            dist = math.hypot(cx - rx, cy - ry)

            # 2️⃣ 方向差
            heading = math.atan2(cy - ry, cx - rx)
            heading_cost = abs(angle_diff(heading, ryaw))

            # 3️⃣ 风险（只考虑障碍，未知区域不惩罚）
            risk = self.estimate_risk(cx, cy)

            # 4️⃣ 未知奖励：计算 frontier 附近未知栅格数量
            res = self.map.info.resolution
            data = np.array(self.map.data).reshape(self.map.info.height, self.map.info.width)
            h = self.map.info.height
            w = self.map.info.width
            r_idx, c_idx = self.world_to_index(cx, cy)
            window = int(1.0 / res / 2)  # 1m 窗口
            unk_count = 0
            for rr in range(max(0, r_idx-window), min(h, r_idx+window)):
                for cc in range(max(0, c_idx-window), min(w, c_idx+window)):
                    if data[rr, cc] == -1:
                        unk_count += 1

            # 组合打分公式
            score = (
                1.0 * dist +          # 距离权重
                1.5 * heading_cost -  # 方向权重
                2.0 * size_factor -           # 面积权重
                4.0 * (1-risk) -       # 越安全越优先，risk 越大扣分越多
                0.0                     # 可加其他惩罚项
            )
            # 加上未知奖励（未知越多分越低，前沿越优先）
            # 1. 计算窗口内未知的实际面积（单位：平方米）
            unk_area = unk_count * (res ** 2)

            # 2. 核心改动：给奖励设定上限 (例如最大奖励抵消 3 米距离)
            # 这样即使前方有一片大海般的未知区域，它也只会被视为“非常有价值”，而不会变成“无限价值”
            unk_bonus = min(unk_area, 1.5) # 1.5㎡ 以上的未知区域，奖励封顶
            score -= 2.0 * unk_bonus

            self.get_logger().debug(
                f"[FRONTIER-CAND] {cx:.2f},{cy:.2f} "
                f"dist={dist:.2f} size={size} risk={risk:.2f} unk={unk_count} score={score:.2f}"
            )

            scored.append((score, (cx, cy), size))

        # 按 score 从小到大排序（分数越低越优先）
        scored.sort(key=lambda x: x[0])
        return scored


    # ================= Risk / Collision =================
    # estimate_risk 就是计算一个点附近 1m 范围内有多少障碍和未知，越多 → 越危险。
    def estimate_risk(self, cx, cy, window=1.0):
        """
        计算给定点的风险值（只考虑障碍），使用障碍距离衰减。
        未知区域不被当作风险，探索奖励在 score_frontiers 中处理。
        
        参数:
            cx, cy: 世界坐标下的点
            window: 检查区域的边长（米）
        
        返回:
            risk: 0~1，越大表示越危险
        """
        res = self.map.info.resolution
        h = self.map.info.height
        w = self.map.info.width

        # 转换为栅格索引
        r, c = self.world_to_index(cx, cy)

        half = int(window / res / 2)
        risk = 0.0

        # 取窗口内数据
        data = np.array(self.map.data).reshape(h, w)

        for rr in range(max(0, r-half), min(h, r+half)):
            for cc in range(max(0, c-half), min(w, c+half)):
                v = data[rr, cc]
                if v > 50:  # 占用栅格才算风险
                    # 距离衰减：离目标越远，风险贡献越小
                    dist = math.hypot(rr - r, cc - c) * res
                    risk += math.exp(-dist / 0.5)  # 0.5m 是衰减长度，可调

        # 窗口内最大可能栅格数，用于归一化
        max_cells = (2*half)**2
        return min(risk / max_cells, 1.0)  # 归一化到 0~1
    ####################################recover########################################

    def execute_hard_recovery(self):
        self.get_logger().error("🚨 检测到物理卡死！启动强制底层脱困...")
        self.state = RobotState.RECOVERING

        # 1. 停止 Nav2 任务
        self.cancel_goal()
        # 2. 直接发布速度指令（阻塞式或定时器式）
        # 这里为了简单演示用循环，实际建议用定时器
        self.drive_manually_non_blocking(vx=-0.15)
        
        # 3. 记录该区域为失败区域，短时间内不再尝试
        if self.current_goal:
            self.failed_goals[self.current_goal] = time.time()

    def check_motion_status(self):
        """
        判断小车是否真的在移动
        """

        now = self.get_clock().now().nanoseconds / 1e9
        # 1. 获取机器人当前的位姿 (x, y, yaw)
        current_pose = self.get_robot_pose()
        if current_pose is None:
            self.get_logger().error("无法获取位姿，放弃后退自救")
            return
        curr_x, curr_y, _ = current_pose

        if self.last_pose is None:
            self.last_pose = (curr_x, curr_y)
            self.last_progress_time = now
            return True

        # 计算位移
        dist = math.hypot(curr_x - self.last_pose[0], curr_y - self.last_pose[1])
        
        # 如果位移超过阈值，更新进度时间
        if dist > self.stuck_threshold_dist:
            self.last_pose = (curr_x, curr_y)
            self.last_progress_time = now
            return True # 正在正常移动
        
        # 如果位移不足，且持续时间超过阈值 -> 判定为卡死
        if (now - self.last_progress_time) > self.stuck_threshold_time:
            return False # 判定为卡死
            
        return True

    # ================= Navigation =================

    def drive_manually_non_blocking(self, vx, wz=0.0, duration=1.5):
        """利用 Timer 实现非阻塞控制，不卡死节点"""
        start_time = self.get_clock().now()
        end_time = start_time + Duration(seconds=duration)
        
        def timer_callback():
            if self.get_clock().now() < end_time:
                msg = Twist()
                msg.linear.x = vx
                msg.angular.z = wz
                self.cmd_vel_pub.publish(msg)
            else:
                # 结束自救
                self.cmd_vel_pub.publish(Twist()) # 停止
                self.recovery_timer.cancel()
                self.state = RobotState.IDLE
                self.last_pose = None
                self.get_logger().info("✅ 自救动作完成，返回寻路状态")

        self.recovery_timer = self.create_timer(0.1, timer_callback)

    def send_goal(self, pos):
        self.get_logger().info(f"[NAV] send goal {pos}")
        self.state = RobotState.NAVIGATING

        self.goal_start_time = time.time() # 记录开始时间
        # # 1. 获取当前位姿，用于计算“指向目标”的角度
        # current_pose = self.get_robot_pose()
        
        # # 计算朝向：如果能获取当前位姿，就计算指向目标的角度；否则默认不转向
        # target_yaw = 0.0
        # if current_pose is not None:
        #     curr_x, curr_y, curr_yaw = current_pose
        #     # 计算从当前点到目标点的向量夹角
        #     target_yaw = math.atan2(pos[1] - curr_y, pos[0] - curr_x)
        
        # self.get_logger().info(f"[NAV] 目标点: ({pos[0]:.2f}, {pos[1]:.2f}), 规划朝向: {target_yaw:.2f} rad")

        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = pos[0]
        ps.pose.position.y = pos[1]
        ps.pose.position.z = 0.0

        ps.pose.orientation.x = 0.0
        ps.pose.orientation.y = 0.0
        # ps.pose.orientation.z = math.sin(target_yaw / 2.0)
        # ps.pose.orientation.w = math.cos(target_yaw / 2.0)
        ps.pose.orientation.w = 1.0

        r, c = self.world_to_index(pos[0], pos[1])
        self.current_goal = (int(r), int(c))

        goal = NavigateToPose.Goal()
        goal.pose = ps

        self.nav_client.wait_for_server()
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response)


    def goal_response(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().warn("[NAV] rejected")
            self.goal_handle = None
            return
        self.get_logger().info("[NAV] accepted")
        self.goal_handle.get_result_async().add_done_callback(self.goal_done)

    def goal_done(self, future):
        status = future.result().status
        # 获取状态描述以便调试
        status_msg = "UNKNOWN"
        if status == GoalStatus.STATUS_SUCCEEDED:
            status_msg = "SUCCEEDED"
        elif status == GoalStatus.STATUS_ABORTED:
            status_msg = "ABORTED"
            self.failed_goals[self.current_goal] = time.time()
        elif status == GoalStatus.STATUS_CANCELED:
            status_msg = "CANCELED"
            self.failed_goals[self.current_goal] = time.time()
        else:
            self.failed_goals[self.current_goal] = time.time()

        self.get_logger().info(f"[NAV] Goal finished with status: {status_msg} ({status})")
        self.state = RobotState.IDLE
        self.goal_handle = None
        self.current_goal = None

    def cancel_goal(self):
        if self.goal_handle:
            self.goal_handle.cancel_goal_async()
            # self.goal_handle = None


    def _map_info(self):
        if self.map is None:
            return (0.1, 0, 0, 0.0, 0.0) # 或者抛出异常
        return (self.map.info.resolution, self.map.info.width, self.map.info.height,
                self.map.info.origin.position.x, self.map.info.origin.position.y)
    
    def _get_map_value(self, r, c):
        if self.map is None:
            return -1
        data = np.array(self.map.data).reshape(self.map.info.height, self.map.info.width)
        r = np.clip(r, 0, self.map.info.height-1)
        c = np.clip(c, 0, self.map.info.width-1)
        return data[r, c] 


    def world_to_index(self, x, y):
        res, w, h, ox, oy = self._map_info()
        # 使用 floor 确保在负值区间也能正确映射到栅格
        c = math.floor((x - ox) / res)
        r = math.floor((y - oy) / res)
        # c = int((x - ox) / res)
        # r = h - 1 - int((y - oy) / res)
        
        # 使用 numpy.clip 或原生 min/max 限制在数组范围内
        r = max(0, min(r, h - 1))
        c = max(0, min(c, w - 1))
        return r, c
    
    def index_to_world(self, r, c):
        # r: row index (0..h-1), c: col index (0..w-1)
        res, w, h, ox, oy = self._map_info()
        # 假定 data 按 row-major 从 origin.y 向上增加（常见情况）
        x = ox + (c + 0.5) * res
        y = oy + (r + 0.5) * res

        # x = ox + (c + 0.5) * res
        # y = oy + (h - r - 0.5) * res

        return x, y
    
    def cleanup_failed_goals(self):
        """清理陈旧的失败记录，防止内存泄漏"""
        now = time.time()
        # 建议清理阈值设为比屏蔽时间（30s）略长，例如 60s
        expiry_threshold = 60.0
        
        # 使用 list() 包装 keys 是为了避免在遍历时修改字典导致报错
        expired_keys = [
            key for key, timestamp in self.failed_goals.items() 
            if now - timestamp > expiry_threshold
        ]
        
        for key in expired_keys:
            del self.failed_goals[key]
            
        if expired_keys:
            self.get_logger().info(f"已清理 {len(expired_keys)} 条过期的失败目标记录")
    
    def validate_map_orientation(self):
        """
        【极简调试】无激光雷达版本
        仅通过检查机器人自身位置和前方1米处的地图值来验证坐标转换。
        
        使用方法：
        1. 在仿真中将机器人移动到已知位置（例如：面对墙壁，距离1米）。
        2. 观察日志。
        3. 如果 "Front_1m" 显示 Free(0) 但前面明明是墙 -> h-r 逻辑错误。
        """
        if self.map is None:
            return

        pose = self.get_robot_pose()
        if pose is None:
            self.get_logger().warn("[CHECK] 无法获取机器人位姿 (TF 问题?)")
            return

        x, y, yaw = pose
        res, w, h, ox, oy = self._map_info()

        # 1. 检查机器人脚下 (应该是 Free 或 Unknown，绝不应该是 Occupied)
        r_self, c_self = self.world_to_index(x, y)
        val_self = self._get_map_value(r_self, c_self)
        
        # 2. 检查机器人正前方 1.0 米处
        check_dist = 1.0
        front_x = x + check_dist * math.cos(yaw)
        front_y = y + check_dist * math.sin(yaw)
        r_front, c_front = self.world_to_index(front_x, front_y)
        val_front = self._get_map_value(r_front, c_front)

        # 3. 检查机器人正后方 1.0 米处 (用于对比)
        back_x = x - check_dist * math.cos(yaw)
        back_y = y - check_dist * math.sin(yaw)
        r_back, c_back = self.world_to_index(back_x, back_y)
        val_back = self._get_map_value(r_back, c_back)

        # 格式化输出
        def status(v):
            if v == -1: return "Unknown(-1)"
            if v == 0:  return "Free(0)  "
            if v >= 100: return "WALL(100)"
            return f"Prob({v}) "

        self.get_logger().info("--- [MAP ORIENTATION CHECK] ---")
        self.get_logger().info(f"Robot Pose: x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.1f}°")
        self.get_logger().info(f"Self Grid : r={r_self}, c={c_self} -> {status(val_self)}")
        self.get_logger().info(f"Front Grid: r={r_front}, c={c_front} -> {status(val_front)}  (距离前方{check_dist}m)")
        self.get_logger().info(f"Back Grid : r={r_back}, c={c_back} -> {status(val_back)}  (距离后方{check_dist}m)")
        
        # 简单逻辑判断
        if val_self >= 100:
            self.get_logger().error("❌ 严重错误：机器人脚下显示为墙壁！(定位或地图原点错误)")
        elif val_front == 0 and val_back >= 100:
            self.get_logger().error("❌ 疑似错误：前方是空地，后方是墙？如果你正对着墙，说明 h-r 逻辑反了！")
        elif val_front >= 100 and val_back == 0:
            self.get_logger().info("✅ 看起来正常：前方检测到墙，后方是空地。")
        else:
            self.get_logger().info("ℹ️  周围都是空地或未知，请移动机器人到墙边再试。")
        self.get_logger().info("-------------------------------")

# ================= main =================

def main():
    rclpy.init()
    node = Explorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()