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
        self.last_pose = None
        self.last_progress_time = 0.0
        self.stuck_threshold_dist = 0.05  # 5厘米
        self.stuck_threshold_time = 10.0   # 10秒

        # ---- 状态 ----
        self.map = None
        self.scan = None
        self.goal_handle = None
        self.current_goal = None

        self.goal_start_time = None

        # ---- 参数（工程可调）----
        self.frontier_min_size = 5

        self.failed_goals = {}  # (x,y) -> time

        # --- 服务客户端：调用 map_server 保存地图 ---
        self.save_map_cli = self.create_client(
            SaveMap,
            '/map_saver_server/save_map'
        )
        self.map_url = "/home/ros_user/ros2_ws/my_map"
        self.map_save_timer = self.create_timer(60, self.save_current_map)

        self.recovery_timer = None

        # 初始化状态
        self.cancel_goal()
        self.cmd_vel_pub.publish(Twist())

        # --- 组装大脑 ---
        self.bt_root = Selector(self, [
            # 0. 正在倒车中：直接锁定，不看后续逻辑
            ConditionIsRecovering(self),
            # 优先级 1: 异常处理 (卡死或超时) -> 触发自救
            Sequence(self, [
                Selector(self, [ConditionIsStuck(self), ConditionIsExploreTimeout(self)]),
                ActionRecovery(self)
            ]),

            # 优先级 2: 状态维持 -> 如果正在导航，直接返回 SUCCESS/RUNNING，拦截下方的找点逻辑
            ConditionIsExploring(self),

            # 优先级 3: 任务发起 -> 只有上面都不满足（不卡死且没任务），才执行探索
            ActionExplore(self)
        ])

        self.create_timer(3.0, self.on_tick)

        self.get_logger().info("🚀 Explorer node started")


    # ================= ROS Callbacks =================

    def map_cb(self, msg):
        self.map = msg

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
        req.free_thresh = 0.25
        req.occupied_thresh = 0.65
        
        # 使用异步调用
        future = self.save_map_cli.call_async(req)
        return future

    def on_tick(self):
        """每秒一次的逻辑脉搏"""
        if self.map is None: return
        # 彻底抛弃 if self.state == ...
        # 行为树会根据 tick() 的结果自动决定当前该干什么
        self.bt_root.tick()

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
        for r in range(1, h - 1):
            for c in range(1, w - 1):
                if data[r, c] != -1:
                    continue
                if (data[r+1, c] == 0 or data[r-1, c] == 0 or
                    data[r, c+1] == 0 or data[r, c-1] == 0):
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
    
    def score_frontiers(self, frontiers):
        pose = self.get_robot_pose()
        if pose is None:
            self.get_logger().warn("[TF] 无法获取机器人位姿")
            return []
        
        rx, ry, ryaw = pose
        scored = []
        now = time.time()

        # --- 性能优化：将高耗时操作移出循环 ---
        res = self.map.info.resolution
        # 只转换一次 numpy 数组
        map_data = np.array(self.map.data).reshape(self.map.info.height, self.map.info.width)
        h, w = self.map.info.height, self.map.info.width

        for f in frontiers:
            cx, cy = f["centroid"]
            size = f["size"]
            
            # 1️⃣ 基础物理距离 (单位: 米)
            dist = math.hypot(cx - rx, cy - ry)
            dist_cost = 1.0 * dist 

            # 核心改进：防止原地打转的“近距离惩罚”
            # 逻辑：如果目标点距离机器人小于 1.0 米，视为“无意义目标”，强制加 10 米惩罚
            proximity_penalty = 0.0
            if dist < 1.0:
                # 距离越近，惩罚越重（阶梯式或固定值）
                proximity_penalty = 10.0 * (1.0 - dist) 
                # self.get_logger().debug(f"目标太近({dist:.2f}m)，已施加避让惩罚")
            
            # 2️⃣ 方向代价 (等效距离)
            # 逻辑：掉头(pi)相当于多跑 2 米。转换率约 0.63m/rad
            heading = math.atan2(cy - ry, cx - rx)
            heading_cost = abs(angle_diff(heading, ryaw))
            heading_penalty = 0.63 * heading_cost 

            # 3️⃣ 风险惩罚 (等效距离)
            # 逻辑：risk 是 0~1。如果目标点很挤(risk=0.5)，相当于多跑 5 米绕路
            risk = self.estimate_risk(cx, cy)
            risk_penalty = 10.0 * risk

            # 4️⃣ 未知区域奖励 (等效距离 - 负分)
            # 逻辑：发现 1㎡ 未知区域，相当于心理距离缩短 2 米
            r_idx, c_idx = self.world_to_index(cx, cy)
            win = int(1.0 / res / 2)
            # 使用 numpy 切片快速计算区域
            r0, r1 = max(0, r_idx-win), min(h, r_idx+win)
            c0, c1 = max(0, c_idx-win), min(w, c_idx+win)
            unk_count = np.sum(map_data[r0:r1, c0:c1] == -1)
            unk_area = unk_count * (res ** 2)
            unk_reward = 2.0 * min(unk_area, 1.5) # 封顶奖励 3 米

            # 5️⃣ 面积收益 (等效距离 - 负分)
            # 逻辑：每 100 个像素的 size 奖励 0.5 米
            size_reward = 0.005 * size

            # 6️⃣ 失败记录惩罚 (等效距离)
            failure_penalty = 0.0
            for (fr, fc), timestamp in self.failed_goals.items():
                if now - timestamp < 60.0:
                    fx, fy = self.index_to_world(fr, fc)
                    d_fail = math.hypot(cx - fx, cy - fy)
                    if d_fail < 2.0:
                        failure_penalty += 5.0 * (2.0 - d_fail)

            # 7️⃣ 穿墙检查 (等效距离)
            # 逻辑：一旦穿墙，等效距离增加 50 米，使其几乎不可能被选中
            is_clear = self.is_line_of_sight_clear((rx, ry), (cx, cy))
            wall_penalty = 0.0 if is_clear else 50.0

            # --- 最终评分汇总 ---
            # 基础代价 + 各种惩罚 - 各种奖励
            score = (dist_cost + proximity_penalty + heading_penalty + risk_penalty + 
                    failure_penalty + wall_penalty - size_reward - unk_reward)

            self.get_logger().info(
                f"[CANDIDATE] {cx:.1f},{cy:.1f} | Dist: {dist:.1f}m | "
                f"Wall: {'OK' if is_clear else 'BLOCKED'} | Score: {score:.2f}"
            )

            scored.append((score, (cx, cy), size))

        # 分数越低越优先
        scored.sort(key=lambda x: x[0])
        return scored


    # ================= Risk / Collision =================
    # 判断起点终点是否穿越障碍
    def is_line_of_sight_clear(self, start_pos, end_pos):
        """
        检查起点到终点的连线是否穿过障碍物。
        start_pos, end_pos: (x, y) 世界坐标
        """
        if self.map is None:
            return False
        
        # 1. 转换坐标为栅格索引
        r0, c0 = self.world_to_index(start_pos[0], start_pos[1])
        r1, c1 = self.world_to_index(end_pos[0], end_pos[1])
        
        # 2. 采样连线上的点（根据距离动态决定采样点数）
        dist_px = math.hypot(r1 - r0, c1 - c0)
        if dist_px < 1: return True
        
        num_samples = int(dist_px) # 每像素采样一次
        rs = np.linspace(r0, r1, num_samples).astype(int)
        cs = np.linspace(c0, c1, num_samples).astype(int)
        
        # 3. 检查地图数据
        data = np.array(self.map.data).reshape(self.map.info.height, self.map.info.width)
        for i in range(num_samples):
            r, c = rs[i], cs[i]
            # 如果路径上存在确定的障碍物 (> 50)
            if data[r, c] > 50:
                return False
        return True

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

        # 3. 记录该区域为失败区域，短时间内不再尝试
        if self.current_goal:
            self.failed_goals[self.current_goal] = time.time()
        # 1. 停止 Nav2 任务
        self.cancel_goal()

        # 关键修复：重置进度监控，给自救留出时间
        self.last_pose = None 
        self.last_progress_time = self.get_clock().now().nanoseconds / 1e9

        # 2. 直接发布速度指令（阻塞式或定时器式）
        # 这里为了简单演示用循环，实际建议用定时器
        self.drive_manually_non_blocking(vx=-0.1, duration=5.0) # 后退 2 秒，速度 -0.1 m/s
        

    def check_motion_status(self):
        """
        判断小车是否真的在移动
        """
        now = self.get_clock().now().nanoseconds / 1e9

        # 修复：如果没有任务，或者任务刚刚开始（不到5秒），不判定为卡死
        if not self.goal_handle or (now - self.goal_start_time < 5.0):
            self.last_progress_time = now # 重置计时，防止误判
            return True
        
        # 1. 获取机器人当前的位姿 (x, y, yaw)
        current_pose = self.get_robot_pose()
        if current_pose is None:
            self.get_logger().error("无法获取位姿，放弃后退自救")
            return True
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
        # 如果已有定时器正在运行，先清理
        if self.recovery_timer:
            return

        start_time = self.get_clock().now()
        end_time = start_time + Duration(seconds=duration)
        
        def timer_callback():
            if self.get_clock().now() < end_time:
                msg = Twist()
                msg.linear.x = vx
                msg.angular.z = wz
                self.cmd_vel_pub.publish(msg)
            else:
                self.cmd_vel_pub.publish(Twist()) # 停止
                self.recovery_timer.cancel()
                self.destroy_timer(self.recovery_timer)
                self.recovery_timer = None
                self.get_logger().info("✅ 自救完成")
                self.reset_state() # 统一重置
                
        self.recovery_timer = self.create_timer(0.1, timer_callback)

    def do_explore(self):
        frontiers = self.find_frontiers()
        self.get_logger().info(f"[FRONTIER] clusters={len(frontiers)}")

        if not frontiers:
            self.get_logger().warn("[FRONTIER] none found")
            return False

        # 前沿点打分，分数越低优先级约高
        scored = self.score_frontiers(frontiers)

        for score, cent, size in scored:
            grid_r, grid_c = self.world_to_index(cent[0], cent[1])
            key = (int(grid_r), int(grid_c)) # 显式转 int 确保 key 稳定
            if key in self.failed_goals and time.time() - self.failed_goals[key] < 30:
                self.get_logger().debug(f"[SKIP] failed frontier {cent}")
                continue

            self.send_goal(cent)
            return True
        
        return False
        
    def send_goal(self, pos, need_yaw= False):
        self.get_logger().info(f"[NAV] send goal {pos}")

        self.goal_start_time = time.time() # 记录开始时间

        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = pos[0]
        ps.pose.position.y = pos[1]
        ps.pose.position.z = 0.0

        ps.pose.orientation.x = 0.0
        ps.pose.orientation.y = 0.0
        ps.pose.orientation.w = 1.0
        if need_yaw:
            # 1. 获取当前位姿，用于计算“指向目标”的角度
            current_pose = self.get_robot_pose()
            # 计算朝向：如果能获取当前位姿，就计算指向目标的角度；否则默认不转向
            target_yaw = 0.0
            if current_pose is not None:
                curr_x, curr_y, curr_yaw = current_pose
                # 计算从当前点到目标点的向量夹角
                target_yaw = math.atan2(pos[1] - curr_y, pos[0] - curr_x)
            
            ps.pose.orientation.z = math.sin(target_yaw / 2.0)
            ps.pose.orientation.w = math.cos(target_yaw / 2.0)
            self.get_logger().info(f"[NAV] 目标点: ({pos[0]:.2f}, {pos[1]:.2f}), 规划朝向: {target_yaw:.2f} rad")

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
        try:
            result = future.result()
            status = result.status
        except Exception as e:
            self.get_logger().error(f"无法获取 Action 结果: {e}")
            self.reset_state()
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("🎯 到达目标点")
            # 成功到达，可以考虑清理该点附近的失败记录（如果有）
        
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("❌ 导航中止（可能撞墙或规划失败）")
            if self.current_goal:
                self.failed_goals[self.current_goal] = time.time()

        elif status == GoalStatus.STATUS_CANCELED:
            # 💡 重要：抢占导致的任务取消不计入失败
            self.get_logger().info("🔄 任务被抢占/取消")
        
        else:
            self.get_logger().warn(f"❓ 任务结束，状态码: {status}")

        # 无论如何，重置状态以便下一轮循环
        self.reset_state()

    def reset_state(self):
        """清理所有状态位"""
        self.goal_handle = None
        self.current_goal = None
        self.last_pose = None # 重置卡死监控起始点

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


#################################### base ####################################
class NodeStatus(Enum):
    SUCCESS = 1
    FAILURE = 2
    RUNNING = 3



class BTNode:
    def __init__(self, context: Explorer):
        self.ctx = context  # 指向 Explorer 实例
    def tick(self) -> NodeStatus:
        raise NotImplementedError

class Selector(BTNode):
    """Fallback 逻辑：只要有一个子节点不返回 FAILURE，就停止并返回该状态"""
    def __init__(self, context, children):
        super().__init__(context)
        self.children = children
    def tick(self):
        for child in self.children:
            status = child.tick()
            if status != NodeStatus.FAILURE: return status
        return NodeStatus.FAILURE

class Sequence(BTNode):
    """逻辑与：所有子节点必须返回 SUCCESS"""
    def __init__(self, context, children):
        super().__init__(context)
        self.children = children
    def tick(self):
        for child in self.children:
            status = child.tick()
            if status != NodeStatus.SUCCESS: return status
        return NodeStatus.SUCCESS
    
##################################### condition #################################################
class ConditionIsStuck(BTNode):
    def tick(self):
        # 询问躯体：是否满足物理卡死阈值；通过一定时间内的位移判断
        return NodeStatus.SUCCESS if not self.ctx.check_motion_status() else NodeStatus.FAILURE

class ConditionIsRecovering(BTNode):
    def tick(self):
        # 如果倒车定时器存在，返回 SUCCESS 锁定自救分支
        return NodeStatus.SUCCESS if self.ctx.recovery_timer else NodeStatus.FAILURE
    
class ConditionIsExploreTimeout(BTNode):
    def tick(self):
        # 询问躯体：是否执行超时
        return NodeStatus.SUCCESS if (
            self.ctx.goal_start_time  and 
            (time.time() - self.ctx.goal_start_time > 60.0)
        ) else NodeStatus.FAILURE
        
class ConditionIsExploring(BTNode):
    def tick(self):
        # 询问躯体：是否正在探索，通过句柄判定
        return NodeStatus.SUCCESS if self.ctx.goal_handle else NodeStatus.FAILURE
##################################### action #####################################################
class ActionRecovery(BTNode):
    def tick(self):
        # 如果还没开始倒车，则触发
        if not self.ctx.recovery_timer:
            self.ctx.execute_hard_recovery()
        return NodeStatus.RUNNING

class ActionExplore(BTNode):
    def tick(self):
        if self.ctx.do_explore():
            return NodeStatus.RUNNING
        return NodeStatus.FAILURE
        

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