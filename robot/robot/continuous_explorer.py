#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Pose
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener
import numpy as np
import math
import time

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

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # ---- 状态 ----
        self.map = None
        self.scan = None
        self.goal_handle = None
        self.current_goal = None

        # ---- 参数（工程可调）----
        self.frontier_min_size = 3
        self.safe_dist = 0.6
        self.unknown_soft_limit = 10.0     # unknown 允许的最大前进距离
        self.laser_blind_frac = 0.9
        self.bad_frame_req = 3

        self.recompute_interval = 1.0
        self.last_compute = 0.0
        self.bad_frames = 0

        self.failed_goals = {}  # (x,y) -> time

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

    def loop(self):
        if self.map is None:
            self.get_logger().debug("[WAIT] no map yet")
            return

        pose = self.get_robot_pose()
        if pose is None:
            self.get_logger().warn("[TF] cannot get robot pose")
            return

        # 初始时全-1，主动触发前进1m
        if np.all(np.array(self.map.data) == -1):
            self.get_logger().info("[INIT] Map all unknown → move forward")
            x, y, yaw = pose
            goal_x = x + 1.0 * math.cos(yaw)
            goal_y = y + 1.0 * math.sin(yaw)
            self.send_goal((goal_x, goal_y))
            return
    
        # 正在导航 → 监控安全
        if self.goal_handle:
            if self.check_emergency(pose):
                self.get_logger().warn("[EMERGENCY] cancel goal")
                self.cancel_goal()
                self.simple_recover()
            return

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
            key = (round(cent[0], 2), round(cent[1], 2))
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
        res = self.map.info.resolution
        ox = self.map.info.origin.position.x # 地图原点 x
        oy = self.map.info.origin.position.y # 地图原点 y

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
                        mx = ox + (ac + 0.5) * res
                        my = oy + (h - ar - 0.5) * res
                        # my = oy + ar * res

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
            ox = self.map.info.origin.position.x
            oy = self.map.info.origin.position.y
            c_idx = int((cx - ox) / res)
            r_idx = int(h - (cy - oy) / res)
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
                2.0 * size -           # 面积权重
                4.0 * (1-risk) -       # 越安全越优先，risk 越大扣分越多
                0.0                     # 可加其他惩罚项
            )
            # 加上未知奖励（未知越多分越低，前沿越优先）
            score -= 2.0 * unk_count * res**2  # 未知面积权重，可调

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
        ox = self.map.info.origin.position.x
        oy = self.map.info.origin.position.y

        # 转换为栅格索引
        c = int((cx - ox) / res)
        r = int(h - (cy - oy) / res)

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

    def check_emergency(self, pose):
        if self.scan is None:
            return False

        ranges = np.array(self.scan.ranges)
        maxr = self.scan.range_max
        invalid = np.logical_or(np.isinf(ranges), ranges >= maxr - 1e-3)
        frac = np.count_nonzero(invalid) / len(ranges)

        self.get_logger().debug(
            f"[LASER] blind_frac={frac:.2f} bad_frames={self.bad_frames}"
        )
        # 激光90%为无效障碍的情况下
        if frac > self.laser_blind_frac:
            self.bad_frames += 1
            if self.bad_frames >= self.bad_frame_req:
                self.get_logger().warn("[LASER] blind → map fallback")
                return self.map_forward_collision(pose)
            return False

        self.bad_frames = 0

        # 视野内激光障碍小于安全距离0.6m
        front = ranges[len(ranges)//2 - 10 : len(ranges)//2 + 10]
        if np.any(front < self.safe_dist):
            self.get_logger().warn("[LASER] obstacle detected")
            return True

        return False

    def map_forward_collision(self, pose):
        x, y, yaw = pose
        res = self.map.info.resolution
        steps = int(self.safe_dist / res)
        # 循环沿机器人前方每格
        for i in range(steps):
            d = i * res
            px = x + d * math.cos(yaw)
            py = y + d * math.sin(yaw)
            # 判断是否阻塞
            if self.map_cell_blocked(px, py, d):
                self.get_logger().warn(
                    f"[MAP-COLLISION] blocked at {px:.2f},{py:.2f}"
                )
                return True
        return False

    def map_cell_blocked(self, x, y, dist):
        res = self.map.info.resolution
        h = self.map.info.height
        w = self.map.info.width
        ox = self.map.info.origin.position.x
        oy = self.map.info.origin.position.y

        c = int((x - ox) / res)
        r = int(h - (y - oy) / res)
        # r = int((y - oy) / res)

        if r < 0 or r >= h or c < 0 or c >= w:
            return False

        v = self.map.data[r * w + c]
        # 如果该格被标记为障碍 → 阻塞 → 返回 True
        if v > 50:
            return True
        # 如果该格未知 (-1)
        # 且距离超过 unknown_soft_limit → 阻塞 → 返回 True
        if v == -1 and dist > self.unknown_soft_limit:
            return True
        return False

    # ================= Navigation =================

    def send_goal(self, pos):
        self.get_logger().info(f"[NAV] send goal {pos}")
        goal = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = self.map.header.frame_id
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = pos[0]
        ps.pose.position.y = pos[1]
        ps.pose.orientation.w = 1.0
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
        self.get_logger().info(f"[NAV] finished status={status}")
        if status != 4:
            self.failed_goals[self.current_goal] = time.time()
        self.goal_handle = None

    def cancel_goal(self):
        if self.goal_handle:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None

    def simple_recover(self):
        self.get_logger().info("[RECOVER] simple pause & retry")

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