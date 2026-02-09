import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus

import numpy as np
import cv2
import random
import math
import threading

import tf2_ros


class ContinuousExplorer(Node):

    def __init__(self):
        super().__init__('continuous_explorer')

        # ---- ROS interfaces ----
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10
        )
        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- state ----
        self.map_data = None
        self.map_lock = threading.Lock()
        self.is_moving = False
        self.last_goal_failed = False

        # ---- parameters ----
        self.declare_parameter('frontier_kernel', 5)
        self.declare_parameter('min_frontier_area', 20.0)
        self.declare_parameter('wander_distance', 6.0)

        # ---- timer ----
        self.timer = self.create_timer(3.0, self.exploration_loop)

        self.get_logger().info("🚀 Continuous Explorer v2 启动")

        # 等一次 action server（不在发送时阻塞）
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn("navigate_to_pose action server 尚不可用")

    # ------------------------------------------------------------------

    def map_callback(self, msg):
        with self.map_lock:
            self.map_data = msg

    # ------------------------------------------------------------------

    def exploration_loop(self):
        if self.is_moving:
            return

        with self.map_lock:
            msg = self.map_data
        if msg is None: return

        # 尝试寻找边界
        pose = self.find_frontier(msg)

        if pose is not None:
            self.get_logger().info("🎯 发现有效边界，正在前往...")
            self.send_goal(pose)
        else:
            # 只有找不到边界时才进入漫游模式
            self.get_logger().warn("❓ 未发现边界，开始向未知区域漫游...")
            pose = self.sample_wander_goal(msg)
            if pose:
                self.send_goal(pose)

    def find_frontier(self, msg):
        w, h = msg.info.width, msg.info.height
        # 注意：ROS OccupancyGrid 是 row-major，data[0] 是 (0,0)
        data = np.array(msg.data, dtype=np.int8).reshape(h, w)

        free = (data == 0).astype(np.uint8) * 255
        unknown = (data == -1).astype(np.uint8) * 255

        k = self.get_parameter('frontier_kernel').value
        kernel = np.ones((k, k), np.uint8)

        # 膨胀自由空间，寻找与未知空间的交集
        dilated = cv2.dilate(free, kernel, iterations=1)
        frontier_mask = cv2.bitwise_and(dilated, unknown)

        contours, _ = cv2.findContours(frontier_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        min_area = self.get_parameter('min_frontier_area').value
        
        robot_pose = self.get_robot_pose()
        best_pose = None
        min_dist = float('inf')

        for c in contours:
            if cv2.contourArea(c) < min_area: continue
            
            M = cv2.moments(c)
            if M['m00'] == 0: continue
            cx, cy = M['m10'] / M['m00'], M['m01'] / M['m00']
            
            # 使用修复后的坐标转换
            mx, my = self.grid_to_map(cx, cy, msg.info)

            if robot_pose:
                dist = math.hypot(mx - robot_pose[0], my - robot_pose[1])
                # 过滤掉距离机器人太近的边界（避免原地打转）
                if dist < 1.0: continue 
                
                if dist < min_dist:
                    min_dist = dist
                    best_pose = (mx, my)
        
        return self.make_pose(best_pose[0], best_pose[1], msg.header.frame_id) if best_pose else None

    def sample_wander_goal(self, msg):
        """
        优化后的漫游：基于机器人当前位置和朝向，向前方投射点。
        """
        robot_pose = self.get_robot_pose()
        if not robot_pose: return None
        
        rx, ry, r_yaw = robot_pose
        d = self.get_parameter('wander_distance').value
        
        # 在当前朝向的 ±90 度范围内随机选一个角度
        target_yaw = r_yaw + random.uniform(-math.pi/2, math.pi/2)
        
        tx = rx + d * math.cos(target_yaw)
        ty = ry + d * math.sin(target_yaw)
        
        self.get_logger().info(f"🎲 漫游目标点: ({tx:.2f}, {ty:.2f})")
        return self.make_pose(tx, ty, msg.header.frame_id)

    def grid_to_map(self, col, row, info):
        # 修复后的坐标逻辑
        x = col * info.resolution + info.origin.position.x
        # y = row * info.resolution + info.origin.position.y
        y = (info.height - 1 - row) * info.resolution + info.origin.position.y

        return x, y

    def get_robot_pose(self):
        try:
            # 获取 map 到 base_link 的变换
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', timeout=rclpy.duration.Duration(seconds=0.3))
            x = t.transform.translation.x
            y = t.transform.translation.y
            # 获取朝向角 (Yaw)
            q = t.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            return x, y, yaw
        except Exception:
            return None
        
    # ------------------------------------------------------------------

    def make_pose(self, x, y, frame_id):
        p = PoseStamped()
        p.header.frame_id = frame_id or 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        p.pose.orientation.w = 1.0
        return p

    # ------------------------------------------------------------------

    def send_goal(self, pose):
        self.is_moving = True
        goal = NavigateToPose.Goal()
        goal.pose = pose

        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("❌ 目标被拒绝")
            self.is_moving = False
            self.last_goal_failed = True
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("✅ 导航成功")
            self.last_goal_failed = False
        else:
            self.get_logger().warn(f"⚠️ 导航失败 status={status}")
            self.last_goal_failed = True

        self.is_moving = False


# ----------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = ContinuousExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()