#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import (
    BasicNavigator,
    TaskResult
)
import math
import threading
import time


class ManualNavCommander(Node):

    def __init__(self):
        super().__init__('manual_nav_commander')

        self.navigator = BasicNavigator()

        # --------- 配置区 ---------
        self.mode = "patrol"   # single | multi | patrol

        self.single_goal = (1.0, 0.0, 0.0)

        self.multi_goals = [
            (1.0, 0.0, 0.0),
            (1.0, 1.0, 1.57),
            (0.0, 1.0, 3.14)
        ]

        self.patrol_goals = [
            (0.5, 0.0, 0.0),
            (0.5, 0.5, 1.57),
            (0.0, 0.5, 3.14)
        ]
        # -------------------------

        # ❗不要在 __init__ 里直接跑任务
        self.timer = self.create_timer(1.0, self._start_once)
        self.started = False

    def _start_once(self):
        if self.started:
            return
        self.started = True
        self.timer.cancel()

        # 使用线程避免阻塞 spin
        thread = threading.Thread(target=self.run)
        thread.daemon = True
        thread.start()

    # ---------------- 核心逻辑 ----------------

    def run(self):
        self.get_logger().info("Waiting for Nav2...")
        self.navigator.waitUntilNav2Active()

        self._set_initial_pose()

        if self.mode == "single":
            self.run_single()
        elif self.mode == "multi":
            self.run_multi()
        elif self.mode == "patrol":
            self.run_patrol()

    # ---------------- 基础能力 ----------------

    def _set_initial_pose(self):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.orientation.w = 1.0

        self.navigator.setInitialPose(pose)
        time.sleep(1.0)  # 给 AMCL 一点时间

    def _make_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        # 2D 旋转：绕 Z 轴旋转 yaw 弧度
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    # ---------------- 任务模式 ----------------

    def run_single(self):
        goal = self._make_pose(*self.single_goal)
        self.navigator.goToPose(goal)

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().info(
                    f"Distance remaining: {feedback.distance_remaining:.2f} m"
                )
            time.sleep(0.5)

        self._handle_result()

    def run_multi(self):
        goals = [self._make_pose(*g) for g in self.multi_goals]
        self.navigator.followWaypoints(goals)

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().info(
                    f"Executing waypoint {feedback.current_waypoint_index + 1}/{len(goals)}"
                )
            time.sleep(0.5)

        self._handle_result()

    def run_patrol(self):
        goals = [self._make_pose(*g) for g in self.patrol_goals]

        while rclpy.ok():
            self.navigator.followWaypoints(goals)

            while not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
                if feedback:
                    self.get_logger().info(
                        f"[Patrol] Waypoint {feedback.current_waypoint_index + 1}"
                    )
                time.sleep(0.5)

            self.get_logger().info("Patrol round complete, restarting...")
            time.sleep(2.0)

    # ---------------- 结果处理 ----------------

    def _handle_result(self):
        result = self.navigator.getResult()

        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("Navigation succeeded!")
        elif result == TaskResult.CANCELED:
            self.get_logger().warn("Navigation canceled")
        elif result == TaskResult.FAILED:
            self.get_logger().error("Navigation failed!")


def main():
    rclpy.init()
    node = ManualNavCommander()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
