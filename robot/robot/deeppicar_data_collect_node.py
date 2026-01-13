#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CompressedImage
from ackermann_msgs.msg import AckermannDriveStamped

from cv_bridge import CvBridge
import cv2
import csv
import os
import numpy as np
from message_filters import Subscriber, ApproximateTimeSynchronizer


class DeepPiCarDataCollector(Node):

    def __init__(self):
        super().__init__('deeppicar_data_collector')

        # ===== 参数 =====
        self.declare_parameter('output_dir', 'dataset')
        self.declare_parameter('max_steering_angle', 0.34)  # rad ≈ 20deg
        self.declare_parameter('use_compressed', False)

        self.output_dir = self.get_parameter('output_dir').value
        self.max_steering = self.get_parameter('max_steering_angle').value
        self.use_compressed = self.get_parameter('use_compressed').value

        self.image_dir = os.path.join(self.output_dir, 'images')
        os.makedirs(self.image_dir, exist_ok=True)

        self.csv_path = os.path.join(self.output_dir, 'labels.csv')

        self.bridge = CvBridge()
        self.frame_id = 0

        # ===== CSV =====
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['image', 'steering', 'throttle'])

        # ===== Subscribers =====
        if self.use_compressed:
            self.image_sub = Subscriber(
                self,
                CompressedImage,
                '/camera/image_raw/compressed'
            )
            self.get_logger().info('📷 Using COMPRESSED image')
        else:
            self.image_sub = Subscriber(
                self,
                Image,
                '/camera/image_raw'
            )
            self.get_logger().info('📷 Using RAW image')

        self.ctrl_sub = Subscriber(
            self,
            AckermannDriveStamped,
            '/cmddel'
        )

        self.ts = ApproximateTimeSynchronizer(
            [self.image_sub, self.ctrl_sub],
            queue_size=30,
            slop=0.05
        )
        self.ts.registerCallback(self.callback)

        self.get_logger().info('🚗 DeepPiCar data collector started')

    # ===============================
    def callback(self, image_msg, ctrl_msg):
        # ---- image decode ----
        if self.use_compressed:
            np_arr = np.frombuffer(image_msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        else:
            cv_image = self.bridge.imgmsg_to_cv2(
                image_msg, desired_encoding='bgr8'
            )

        # ---- DeepPiCar crop + resize ----
        h, w, _ = cv_image.shape
        crop = cv_image[int(h * 0.35):h, :]
        resized = cv2.resize(crop, (200, 66))

        # ---- control ----
        steering = ctrl_msg.drive.steering_angle
        throttle = ctrl_msg.drive.speed

        steering_norm = steering / self.max_steering
        steering_norm = max(-1.0, min(1.0, steering_norm))

        # ---- save ----
        img_name = f'{self.frame_id:06d}.jpg'
        img_path = os.path.join(self.image_dir, img_name)

        cv2.imwrite(img_path, resized)
        self.csv_writer.writerow([img_name, steering_norm, throttle])

        self.frame_id += 1

        if self.frame_id % 50 == 0:
            self.get_logger().info(f'Saved {self.frame_id} frames')

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = DeepPiCarDataCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
