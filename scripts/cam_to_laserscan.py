#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, LaserScan
from cv_bridge import CvBridge
import message_filters
import numpy as np
import cv2
import math

class CamToLaserScan(Node):
    def __init__(self):
        super().__init__('cam_to_laserscan')

        # Parameters
        self.declare_parameter('image_topic',        '/camera/image_raw')
        self.declare_parameter('info_topic',         '/camera/camera_info')
        self.declare_parameter('scan_topic',         '/scan')
        self.declare_parameter('frame_id',           'base_link')
        self.declare_parameter('min_range',          0.2)
        self.declare_parameter('max_range',          5.0)
        # How dark a pixel must be to count as "obstacle" (0–255)
        self.declare_parameter('obstacle_threshold', 50)
        # If >0, overrides computed FOV (radians)
        self.declare_parameter('horizontal_fov',     0.94)

        img_topic  = self.get_parameter('image_topic').value
        info_topic = self.get_parameter('info_topic').value
        scan_topic = self.get_parameter('scan_topic').value
        self.frame_id  = self.get_parameter('frame_id').value
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        self.thresh    = self.get_parameter('obstacle_threshold').value
        self.param_fov = self.get_parameter('horizontal_fov').value

        # Publisher & bridge
        self.scan_pub = self.create_publisher(LaserScan, scan_topic, 10)
        self.bridge   = CvBridge()

        # Synchronize Image + CameraInfo
        sub_img  = message_filters.Subscriber(self, Image,      img_topic,  qos_profile=10)
        sub_info = message_filters.Subscriber(self, CameraInfo, info_topic, qos_profile=10)
        ts = message_filters.ApproximateTimeSynchronizer(
            [sub_img, sub_info], queue_size=10, slop=0.1)
        ts.registerCallback(self.callback)

        self.get_logger().info(
            f'Listening to {img_topic} + {info_topic}, publishing LaserScan on {scan_topic}'
        )

    def callback(self, img_msg, ci_msg):
        # Convert to OpenCV BGR
        cv_img = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        height, width = cv_img.shape[:2]

        # Compute FOV from camera_info (unless overridden)
        fx = ci_msg.k[0]
        cx = ci_msg.k[2]
        h_fov = 2.0 * math.atan2(cx, fx)
        if self.param_fov > 0.0:
            h_fov = self.param_fov

        # Build LaserScan
        scan = LaserScan()
        scan.header.stamp    = img_msg.header.stamp
        scan.header.frame_id = self.frame_id
        scan.angle_min       = -h_fov / 2.0
        scan.angle_max       =  h_fov / 2.0
        scan.angle_increment = h_fov / float(width)
        scan.time_increment  = 0.0
        scan.scan_time       = 0.0
        scan.range_min       = self.min_range
        scan.range_max       = self.max_range
        scan.ranges          = [float('inf')] * width

        # Obstacle segmentation using threshold
        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        _, obstacle_mask = cv2.threshold(
            gray, self.thresh, 255, cv2.THRESH_BINARY_INV)

        # For each column, find nearest obstacle pixel
        for col in range(width):
            ys = np.where(obstacle_mask[:, col] > 0)[0]
            if ys.size:
                # topmost obstacle → farthest (inverse y)
                y = int(ys.min())
                depth = (
                    self.min_range +
                    (1.0 - y / float(height - 1)) *
                    (self.max_range - self.min_range)
                )
                scan.ranges[col] = float(depth)

        # Publish scan
        self.scan_pub.publish(scan)

def main(args=None):
    rclpy.init(args=args)
    node = CamToLaserScan()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()