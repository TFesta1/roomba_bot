#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from camera_info_manager import CameraInfoManager
from cv_bridge import CvBridge
import cv2
import requests
import numpy as np

class ESP32CamHTTP(Node):
    def __init__(self):
        super().__init__('esp32_cam_http')
        # Parameters
        self.declare_parameter('url', 'http://192.168.0.84/cam-hi.jpg')
        self.declare_parameter('frame_id', 'camera_frame')
        self.declare_parameter('fps', 5.0)
        self.declare_parameter('camera_info_url', '')

        url           = self.get_parameter('url').value
        self.frame_id = self.get_parameter('frame_id').value
        fps           = self.get_parameter('fps').value
        info_url      = self.get_parameter('camera_info_url').value
        # print(info_url)

        # Publishers
        self.img_pub  = self.create_publisher(Image,      'camera/image_raw', 10)
        self.ci_pub   = self.create_publisher(CameraInfo,'camera/camera_info',10)

        # CameraInfo manager (load YAML if provided)
        self.ci_man = CameraInfoManager(self, 'esp32_cam', info_url)
        ok = self.ci_man.loadCameraInfo()
        # print(ok)
        # if not ok:
        #     self.get_logger().warn(f'Failed to loadCameraInfo() {ok}')
        if not self.ci_man.isCalibrated():
            self.get_logger().warn('CameraInfo not calibrated or no YAML provided')

        self.bridge = CvBridge()
        timer_period = 1.0 / fps
        self.timer     = self.create_timer(timer_period, self.timer_cb)
        self.cam_url   = url
        self.get_logger().info(f'Publishing {url} at {fps} Hz')

    def timer_cb(self):
        try:
            resp = requests.get(self.cam_url, timeout=1.0)
            img_arr = np.frombuffer(resp.content, np.uint8)
            cv_img  = cv2.imdecode(img_arr, cv2.IMREAD_COLOR)
            ros_img = self.bridge.cv2_to_imgmsg(cv_img, 'bgr8')
            ros_img.header.stamp    = self.get_clock().now().to_msg()
            ros_img.header.frame_id = self.frame_id

            ci = self.ci_man.getCameraInfo()
            ci.header.stamp    = ros_img.header.stamp
            ci.header.frame_id = self.frame_id

            self.img_pub.publish(ros_img)
            self.ci_pub.publish(ci)
            self.get_logger().info(f'Successfully published to camera/image_raw and camera/camera_info from http://192.168.0.84/cam-hi.jpg')

        except Exception as e:
            self.get_logger().info(f'Failed to fetch or publish image: {e}')
            # self.get_logger().error(f'Failed to fetch or publish image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ESP32CamHTTP()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()