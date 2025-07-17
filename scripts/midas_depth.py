#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import onnxruntime
import cv2


class MiDaSDepthNode(Node):
    def __init__(self):
        super().__init__('midas_depth')
        # --- Parameters ---
        self.declare_parameter(
            'model_path',
            '/home/you/models/midas_small.onnx'
        )
        self.declare_parameter(
            'input_topic',
            '/camera/image_raw'
        )
        self.declare_parameter(
            'output_topic',
            '/camera/depth/image_raw'
        )

        self.publish_once = False

        model_path = self.get_parameter('model_path').value
        in_topic   = self.get_parameter('input_topic').value
        out_topic  = self.get_parameter('output_topic').value

        # --- Load ONNX model ---
        self.session  = onnxruntime.InferenceSession(
            model_path,
            providers=['CPUExecutionProvider']
        )
        # Input metadata: name and shape (may contain symbolic dims)
        inp_meta = self.session.get_inputs()[0]
        self.input_name  = inp_meta.name
        self.input_shape = [dim if isinstance(dim, int) else '?' 
                             for dim in inp_meta.shape]
        self.get_logger().info(
            f'MiDaS model expects input "{self.input_name}" '
            f'with shape {self.input_shape}'
        )

        # --- ROS pubs/subs ---
        self.bridge = CvBridge()
        self.pub    = self.create_publisher(Image, out_topic, 10)
        self.sub    = self.create_subscription(
            Image, in_topic, self.callback, 10
        )

    def callback(self, img_msg: Image):
        # Convert ROS image to OpenCV RGB
        cv_img = self.bridge.imgmsg_to_cv2(img_msg, 'rgb8')
        h, w, _ = cv_img.shape

        # Preprocess: resize + normalize to [0,1]
        tgt_h, tgt_w = 256, 256
        img = cv2.resize(cv_img, (tgt_w, tgt_h)).astype(np.float32) / 255.0

        # Determine if model wants NHWC or NCHW
        # We'll assume NCHW (most MiDaS exports). If your model really
        # expects NHWC (rare), you can swap these two lines.
        inp = img.transpose(2, 0, 1)  # HWC→CHW

        # Add batch dimension if missing
        if inp.ndim == 3:
            inp = np.expand_dims(inp, axis=0)  # CHW→NCHW

        # Debug prints
        self.get_logger().debug(
            f'Feeding input "{self.input_name}" of shape {inp.shape}'
        )

        # Run inference
        try:
            raw_out = self.session.run(None, {self.input_name: inp})[0]
        except Exception as e:
            self.get_logger().error(
                f'ONNXRuntime failed: {e}\n'
                f'  Expected shape: {self.input_shape}, '
                f'got array with shape {inp.shape}'
            )
            return

        # Extract depth map (handles both 4D and 3D outputs)
        if raw_out.ndim == 4:
            depth_map = raw_out[0, 0, :, :]
        elif raw_out.ndim == 3:
            depth_map = raw_out[0, :, :]
        else:
            self.get_logger().error(
                f'Unexpected output dims: {raw_out.ndim}, '
                f'shape: {raw_out.shape}'
            )
            return

        # Resize to original camera resolution
        depth_resized = cv2.resize(depth_map, (w, h))

        # Publish as 32FC1 ROS Image
        depth_msg = self.bridge.cv2_to_imgmsg(
            depth_resized.astype(np.float32),
            encoding='32FC1'
        )
        depth_msg.header = img_msg.header
        self.pub.publish(depth_msg)
        if self.publish_once == False:
            self.get_logger().info("Published depth")
            self.publish_once = True

def main(args=None):
    rclpy.init(args=args)
    node = MiDaSDepthNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()