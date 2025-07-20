#!/usr/bin/env python3
"""
roomba_bridge.py

A ROS 2 node that:
- Translates Twist messages into HTTP motor commands on an ESP32
- Polls /encoders → publishes JointState
- Integrates wheel motions → publishes Odometry + TF
"""

from math import sin, cos
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
# import tf_transformations
# from tf_transformations_py import transformations as tf_transformations
import tf2_ros
import requests
from requests.exceptions import RequestException


def quaternion_from_yaw(yaw: float):
    """
    Return (x,y,z,w) quaternion for a rotation about Z by yaw radians.
    """
    half = yaw * 0.5
    return (0.0, 0.0, sin(half), cos(half))


class RoombaBridge(Node):
    def __init__(self):
        super().__init__('roomba_bridge')

        # 1) Parameters
        self.declare_parameter('base_url', 'http://192.168.0.84')
        self.base_url = self.get_parameter('base_url').get_parameter_value().string_value

        # Robot geometry (must match your real wheels)
        self.declare_parameter('wheel_radius', 0.035)       # meters
        self.declare_parameter('wheel_separation', 0.28)    # meters
        self.wheel_radius    = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value

        # 2) Publishers
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.odom_pub  = self.create_publisher(Odometry,     'odom',         10)

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 3) Subscriber—where teleop publishes remapped Twist
        self.create_subscription(
            Twist,
            '/diff_cont/cmd_vel_unstamped',#'/cmd_vel', ##'',            # have teleop publish here
            self.twist_callback,
            10
        )

        # 4) State
        self.last_cmd = 'stop'
        self.x = 0.0      # robot pose
        self.y = 0.0
        self.theta = 0.0

        # Last wheel angles (radians) from encoders
        self.last_left_angle  = 0.0
        self.last_right_angle = 0.0

        # 5) Timer: poll encoders @ 50 Hz
        self.create_timer(1.0/50.0, self.poll_encoders)

        self.get_logger().info(f"RoombaBridge up. ESP32 at {self.base_url}")

    def twist_callback(self, msg: Twist):
        """
        Send HTTP control for btn-forward/back/left/right depending on Twist
        """
        lin = msg.linear.x
        ang = msg.angular.z
        cmd = 'stop'
        eps = 1e-2

        if lin > eps and abs(ang) < eps:
            cmd = 'btn-forward'
        elif lin < -eps and abs(ang) < eps:
            cmd = 'btn-back'
        elif ang > eps and abs(lin) < eps:
            cmd = 'btn-left'
        elif ang < -eps and abs(lin) < eps:
            cmd = 'btn-right'

        if cmd != self.last_cmd:
            # Turn everything off if stopping
            if cmd == 'stop':
                for v in ['btn-forward','btn-back','btn-left','btn-right']:
                    self.send_control(v, 0)
            else:
                # Activate just the chosen direction
                self.send_control(cmd, 1)

            self.last_cmd = cmd
            self.get_logger().debug(f"HTTP cmd: {cmd}")

    def poll_encoders(self):
        """
        1) GET /encoders
        2) Publish JointState
        3) Integrate wheel angles → pose → Odometry + TF
        """
        try:
            resp = requests.get(f"{self.base_url}/encoders", timeout=0.1)
            data = resp.json()
            left_angle  = float(data.get('left',  0.0))
            right_angle = float(data.get('right', 0.0))

            # --- 2) Publish JointState ---
            js = JointState()
            now = self.get_clock().now().to_msg()
            js.header.stamp = now
            js.name     = ['left_wheel_joint', 'right_wheel_joint']
            js.position = [left_angle, right_angle]
            self.joint_pub.publish(js)

            # --- 3) Odometry Integration ---
            # compute delta angles
            d_left  = left_angle  - self.last_left_angle
            d_right = right_angle - self.last_right_angle
            self.last_left_angle  = left_angle
            self.last_right_angle = right_angle

            # convert to linear distances
            dist_left  = d_left  * self.wheel_radius
            dist_right = d_right * self.wheel_radius

            # average forward distance, and change in heading
            ds     = (dist_left + dist_right) / 2.0
            dtheta = (dist_right - dist_left) / self.wheel_separation

            # integrate pose
            self.theta += dtheta
            self.x     += ds * math.cos(self.theta)
            self.y     += ds * math.sin(self.theta)

            # publish Odometry
            odom = Odometry()
            odom.header.stamp    = now
            odom.header.frame_id = 'odom'
            odom.child_frame_id  = 'base_link'

            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0.0
            # q = tf_transformations.quaternion_from_euler(0, 0, self.theta)
            qx, qy, qz, qw = quaternion_from_yaw(self.theta)
            odom.pose.pose.orientation.x = qx#q[0]
            odom.pose.pose.orientation.y = qy#q[1]
            odom.pose.pose.orientation.z = qz#q[2]
            odom.pose.pose.orientation.w = qw#q[3]

            # velocities in body frame
            odom.twist.twist.linear.x  = ds * 50.0   # approx v = ds/dt, dt=1/50
            odom.twist.twist.angular.z = dtheta * 50.0
            self.odom_pub.publish(odom)

            # broadcast TF
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'odom'
            t.child_frame_id  = 'base_link'
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = qx#q[0]
            t.transform.rotation.y = qy#q[1]
            t.transform.rotation.z = qz#q[2]
            t.transform.rotation.w = qw#q[3]
            self.tf_broadcaster.sendTransform(t)

        except (RequestException, ValueError) as e:
            self.get_logger().warn(f"Encoder poll failed: {e}")

    def send_control(self, var: str, val: int):
        url = f"{self.base_url}/control?var={var}&val={val}"
        try:
            requests.get(url, timeout=0.05)
        except RequestException as e:
            self.get_logger().warn(f"HTTP send failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RoombaBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()