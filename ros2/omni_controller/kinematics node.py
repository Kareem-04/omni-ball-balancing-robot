import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
import time

# ------------------ Serial to Arduino ------------------
import serial

# Replace with your actual Arduino serial port
ser = serial.Serial('/dev/ttyUSB0', 115200)

def send_to_arduino(w1, w2, w3):
    """Send wheel speeds to Arduino as bytes (0-255)"""
    def map_speed(s):
        # Map -10..10 rad/s → 0..255
        return int(max(min((s / 10.0) * 127 + 128, 255), 0))
    ser.write(bytes([map_speed(w1), map_speed(w2), map_speed(w3)]))

# ------------------ Omni Controller Node ------------------
class OmniController(Node):
    def __init__(self):
        super().__init__('omni_controller')

        # --- Robot parameters ---
        self.r = 0.034   # Wheel radius (m)
        self.L = 0.09    # Distance from center to wheel

        # --- State ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = time.time()

        # --- ROS Interfaces ---
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.02, self.update)  # 50 Hz

        self.get_logger().info("Omni Controller Started")

        # Wheel speeds in rad/s
        self.w1 = 0.0
        self.w2 = 0.0
        self.w3 = 0.0

    # -------------------------------
  def cmd_vel_callback(self, msg):
    vx = msg.linear.x
    vy = msg.linear.y
    wz = msg.angular.z

    # === Corrected kinematics for your real wheel layout ===
    # Motor 1: Back
    self.w1 = ( vy + self.L * wz ) / self.r

    # Motor 2: Front Right
    self.w2 = ( (math.sqrt(3)/2) * vx - 0.5 * vy + self.L * wz ) / self.r

    # Motor 3: Front Left
    self.w3 = ( -(math.sqrt(3)/2) * vx - 0.5 * vy + self.L * wz ) / self.r

    send_to_arduino(self.w1, self.w2, self.w3)

    self.get_logger().info(
        f"Wheels rad/s: w1(back)={self.w1:.2f}, "
        f"w2(fr)={self.w2:.2f}, w3(fl)={self.w3:.2f}"
    )


    # -------------------------------
    def update(self):
        """Update odometry and TF based on last wheel commands"""
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # -------------------------------
        # Odometry estimation from wheel speeds
        # Replace with encoder feedback for accurate odometry
        vx = (self.r / 3.0) * (2*self.w3 - self.w1 - self.w2)
        vy = (self.r / math.sqrt(3)) * (self.w1 - self.w2)
        wz = (self.r / (3*self.L)) * (self.w1 + self.w2 + self.w3)

        # Integrate pose
        self.x += (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
        self.y += (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt
        self.theta += wz * dt

        self.publish_odometry(vx, vy, wz)

    # -------------------------------
    def publish_odometry(self, vx, vy, wz):
        """Publish /odom and TF"""
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz

        self.odom_pub.publish(odom)

        # Publish TF
        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

# -------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = OmniController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
