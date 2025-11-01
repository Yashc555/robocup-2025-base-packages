import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import random
import math

class FakeOdometryPublisher(Node):
    def __init__(self):
        super().__init__('fake_odometry_publisher')
        self.publisher_ = self.create_publisher(Odometry, '/odom', 10)
        self.timer = self.create_timer(0.1, self.publish_odometry)  # 10Hz
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def publish_odometry(self):
        # Update position and orientation randomly (this is fake data)
        self.x += random.uniform(0.0, 0.1)
        self.y += random.uniform(0.0, 0.1)
        self.theta += random.uniform(-0.1, 0.1)
        
        # Create the odometry message
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'

        # Set position (x, y)
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Set orientation (using a simple quaternion)
        quat = Quaternion()
        quat.x = 0.0
        quat.y = 0.0
        quat.z = math.sin(self.theta / 2)
        quat.w = math.cos(self.theta / 2)
        odom.pose.pose.orientation = quat

        # Set velocity (can be zero or random for fake data)
        odom.twist.twist.linear.x = random.uniform(0.0, 0.1)
        odom.twist.twist.linear.y = random.uniform(0.0, 0.1)
        odom.twist.twist.angular.z = random.uniform(-0.1, 0.1)

        # Publish the fake odometry
        self.publisher_.publish(odom)

        self.get_logger().info(f'Publishing fake odometry: x={self.x}, y={self.y}, theta={self.theta}')

def main(args=None):
    rclpy.init(args=args)
    fake_odometry_publisher = FakeOdometryPublisher()
    rclpy.spin(fake_odometry_publisher)
    fake_odometry_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
