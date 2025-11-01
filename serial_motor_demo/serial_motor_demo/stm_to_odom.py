import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import re

class STMToOdom(Node):
    def __init__(self):
        super().__init__('stm_to_odom')

        # Subscriber to STM Data
        self.stm_sub = self.create_subscription(String, '/stm_data', self.process_stm_data, 10)

        # Publisher for Odometry
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # Encoder values
        self.encoder_values = {'A': 0, 'B': 0, 'C': 0, 'D': 0}

    def process_stm_data(self, msg):
        """
        Processes incoming STM data and updates encoder values.
        """
        match = re.match(r'([ABCD])([-]?\d+)', msg.data.strip())
        if match:
            key, value = match.groups()
            self.encoder_values[key] = int(value)
            if all(k in self.encoder_values for k in 'ABCD'):
                self.publish_odometry()
        else:
            self.get_logger().warn(f"Invalid STM data format: {msg.data}")

    def publish_odometry(self):
        """
        Computes and publishes odometry based on encoder values.
        """
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # Convert to seconds
        self.last_time = current_time

        # Extract encoder readings
        A = self.encoder_values['A']
        B = self.encoder_values['B']
        C = self.encoder_values['C']
        D = self.encoder_values['D']

        # Placeholder: Convert encoder values to velocity
        vx = (A + B + C + D) / 4.0  # Forward velocity
        vy = (-A + B + C - D) / 4.0  # Lateral velocity
        vtheta = (-A + B - C + D) / 4.0  # Rotational velocity

        self.x += vx * dt
        self.y += vy * dt
        self.theta += vtheta * dt

        # Create Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"

        # Position update
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = Quaternion(w=1.0)

        # Velocity update
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.angular.z = vtheta

        # Publish odometry
        self.odom_pub.publish(odom_msg)
        self.get_logger().info(f"Published Odometry: x={self.x:.2f}, y={self.y:.2f}, theta={self.theta:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = STMToOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
