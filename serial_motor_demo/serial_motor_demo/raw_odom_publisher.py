import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import math

class RawOdometryNode(Node):
    def __init__(self):
        super().__init__('raw_odometry_node')
        self.create_subscription(String, '/combined_data', self.process_data, 1000)

        self.wheel_odom_pub = self.create_publisher(Odometry, '/odometry/wheel', 1000)
        self.imu_odom_pub = self.create_publisher(Odometry, '/odometry/imu', 1000)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.last_time = self.get_clock().now()
        self.last_encoders = {"a": 0.0, "b": 0.0, "c": 0.0, "d": 0.0}

        # Robot constants
        self.CPR_A = 8650
        self.CPR_B = 8636
        self.CPR_C = 6746
        self.CPR_D = 8641
        self.R = 0.076
        self.L = 0.355
        self.W = 0.245

        # Integrated position state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def process_data(self, msg):
        try:
            parsed = json.loads(msg.data)
            now = self.get_clock().now()
            dt = (now - self.last_time).nanoseconds / 1e9
            if dt <= 0:
                return
            self.last_time = now

            # Get encoder and IMU quaternion data
            a, b, c, d = parsed.get("a", 0.0), parsed.get("b", 0.0), parsed.get("c", 0.0), parsed.get("d", 0.0)
            qx, qy, qz, qw = parsed.get("x", 0.0), parsed.get("y", 0.0), parsed.get("z", 0.0), parsed.get("w", 1.0)

            # Apply inversion just like in your EKF code
            a, d = -a, -d

            # Compute individual wheel speeds
            v_rr = ((a - self.last_encoders["a"]) * 2 * math.pi * self.R) / (self.CPR_A * dt)
            v_fr = ((b - self.last_encoders["b"]) * 2 * math.pi * self.R) / (self.CPR_B * dt)
            v_fl = ((c - self.last_encoders["c"]) * 2 * math.pi * self.R) / (self.CPR_C * dt)
            v_rl = ((d - self.last_encoders["d"]) * 2 * math.pi * self.R) / (self.CPR_D * dt)

            # Update encoder readings AFTER calculating velocities
            self.last_encoders = {"a": a, "b": b, "c": c, "d": d}

            # Robot-centric velocity
            vx = (v_fl + v_fr + v_rl + v_rr) / 4.0
            vy = (-v_fl + v_fr + v_rl - v_rr) / 4.0
            omega = (-v_fl + v_fr - v_rl + v_rr) / (4.0 * (self.L + self.W))

            self.get_logger().info(f"vx: {vx:.4f}, vy: {vy:.4f}, omega: {omega:.4f}")

            # Integrate position
            dx = (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
            dy = (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt
            dtheta = omega * dt

            self.x += dx
            self.y += dy
            self.theta += dtheta
            self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

            timestamp = now.to_msg()

            # Publish
            self.publish_wheel_odom(timestamp, vx, vy, omega)
            self.publish_imu_odom(timestamp, qx, qy, qz, qw)

        except Exception as e:
            self.get_logger().error(f"Error processing data: {e}")

    def publish_wheel_odom(self, timestamp, vx, vy, omega):
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, self.theta)
        odom = Odometry()
        odom.header.stamp = timestamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"

        # Pose
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.pose.covariance = [
            0.05, 0.0,  0.0, 0.0, 0.0, 0.0,
            0.0,  0.05, 0.0, 0.0, 0.0, 0.0,
            0.0,  0.0,  999.0, 0.0, 0.0, 0.0,
            0.0,  0.0,  0.0, 999.0, 0.0, 0.0,
            0.0,  0.0,  0.0, 0.0, 999.0, 0.0,
            0.0,  0.0,  0.0, 0.0, 0.0, 0.2
        ]

        # Twist
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = omega
        odom.twist.covariance = [
            0.1, 0.0,  0.0, 0.0, 0.0, 0.0,
            0.0, 0.1,  0.0, 0.0, 0.0, 0.0,
            0.0, 0.0,  999.0, 0.0, 0.0, 0.0,
            0.0, 0.0,  0.0, 999.0, 0.0, 0.0,
            0.0, 0.0,  0.0, 0.0, 999.0, 0.0,
            0.0, 0.0,  0.0, 0.0, 0.0, 0.2
        ]

        self.wheel_odom_pub.publish(odom)

    def publish_imu_odom(self, timestamp, qx, qy, qz, qw):
        odom = Odometry()
        odom.header.stamp = timestamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"

        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.pose.covariance = [
            999.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 999.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.05
        ]

        odom.twist.covariance = [0.0] * 36
        self.imu_odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = RawOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
