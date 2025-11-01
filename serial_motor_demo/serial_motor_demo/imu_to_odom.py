#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
import math
from tf2_ros import TransformBroadcaster

class IMUToOdom(Node):
    def __init__(self):
        super().__init__('imu_to_odom')

        # Subscriber to IMU + Encoder Data
        self.imu_sub = self.create_subscription(String, '/combined_data', self.process_imu_data, 10)

        # Publisher for Odometry
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # TF broadcaster for odom to base_footprint transform
        self.tf_broadcaster = TransformBroadcaster(self)

        # Robot state variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # Yaw angle (orientation)
        self.last_time = self.get_clock().now()

        # Robot parameters (Adjust as per your robot)
        self.wheel_radius = 0.076  # Example: 5cm wheel radius
        self.robot_width = 0.3  # Distance between left & right wheels
        self.robot_length = 0.4  # Distance between front & back wheels

    def process_imu_data(self, msg):
        """
        Processes incoming IMU and encoder data.
        """
        try:
            # Adjust velocity with acceleration (integrate acceleration data)
            dt = (self.get_clock().now() - self.last_time).nanoseconds / 1e9
            self.last_time = self.get_clock().now()
            
            parsed_data = json.loads(msg.data)
            self.get_logger().info(f"Received: {parsed_data}")

            # Extract IMU data (orientation & angular rates)
            yaw = math.radians(parsed_data.get("yaw", 0.0))
            pitch = math.radians(parsed_data.get("pitch", 0.0))
            roll = math.radians(parsed_data.get("roll", 0.0))
            w = parsed_data.get("w", 1.0)
            x_q = parsed_data.get("x", 0.0)
            y_q = parsed_data.get("y", 0.0)
            z_q = parsed_data.get("z", 0.0)

            # Store IMU data as instance variables
            self.w = w
            self.x_q = x_q
            self.y_q = y_q
            self.z_q = z_q

            # Linear acceleration (p, q, r are accelerations in x, y, z)
            ax = parsed_data.get("p", 0.0)
            ay = parsed_data.get("q", 0.0)
            az = parsed_data.get("r", 0.0)

            # Encoder values (wheel encoders)
            a = parsed_data.get("a", 0.0)  # Front-left
            b = parsed_data.get("b", 0.0)  # Front-right
            c = parsed_data.get("c", 0.0)  # Rear-left
            d = parsed_data.get("d", 0.0)  # Rear-right

            # Compute velocities using encoders
            vx, vy, vtheta = self.compute_velocity(a, b, c, d,dt)
            
            vx += ax * dt
            vy += ay * dt
            vtheta += az * dt  # Approximate rotational acceleration influence

            # Integrate position using dead reckoning
            self.x += vx * math.cos(self.theta) * dt - vy * math.sin(self.theta) * dt
            self.y += vx * math.sin(self.theta) * dt + vy * math.cos(self.theta) * dt
            self.theta += vtheta * dt

            # Publish odometry & TF
            self.publish_odometry(vx, vy, vtheta, w, x_q, y_q, z_q)
            self.publish_transform()
        except json.JSONDecodeError:
            self.get_logger().warn("Invalid JSON format received.")


    def compute_velocity(self, a, b, c, d, dt):
        """
        Computes robot velocity using wheel encoders.
        """
        # Convert encoder values to wheel velocities
        v_fl = (a * 2*math.pi*self.wheel_radius)/(9000*dt)
        v_fr = (b * 2*math.pi*self.wheel_radius)/(9000*dt)
        v_rl = (c * 2*math.pi*self.wheel_radius)/(9000*dt)
        v_rr = (d * 2*math.pi*self.wheel_radius)/(9000*dt)

        # Compute robot velocity in x, y, and angular velocity
        vx = (v_fl + v_fr + v_rl + v_rr) / 4  # Forward speed
        vy = (-v_fl + v_fr + v_rl - v_rr) / 4  # Sideways speed
        vtheta = (-v_fl + v_fr - v_rl + v_rr) / (4 * (self.robot_width + self.robot_length))  # Angular velocity

        return vx, vy, vtheta

    def publish_odometry(self, vx, vy, vtheta, w, x_q, y_q, z_q):
        """
        Publishes odometry message.
        """
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"

        # Position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Orientation (IMU quaternion)
        odom_msg.pose.pose.orientation = Quaternion(w=w, x=x_q, y=y_q, z=z_q)

        # Velocity
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.angular.z = vtheta

        # Publish odometry
        self.odom_pub.publish(odom_msg)
        self.get_logger().info(f"Odom: x={self.x:.2f}, y={self.y:.2f}, yaw={math.degrees(self.theta):.2f}")

    def publish_transform(self):
        """
        Publishes TF transform from 'odom' to 'base_footprint'.
        """
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        # Set the orientation using IMU quaternion
        t.transform.rotation = Quaternion(w=self.w, x=self.x_q, y=self.y_q, z=self.z_q)

        # Publish TF
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = IMUToOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()