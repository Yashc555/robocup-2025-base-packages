import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import math

class SensorProcessor(Node):
    def __init__(self):
        super().__init__('sensor_processor')
        self.subscription = self.create_subscription(String, '/combined_data', self.process_data, 1000)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 1000)
        self.odom_pub = self.create_publisher(Odometry, '/wheel_odom', 1000)
        self.CPR_A = 8650
        self.CPR_B = 8636
        self.CPR_C = 6746
        self.CPR_D = 8641
        self.R = 0.076  # Wheel radius
        self.L = 0.355  # Wheelbase length
        self.W = 0.245  # Wheelbase width
        self.last_encoders = {"a": 0.0, "b": 0.0, "c": 0.0, "d": 0.0}
        self.last_time = self.get_clock().now()

    def process_data(self, msg):
        try:
            parsed_data = json.loads(msg.data)
            current_time = self.get_clock().now()
            dt = (current_time - self.last_time).nanoseconds / 1e9
            if dt <= 0:
                return
            self.last_time = current_time

            # Process IMU Data
            imu_msg = Imu()
            imu_msg.header.stamp = current_time.to_msg()
            imu_msg.header.frame_id = "base_footprint"
            imu_msg.orientation = Quaternion(
                w=parsed_data.get("w", 1.0),
                x=parsed_data.get("x", 0.0),
                y=parsed_data.get("y", 0.0),
                z=parsed_data.get("z", 0.0)
            )
            # Required for robot_localization to accept IMU orientation
            imu_msg.orientation_covariance[0] = 0.01
            imu_msg.orientation_covariance[4] = 0.01
            imu_msg.orientation_covariance[8] = 0.01
            self.imu_pub.publish(imu_msg)

            # Process Encoder Data
            a, b, c, d = parsed_data.get("a", 0.0), parsed_data.get("b", 0.0), parsed_data.get("c", 0.0), parsed_data.get("d", 0.0)
            a, d = -a, -d
            vx, vy, omega = self.compute_velocity(a, b, c, d, dt)
            
            odom_msg = Odometry()
            odom_msg.header.stamp = current_time.to_msg()
            odom_msg.header.frame_id = "odom"
            odom_msg.child_frame_id = "base_footprint"
            odom_msg.twist.twist.linear.x = vx
            odom_msg.twist.twist.linear.y = vy
            odom_msg.twist.twist.angular.z = omega
            # Required for EKF to use twist
            odom_msg.twist.covariance[0] = 0.02
            odom_msg.twist.covariance[7] = 0.02
            odom_msg.twist.covariance[35] = 0.05
            self.odom_pub.publish(odom_msg)
            
            self.last_encoders = {"a": a, "b": b, "c": c, "d": d}

        except json.JSONDecodeError:
            self.get_logger().error("Failed to parse JSON data.")
        except Exception as e:
            self.get_logger().error(f"Error processing data: {e}")

    def compute_velocity(self, a, b, c, d, dt):
        v_fl = ((a - self.last_encoders["a"]) * 2 * math.pi * self.R) / (self.CPR_A * dt)
        v_fr = ((b - self.last_encoders["b"]) * 2 * math.pi * self.R) / (self.CPR_B * dt)
        v_rl = ((c - self.last_encoders["c"]) * 2 * math.pi * self.R) / (self.CPR_C * dt)
        v_rr = ((d - self.last_encoders["d"]) * 2 * math.pi * self.R) / (self.CPR_D * dt)
        vx = (v_fl + v_fr + v_rl + v_rr) / 4
        vy = (-v_fl + v_fr + v_rl - v_rr) / 4
        omega = (-v_fl + v_fr - v_rl + v_rr) / (4 * (self.L + self.W))
        return vx, vy, omega


def main(args=None):
    rclpy.init(args=args)
    node = SensorProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import math

class SensorProcessor(Node):
    def __init__(self):
        super().__init__('sensor_processor')
        self.subscription = self.create_subscription(String, '/combined_data', self.process_data, 1000)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 1000)
        self.odom_pub = self.create_publisher(Odometry, '/wheel_odom', 1000)
        self.CPR_A = 8650
        self.CPR_B = 8636
        self.CPR_C = 6746
        self.CPR_D = 8641
        self.R = 0.076  # Wheel radius
        self.L = 0.355  # Wheelbase length
        self.W = 0.245  # Wheelbase width
        self.last_encoders = {"a": 0.0, "b": 0.0, "c": 0.0, "d": 0.0}
        self.last_time = self.get_clock().now()

    def process_data(self, msg):
        try:
            parsed_data = json.loads(msg.data)
            current_time = self.get_clock().now()
            dt = (current_time - self.last_time).nanoseconds / 1e9
            if dt <= 0:
                return
            self.last_time = current_time

            # Process IMU Data
            imu_msg = Imu()
            imu_msg.header.stamp = current_time.to_msg()
            imu_msg.header.frame_id = "base_footprint"
            imu_msg.orientation = Quaternion(
                w=parsed_data.get("w", 1.0),
                x=parsed_data.get("x", 0.0),
                y=parsed_data.get("y", 0.0),
                z=parsed_data.get("z", 0.0)
            )
            # Required for robot_localization to accept IMU orientation
            imu_msg.orientation_covariance[0] = 0.01
            imu_msg.orientation_covariance[4] = 0.01
            imu_msg.orientation_covariance[8] = 0.01
            self.imu_pub.publish(imu_msg)

            # Process Encoder Data
            a, b, c, d = parsed_data.get("a", 0.0), parsed_data.get("b", 0.0), parsed_data.get("c", 0.0), parsed_data.get("d", 0.0)
            a, d = -a, -d
            vx, vy, omega = self.compute_velocity(a, b, c, d, dt)
            
            odom_msg = Odometry()
            odom_msg.header.stamp = current_time.to_msg()
            odom_msg.header.frame_id = "odom"
            odom_msg.child_frame_id = "base_footprint"
            odom_msg.twist.twist.linear.x = vx
            odom_msg.twist.twist.linear.y = vy
            odom_msg.twist.twist.angular.z = omega
            # Required for EKF to use twist
            odom_msg.twist.covariance[0] = 0.02
            odom_msg.twist.covariance[7] = 0.02
            odom_msg.twist.covariance[35] = 0.05
            self.odom_pub.publish(odom_msg)
            
            self.last_encoders = {"a": a, "b": b, "c": c, "d": d}

        except json.JSONDecodeError:
            self.get_logger().error("Failed to parse JSON data.")
        except Exception as e:
            self.get_logger().error(f"Error processing data: {e}")

    def compute_velocity(self, a, b, c, d, dt):
        v_fl = ((a - self.last_encoders["a"]) * 2 * math.pi * self.R) / (self.CPR_A * dt)
        v_fr = ((b - self.last_encoders["b"]) * 2 * math.pi * self.R) / (self.CPR_B * dt)
        v_rl = ((c - self.last_encoders["c"]) * 2 * math.pi * self.R) / (self.CPR_C * dt)
        v_rr = ((d - self.last_encoders["d"]) * 2 * math.pi * self.R) / (self.CPR_D * dt)
        vx = (v_fl + v_fr + v_rl + v_rr) / 4
        vy = (-v_fl + v_fr + v_rl - v_rr) / 4
        omega = (-v_fl + v_fr - v_rl + v_rr) / (4 * (self.L + self.W))
        return vx, vy, omega


def main(args=None):
    rclpy.init(args=args)
    node = SensorProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
