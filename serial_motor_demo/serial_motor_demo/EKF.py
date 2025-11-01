import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import numpy as np
import math
from filterpy.kalman import ExtendedKalmanFilter as EKF

class EKFOdometry(Node):
    def __init__(self):
        super().__init__('ekf_odometry')
        self.imu_sub = self.create_subscription(String, '/combined_data', self.process_imu_data, 1000)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 1000)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.last_time = self.get_clock().now()

        self.ekf = EKF(dim_x=5, dim_z=4)
        self.ekf.x = np.zeros(5)
        self.ekf.P *= 1.0
        self.ekf.Q = np.diag([0.01, 0.01, 0.01, 0.05, 0.05])
        self.ekf.R = np.diag([0.05, 0.05, 0.02, 0.02])        

        self.CPR_A = 8650
        self.CPR_B = 8636
        self.CPR_C = 6746
        self.CPR_D = 8641
        self.R = 0.076
        self.L = 0.355
        self.W = 0.245

        self.last_encoders = {"a": 0.0, "b": 0.0, "c": 0.0, "d": 0.0}

        self.cumulative_distance = 0.0
        self.prev_pos = np.array([0.0, 0.0])

    def process_imu_data(self, msg):
        try:
            parsed = json.loads(msg.data)
            now = self.get_clock().now()
            dt = (now - self.last_time).nanoseconds / 1e9
            if dt <= 0 or dt > 5.0:  # Avoid processing if dt is too small or too large
                return

            a, b, c, d = parsed.get("a", 0.0), parsed.get("b", 0.0), parsed.get("c", 0.0), parsed.get("d", 0.0)
            qx, qy, qz, qw = parsed.get("x", 0.0), parsed.get("y", 0.0), parsed.get("z", 0.0), parsed.get("w", 1.0)
            imu_yaw = self.quaternion_to_yaw(qx, qy, qz, qw)
            
            self.last_time = now
            
            if not hasattr(self, 'initialized'):
                self.ekf.x[2] = imu_yaw
                self.initialized = True

            vx, vy, omega = self.compute_velocity(a, b, c, d, dt)
            self.get_logger().info(f"vx: {vx:.4f}, vy: {vy:.4f}, omega: {omega:.4f}")
            self.get_logger().info(f"IMU Yaw: {imu_yaw:.4f}")

            self.ekf.F = self.compute_F_matrix(dt)
            self.ekf.predict()

            theta = self.ekf.x[2]
            self.ekf.x[0] += (vx * math.cos(theta) - vy * math.sin(theta)) * dt
            self.ekf.x[1] += (vx * math.sin(theta) + vy * math.cos(theta)) * dt
            self.ekf.x[2] += omega * dt
            self.ekf.x[2] = math.atan2(math.sin(self.ekf.x[2]), math.cos(self.ekf.x[2]))

            z = np.array([vx, vy, omega, imu_yaw])
            self.ekf.update(z, self.H_jacobian, self.measurement_function)
            
            current_pos = np.array([self.ekf.x[0], self.ekf.x[1]])
            step_distance = np.linalg.norm(current_pos - self.prev_pos)
            self.cumulative_distance += step_distance
            self.prev_pos = current_pos
            self.get_logger().info(f"Cumulative distance traveled: {self.cumulative_distance:.4f} meters")


            # ✅ Cumulative distance tracking
            # step_distance = math.sqrt(vx**2 + vy**2) * dt
            # self.cumulative_distance += step_distance
            # self.get_logger().info(f"Cumulative distance traveled: {self.cumulative_distance:.4f} meters")


            timestamp = now.to_msg()
            self.publish_odometry(timestamp)
            self.publish_transform(timestamp)

            self.last_encoders = {"a": -a, "b": b, "c": c, "d": -d}

        except Exception as e:
            self.get_logger().error(f"Error processing data: {e}")

    def compute_velocity(self, a, b, c, d, dt):
        a, d = -a, -d
        v_rr = ((a - self.last_encoders["a"]) * 2 * math.pi * self.R) / (self.CPR_A * dt)
        v_fr = ((b - self.last_encoders["b"]) * 2 * math.pi * self.R) / (self.CPR_B * dt)
        v_fl = ((c - self.last_encoders["c"]) * 2 * math.pi * self.R) / (self.CPR_C * dt)
        v_rl = ((d - self.last_encoders["d"]) * 2 * math.pi * self.R) / (self.CPR_D * dt)

        vx = (v_fl + v_fr + v_rl + v_rr) / 4
        vy = (-v_fl + v_fr + v_rl - v_rr) / 4
        omega = (-v_fl + v_fr - v_rl + v_rr) / (4 * (self.L + self.W))
        return vx, vy, omega

    def compute_F_matrix(self, dt):
        F = np.eye(5)
        F[0, 3] = dt
        F[1, 4] = dt
        return F

    def measurement_function(self, x):
        return np.array([x[3], x[4], x[2], x[2]])

    def H_jacobian(self, x):
        H = np.zeros((4, 5))
        H[0, 3] = 1
        H[1, 4] = 1
        H[2, 2] = 1
        H[3, 2] = 1
        return H

    def quaternion_to_yaw(self, x, y, z, w):
        _, _, yaw = euler_from_quaternion([x, y, z, w])
        return yaw

    def publish_odometry(self, timestamp):
        theta = self.ekf.x[2]
        qx, qy, qz, qw = quaternion_from_euler(0, 0, theta)

        odom = Odometry()
        odom.header.stamp = timestamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x = self.ekf.x[0]
        odom.pose.pose.position.y = self.ekf.x[1]
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.ekf.x[3]
        odom.twist.twist.linear.y = self.ekf.x[4]
        self.odom_pub.publish(odom)

    def publish_transform(self, timestamp):
        theta = self.ekf.x[2]
        qx, qy, qz, qw = quaternion_from_euler(0, 0, theta)

        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = self.ekf.x[0]
        t.transform.translation.y = self.ekf.x[1]
        t.transform.translation.z = 0.0
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = EKFOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
