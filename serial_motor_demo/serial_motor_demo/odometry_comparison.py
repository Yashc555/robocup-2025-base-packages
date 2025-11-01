import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import json
import math

class OdomComparison(Node):
    def __init__(self):
        super().__init__('odom_comparison')

        # Subscribe to raw data from /combined_data and EKF output from /odom
        self.raw_data_sub = self.create_subscription(String, '/combined_data', self.raw_data_callback, 10)
        self.ekf_odom_sub = self.create_subscription(Odometry, '/odom', self.ekf_odom_callback, 10)

        # Store trajectories
        self.raw_x, self.raw_y = [0.0], [0.0]  # Unfiltered odometry
        self.ekf_x, self.ekf_y = [0.0], [0.0]  # EKF-filtered odometry

        self.last_encoders = {"a": 0.0, "b": 0.0, "c": 0.0, "d": 0.0}
        self.last_time = self.get_clock().now()

        # Encoder parameters (same as EKF script)
        self.CPR = 9000  # Encoder counts per revolution
        self.R = 0.076  # Wheel radius in meters

        self.timer = self.create_timer(0.1, self.plot_trajectory)  # Plot update timer

    def raw_data_callback(self, msg):
        """ Process raw encoder and IMU data to estimate odometry (before EKF). """
        try:
            parsed_data = json.loads(msg.data)

            # Compute dt
            current_time = self.get_clock().now()
            dt = (current_time - self.last_time).nanoseconds / 1e9
            if dt <= 0:
                return
            self.last_time = current_time

            # Extract encoder values and yaw
            a, b, c, d = parsed_data.get("a", 0.0), parsed_data.get("b", 0.0), parsed_data.get("c", 0.0), parsed_data.get("d", 0.0)
            yaw = math.radians(parsed_data.get("yaw", 0.0))

            # Compute velocity from encoders
            vx, vy, omega = self.compute_velocity(a, b, c, d, dt)

            # Dead reckoning position update
            x_new = self.raw_x[-1] + (vx * math.cos(yaw) - vy * math.sin(yaw)) * dt
            y_new = self.raw_y[-1] + (vx * math.sin(yaw) + vy * math.cos(yaw)) * dt

            self.raw_x.append(x_new)
            self.raw_y.append(y_new)

            # Update last encoder values
            self.last_encoders["a"] = a
            self.last_encoders["b"] = b
            self.last_encoders["c"] = c
            self.last_encoders["d"] = d

        except json.JSONDecodeError:
            self.get_logger().warn("Invalid JSON format received.")

    def ekf_odom_callback(self, msg):
        """ Store EKF-filtered odometry for comparison. """
        self.ekf_x.append(msg.pose.pose.position.x)
        self.ekf_y.append(msg.pose.pose.position.y)

    def compute_velocity(self, a, b, c, d, dt):
        """ Convert encoder differences to velocity components (same as EKF). """
        v_fl = ((a - self.last_encoders["a"]) * 2 * math.pi * self.R) / (self.CPR * dt)
        v_fr = ((b - self.last_encoders["b"]) * 2 * math.pi * self.R) / (self.CPR * dt)
        v_rl = ((c - self.last_encoders["c"]) * 2 * math.pi * self.R) / (self.CPR * dt)
        v_rr = ((d - self.last_encoders["d"]) * 2 * math.pi * self.R) / (self.CPR * dt)

        vx = (v_fl + v_fr + v_rl + v_rr) / 4  # Linear velocity x
        vy = (-v_fl + v_fr + v_rl - v_rr) / 4  # Linear velocity y
        omega = (-v_fl + v_fr - v_rl + v_rr) / (4 * self.R)  # Angular velocity

        return vx, vy, omega

    def plot_trajectory(self):
        """ Live update of the trajectory plot. """
        if len(self.raw_x) > 5 and len(self.ekf_x) > 5:
            plt.clf()
            plt.plot(self.raw_x, self.raw_y, label='Raw Odometry (Dead Reckoning)', linestyle='dashed', color='red')
            plt.plot(self.ekf_x, self.ekf_y, label='EKF Odometry', linestyle='solid', color='blue')
            plt.xlabel('X Position (m)')
            plt.ylabel('Y Position (m)')
            plt.title('Odometry Comparison: Before vs After EKF')
            plt.legend()
            plt.pause(0.1)  # Non-blocking update

def main(args=None):
    rclpy.init(args=args)
    node = OdomComparison()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        plt.show()  # Keep the final plot open

if __name__ == '__main__':
    main()
