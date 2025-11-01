#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import json
import time

try:
    import serial
except Exception:
    serial = None

class MecanumClosedLoopControl(Node):
    def __init__(self):
        super().__init__('mecanum_closedloop_control')

        # kinematic constants (meters)
        self.L = float(self.declare_parameter('L', 0.32).value)
        self.W = float(self.declare_parameter('W', 0.24).value)
        self.R = float(self.declare_parameter('R', 0.05).value)

        # pwm / scaling and control gains
        self.max_pwm = int(self.declare_parameter('max_pwm', 255).value)
        self.scale_factor = float(self.declare_parameter('scale_factor', 100.0).value)
        self.Kp_vx = float(self.declare_parameter('Kp_vx', 1.0).value)
        self.Kp_vy = float(self.declare_parameter('Kp_vy', 1.0).value)
        self.Kp_wz = float(self.declare_parameter('Kp_wz', 1.0).value)

        # topics and serial config
        odom_topic = self.declare_parameter('odom_topic', '/odom').value
        cmd_topic = self.declare_parameter('cmd_vel_topic', '/cmd_vel').value
        serial_port_name = self.declare_parameter('serial_port', '/dev/ttyACM0').value
        baud = int(self.declare_parameter('serial_baud', 115200).value)

        # safety and timing
        self.send_hz = float(self.declare_parameter('send_hz', 20.0).value)
        self.cmd_timeout = float(self.declare_parameter('cmd_timeout', 0.5).value)

        # internal state
        self.desired_vx = 0.0
        self.desired_vy = 0.0
        self.desired_wz = 0.0
        self.last_cmd_stamp = 0.0

        self.actual_vx = 0.0
        self.actual_vy = 0.0
        self.actual_wz = 0.0
        self.have_odom = False

        # serial setup
        self.serial_port = None
        if serial is not None:
            try:
                self.serial_port = serial.Serial(serial_port_name, baud, timeout=1)
                self.get_logger().info(f"Opened serial {serial_port_name} @ {baud}")
            except Exception as e:
                self.get_logger().warn(f"Could not open serial {serial_port_name}: {e}")
                self.serial_port = None
        else:
            self.get_logger().warn("pyserial not available. Install pyserial to enable serial comms.")

        # subscriptions
        self.create_subscription(Twist, cmd_topic, self.cmd_vel_cb, 10)
        self.create_subscription(Odometry, odom_topic, self.odom_cb, 20)

        # timer to send PWM at fixed rate
        self.create_timer(1.0 / self.send_hz, self.timer_send_pwm)

        self.get_logger().info("Mecanum closed-loop controller started")

    def cmd_vel_cb(self, msg: Twist):
        self.desired_vx = float(msg.linear.x)
        self.desired_vy = float(msg.linear.y)
        self.desired_wz = float(msg.angular.z)
        self.last_cmd_stamp = self.get_clock().now().nanoseconds * 1e-9

    def odom_cb(self, msg: Odometry):
        # nav_msgs/Odometry.twist.twist is usually expressed in the robot base frame
        self.actual_vx = float(msg.twist.twist.linear.x)
        self.actual_vy = float(msg.twist.twist.linear.y)
        self.actual_wz = float(msg.twist.twist.angular.z)
        self.have_odom = True

    def compute_corrected_vel(self):
        # If cmd is stale, stop
        now = self.get_clock().now().nanoseconds * 1e-9
        if (now - self.last_cmd_stamp) > self.cmd_timeout:
            return 0.0, 0.0, 0.0

        # If we do not have odom yet, just use open-loop desired velocities
        if not self.have_odom:
            return self.desired_vx, self.desired_vy, self.desired_wz

        # error = desired - actual
        err_vx = self.desired_vx - self.actual_vx
        err_vy = self.desired_vy - self.actual_vy
        err_wz = self.desired_wz - self.actual_wz

        # P-control: new_command = desired + Kp * error
        vx_cmd = self.desired_vx + self.Kp_vx * err_vx
        vy_cmd = self.desired_vy + self.Kp_vy * err_vy
        wz_cmd = self.desired_wz + self.Kp_wz * err_wz

        return vx_cmd, vy_cmd, wz_cmd

    def kinematic_to_wheel_pwms(self, vx, vy, wz):
        # Standard mecanum kinematic (robot frame)
        LpW = (self.L + self.W)
        wheel_speeds = [
            vx - vy - wz * LpW,  # Front Left  (w0)
            vx + vy + wz * LpW,  # Front Right (w1)
            vx + vy - wz * LpW,  # Rear Left   (w2)
            vx - vy + wz * LpW   # Rear Right  (w3)
        ]
        # Scale to PWM. scale_factor should map m/s to PWM units; tune this.
        pwms = [int(speed * self.scale_factor) for speed in wheel_speeds]
        # Clamp
        pwms = [self.clamp(p, -self.max_pwm, self.max_pwm) for p in pwms]
        return pwms

    def timer_send_pwm(self):
        vx_cmd, vy_cmd, wz_cmd = self.compute_corrected_vel()

        # If near zero, send zeros
        if abs(vx_cmd) < 0.001 and abs(vy_cmd) < 0.001 and abs(wz_cmd) < 0.001:
            pwm_values = [0, 0, 0, 0]
        else:
            pwm_values = self.kinematic_to_wheel_pwms(vx_cmd, vy_cmd, wz_cmd)

        # Map to your Arduino ordering (kept same as your original mapping)
        pwm_message = json.dumps({
            "pwm1": pwm_values[3],  # Rear Right
            "pwm2": pwm_values[1],  # Front Right
            "pwm3": pwm_values[0],  # Front Left
            "pwm4": pwm_values[2]   # Rear Left
        })

        self.get_logger().debug(f"desired: ({self.desired_vx:.3f},{self.desired_vy:.3f},{self.desired_wz:.3f}) "
                                f"actual: ({self.actual_vx:.3f},{self.actual_vy:.3f},{self.actual_wz:.3f}) "
                                f"cmd: ({vx_cmd:.3f},{vy_cmd:.3f},{wz_cmd:.3f}) "
                                f"pwms: {pwm_values}")

        if self.serial_port:
            try:
                self.serial_port.write((pwm_message + "\n").encode())
            except Exception as e:
                self.get_logger().warn(f"Serial write failed: {e}")

        else:
            # useful for simulation / dry-run
            self.get_logger().info(f"Send PWM (closed-loop): {pwm_message}")

    def clamp(self, value, min_v, max_v):
        return max(min(value, max_v), min_v)

    def destroy_node(self):
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.close()
            except Exception:
                pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MecanumClosedLoopControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
