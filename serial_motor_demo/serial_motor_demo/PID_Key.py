import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import json
import time
import serial

class PIDController:
    def __init__(self, kp, ki, kd, max_output=150, min_output=-150):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.min_output = min_output
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, error, dt):
        if dt < 0.001:
            return 0
        self.integral += error * dt
        if abs(error) < 0.01:
            self.integral = 0.0
        derivative = (error - self.prev_error) / dt
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        output = max(min(output, self.max_output), self.min_output)
        self.prev_error = error
        return int(output)

class MecanumPIDControl(Node):
    def __init__(self):
        super().__init__('mecanum_pid_control')

        self.pid_fl = PIDController(7.0, 0.2, 0.05)
        self.pid_fr = PIDController(7.0, 0.2, 0.05)
        self.pid_rl = PIDController(7.0, 0.2, 0.05)
        self.pid_rr = PIDController(7.0, 0.2, 0.05)

        self.L, self.W, self.R, self.CPR = 0.32, 0.24, 0.05, 9000
        self.last_time = time.time()
        self.last_encoders = {"a": 0, "b": 0, "c": 0, "d": 0}
        self.actual_wheel_speeds = [0.0, 0.0, 0.0, 0.0]

        try:
            self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            self.get_logger().info("Serial connection established on /dev/ttyACM0")
        except serial.SerialException:
            self.get_logger().error("Failed to open serial port /dev/ttyACM0")
            self.serial_port = None

        self.create_subscription(Twist, 'cmd_vel', self.twist_callback, 10)
        self.create_subscription(String, '/combined_data', self.encoder_callback, 10)

    def twist_callback(self, msg: Twist):
        vx, vy, wz = msg.linear.x, msg.linear.y, msg.angular.z
        if abs(vx) < 0.01 and abs(vy) < 0.01 and abs(wz) < 0.01:
            pwm_values = [0, 0, 0, 0]
        else:
            desired_wheel_speeds = self.calculate_wheel_speeds(vx, vy, wz)
            pwm_values = self.compute_pid_pwm(desired_wheel_speeds, self.actual_wheel_speeds)
        
        pwm_message = json.dumps({"pwm1": pwm_values[0], "pwm2": pwm_values[1], "pwm3": pwm_values[2], "pwm4": pwm_values[3]})
        self.get_logger().info(f"Sent PWM: {pwm_message}")

        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write((pwm_message + "\n").encode())
            except serial.SerialException:
                self.get_logger().error("Failed to send data over serial port")

    def encoder_callback(self, msg: String):
        try:
            parsed_data = json.loads(msg.data)
            a, b, c, d = parsed_data.get("a", 0), parsed_data.get("b", 0), parsed_data.get("c", 0), parsed_data.get("d", 0)
            self.get_logger().info(f"Raw Encoder Values -> a: {a}, b: {b}, c: {c}, d: {d}")
            dt = max(time.time() - self.last_time, 0.001)
            self.last_time = time.time()
            self.actual_wheel_speeds = self.compute_velocity(a, b, c, d, dt)
        except json.JSONDecodeError:
            self.get_logger().warn("Invalid JSON format received in /combined_data.")

    def compute_velocity(self, a, b, c, d, dt):
        v_fl = ((a - self.last_encoders["a"]) * 2 * 3.14159 * self.R) / (self.CPR * dt)
        v_fr = ((b - self.last_encoders["b"]) * 2 * 3.14159 * self.R) / (self.CPR * dt)
        v_rl = ((c - self.last_encoders["c"]) * 2 * 3.14159 * self.R) / (self.CPR * dt)
        v_rr = ((d - self.last_encoders["d"]) * 2 * 3.14159 * self.R) / (self.CPR * dt)

        self.last_encoders = {"a": a, "b": b, "c": c, "d": d}
        return [v_fl, v_fr, v_rl, v_rr]

    def calculate_wheel_speeds(self, vx, vy, wz):
        return [
            (vx - vy - wz * (self.L + self.W)) / self.R,
            (vx + vy + wz * (self.L + self.W)) / self.R,
            (vx + vy - wz * (self.L + self.W)) / self.R,
            (vx - vy + wz * (self.L + self.W)) / self.R
        ]

    def compute_pid_pwm(self, desired_speeds, actual_speeds):
        dt = max(time.time() - self.last_time, 0.001)
        self.last_time = time.time()
        return [
            self.pid_fl.compute(desired_speeds[0] - actual_speeds[0], dt),
            self.pid_fr.compute(desired_speeds[1] - actual_speeds[1], dt),
            self.pid_rl.compute(desired_speeds[2] - actual_speeds[2], dt),
            self.pid_rr.compute(desired_speeds[3] - actual_speeds[3], dt)
        ]

def main(args=None):
    rclpy.init(args=args)
    node = MecanumPIDControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
