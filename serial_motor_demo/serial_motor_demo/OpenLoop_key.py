import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import json
import time
import serial

class MecanumOpenLoopControl(Node):
    def __init__(self):
        super().__init__('mecanum_openloop_control')

        self.L, self.W, self.R = 0.32, 0.24, 0.05
        self.max_pwm = 255  # maximum allowed PWM value

        try:
            self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            self.get_logger().info("Serial connection established on /dev/ttyACM0")
        except serial.SerialException:
            self.get_logger().error("Failed to open serial port /dev/ttyACM0")
            self.serial_port = None

        self.create_subscription(Twist, 'cmd_vel', self.twist_callback, 10)

    def twist_callback(self, msg: Twist):
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        if abs(vx) < 0.01 and abs(vy) < 0.01 and abs(wz) < 0.01:
            pwm_values = [0, 0, 0, 0]
        else:
            # Compute open-loop speeds (unitless) and scale to PWM
            wheel_speeds = [
                vx - vy - wz * (self.L + self.W),  # Front Left
                vx + vy + wz * (self.L + self.W),  # Front Right
                vx + vy - wz * (self.L + self.W),  # Rear Left
                vx - vy + wz * (self.L + self.W)   # Rear Right
            ]

            # Scale to PWM range
            scale_factor = 100  # tune this as needed
            pwm_values = [self.clamp(int(speed * scale_factor), -self.max_pwm, self.max_pwm) for speed in wheel_speeds]

        # Create the PWM message for Arduino or motor controller
        pwm_message = json.dumps({"pwm1": pwm_values[3], "pwm2": pwm_values[1], "pwm3": pwm_values[0], "pwm4": pwm_values[2]})

        self.get_logger().info(f"Sent PWM (Open-loop): {pwm_message}")

        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write((pwm_message + "\n").encode())
            except serial.SerialException:
                self.get_logger().error("Failed to send data over serial port")

    def clamp(self, value, min_value, max_value):
        return max(min(value, max_value), min_value)

    def destroy_node(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MecanumOpenLoopControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()