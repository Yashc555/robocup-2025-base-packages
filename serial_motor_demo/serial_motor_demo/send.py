import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

# Initialize the serial connection to Arduino
arduino_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
time.sleep(2)  # Wait for the connection to establish

class MotorControl(Node):
    def __init__(self):
        super().__init__('motor_control')

        # Initial PWM and direction values for four wheels
        self.pwm_fl = 0  # Front Left
        self.pwm_fr = 0  # Front Right
        self.pwm_rl = 0  # Rear Left
        self.pwm_rr = 0  # Rear Right
        self.dir_fl = 1  # Direction Front Left
        self.dir_fr = 1  # Direction Front Right
        self.dir_rl = 1  # Direction Rear Left
        self.dir_rr = 1  # Direction Rear Right

        # Create a subscriber to the /cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def cmd_vel_callback(self, msg):
        # Extract linear and angular velocities
        linear_x = msg.linear.x   # Forward/backward
        linear_y = msg.linear.y   # Left/right strafing
        angular_z = msg.angular.z  # Rotation

        # Maximum PWM
        max_pwm = 255

        # Calculate PWM values for mecanum wheels
        self.pwm_fl = linear_x - linear_y - angular_z
        self.pwm_fr = linear_x + linear_y + angular_z
        self.pwm_rl = linear_x + linear_y - angular_z
        self.pwm_rr = linear_x - linear_y + angular_z

        # Determine directions based on sign of PWM
        self.dir_fl = 1 if self.pwm_fl >= 0 else 0
        self.dir_fr = 1 if self.pwm_fr >= 0 else 0
        self.dir_rl = 1 if self.pwm_rl >= 0 else 0
        self.dir_rr = 1 if self.pwm_rr >= 0 else 0

        # Take the absolute values of PWM
        self.pwm_fl = abs(self.pwm_fl)
        self.pwm_fr = abs(self.pwm_fr)
        self.pwm_rl = abs(self.pwm_rl)
        self.pwm_rr = abs(self.pwm_rr)

        # Normalize PWM if any exceeds max_pwm
        max_wheel_speed = max(self.pwm_fl, self.pwm_fr, self.pwm_rl, self.pwm_rr)
        if max_wheel_speed > max_pwm:
            scale = max_pwm / max_wheel_speed
            self.pwm_fl *= scale
            self.pwm_fr *= scale
            self.pwm_rl *= scale
            self.pwm_rr *= scale

        # Convert PWM values to integers
        self.pwm_fl = int(self.pwm_fl)
        self.pwm_fr = int(self.pwm_fr)
        self.pwm_rl = int(self.pwm_rl)
        self.pwm_rr = int(self.pwm_rr)

        # Send the command to Arduino
        self.send_pwm_to_arduino()

    def send_pwm_to_arduino(self):
        # Format the command string
        command = (
            f"Dir{self.dir_fl},{self.pwm_fl},"
            f"Dir{self.dir_fr},{self.pwm_fr},"
            f"Dir{self.dir_rl},{self.pwm_rl},"
            f"Dir{self.dir_rr},{self.pwm_rr}\n"
        )
        # Send the command via serial
        arduino_port.write(command.encode('utf-8'))
        self.get_logger().info(f"Sent Command: {command}")

    def destroy_node(self):
        arduino_port.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    motor_control = MotorControl()

    try:
        rclpy.spin(motor_control)
    except KeyboardInterrupt:
        print("Exiting motor control.")
    finally:
        motor_control.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
