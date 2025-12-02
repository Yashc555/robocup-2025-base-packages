import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import serial

class MecanumJoystickControl(Node):
    def __init__(self):
        super().__init__('mecanum_joystick_control')

        # Open Serial Port
        self.ser = serial.Serial('/dev/ttyACM0', 115200)

        # Subscribe to joystick messages
        self.joy_subscriber = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        # Robot Dimensions (Adjust for your bot)
        self.L = 0.32  # Half-length (meters)
        self.W = 0.24  # Half-width (meters)
        self.R = 0.05  # Wheel radius (meters)

        # Scaling parameters
        self.MAX_VELOCITY = 1.0  # Max joystick velocity
        self.MAX_PWM = 150  # Max PWM value

        # Joystick axis mapping (adjust based on your joystick layout)
        self.axis_linear_x = 1  # Left stick Y-axis (forward/backward)
        self.axis_linear_y = 0  # Left stick X-axis (strafe left/right)
        self.axis_angular_z = 2  # Right stick X-axis (rotate)

    def joy_callback(self, msg):
        # Extract joystick values
        vx = self.MAX_VELOCITY * msg.axes[self.axis_linear_x]  # Forward/backward
        vy = self.MAX_VELOCITY * msg.axes[self.axis_linear_y]  # Left/right
        wz = self.MAX_VELOCITY * msg.axes[self.axis_angular_z]  # Rotation

        # Debug: Print received joystick values
        self.get_logger().info(f"Joystick Input: vx={vx}, vy={vy}, wz={wz}")

        # Compute PWM values for each wheel
        pwm_values = self.calculate_pwm(vx, vy, wz)

        # Debug: Print PWM values
        self.get_logger().info(f"Sending PWM: {pwm_values[0]},{pwm_values[1]},{pwm_values[2]},{pwm_values[3]},A")

        # Format message and send over serial
        pwm_message = f"{pwm_values[0]},{pwm_values[1]},{pwm_values[2]},{pwm_values[3]},A"
        self.ser.write(pwm_message.encode())

    def calculate_pwm(self, vx, vy, wz):
        """ Convert joystick velocity to PWM values """
        wheel1 = (vx - vy - wz * (self.L + self.W)) / self.R
        wheel2 = (vx + vy + wz * (self.L + self.W)) / self.R
        wheel3 = (vx + vy - wz * (self.L + self.W)) / self.R
        wheel4 = (vx - vy + wz * (self.L + self.W)) / self.R

        # Scale to PWM range
        pwm_front_left = self.map_to_pwm(wheel1)
        pwm_front_right = self.map_to_pwm(wheel2)
        pwm_back_left = self.map_to_pwm(wheel3)
        pwm_back_right = self.map_to_pwm(wheel4)

        return [pwm_front_left, pwm_front_right, pwm_back_left, pwm_back_right]

    def map_to_pwm(self, value):
        """ Map velocity to PWM range """
        max_velocity = self.MAX_VELOCITY / self.R
        pwm_value = int((value / max_velocity) * self.MAX_PWM)

        # Clamp PWM values within range
        pwm_value = max(min(pwm_value, self.MAX_PWM), -self.MAX_PWM)
        return pwm_value

def main(args=None):
    rclpy.init(args=args)
    node = MecanumJoystickControl()
    rclpy.spin(node)

    # Cleanup on exit
    node.ser.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
