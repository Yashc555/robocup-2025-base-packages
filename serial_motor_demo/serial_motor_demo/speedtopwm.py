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
        
        # Initial PWM values
        self.left_pwm = 0
        self.right_pwm = 0
        self.left_direction = 0  # 0 for forward, 1 for reverse
        self.right_direction = 0  # 0 for forward, 1 for reverse
        
        # Create a subscriber to the /cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def cmd_vel_callback(self, msg):
        # Map linear and angular velocities to PWM values for each motor
        linear_vel = msg.linear.x  # Forward/backward
        angular_vel = msg.angular.z  # Left/right turn

        # Convert the linear and angular velocities to left and right PWM
        max_pwm = 255
        base_pwm = abs(int(linear_vel * 100))  # Base PWM based on forward/reverse speed
        
        # Determine direction and PWM for left motor
        if linear_vel - angular_vel >= 0:
            self.left_direction = 0
            self.left_pwm = min(base_pwm - int(angular_vel * 50), max_pwm)
        else:
            self.left_direction = 1
            self.left_pwm = min(base_pwm - int(angular_vel * 50), max_pwm)

        # Determine direction and PWM for right motor
        if linear_vel + angular_vel >= 0:
            self.right_direction = 0
            self.right_pwm = min(base_pwm + int(angular_vel * 50), max_pwm)
        else:
            self.right_direction = 1
            self.right_pwm = min(base_pwm + int(angular_vel * 50), max_pwm)

        # Send the PWM and direction values to the Arduino
        self.send_pwm_to_arduino(self.left_pwm, self.right_pwm, self.left_direction, self.right_direction)

    def send_pwm_to_arduino(self, left_pwm, right_pwm, left_direction, right_direction):
        # Format the command to include both PWM values and directions
        command = f"{left_pwm},{right_pwm},{left_direction},{right_direction}\n"
        arduino_port.write(command.encode('utf-8'))
        self.get_logger().info(f"Sent PWM: Left={left_pwm} (Dir={left_direction}), Right={right_pwm} (Dir={right_direction})")

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
