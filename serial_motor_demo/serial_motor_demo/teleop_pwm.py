import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class MecanumControl(Node):
    def __init__(self):
        super().__init__('mecanum_control')
        
        self.ser = serial.Serial('/dev/ttyACM1', 115200)

        self.twist_subscriber = self.create_subscription(
            Twist,
            'cmd_vel',
            self.twist_callback,
            10
        )

    def twist_callback(self, msg: Twist):
        vx = msg.linear.x 
        vy = msg.linear.y 
        wz = msg.angular.z 

        # Print received Twist values for debugging
        self.get_logger().info(f"Received cmd_vel: vx={vx}, vy={vy}, wz={wz}")

        pwm_values = self.calculate_pwm(vx, vy, wz)

        # Print calculated PWM values
        self.get_logger().info(f"Sending PWM: {pwm_values[0]},{pwm_values[1]},{pwm_values[2]},{pwm_values[3]},A")

        pwm_message = f"{pwm_values[0]},{pwm_values[1]},{pwm_values[2]},{pwm_values[3]},A"
        self.ser.write(pwm_message.encode())


    def calculate_pwm(self, vx, vy, wz):
        # Define robot dimensions (adjust as per your robot)
        L = 0.32  # Distance from center to wheel in meters
        W = 0.24  # Distance from center to wheel in meters
        R = 0.05  # Wheel radius in meters (example: 5cm)

        # Scaling factor to convert velocity to PWM range (adjust as needed)
        MAX_VELOCITY = 1.0  # Max velocity from teleop (adjust as needed)
        MAX_PWM = 150

        # Compute raw wheel speeds (convert linear velocity to angular velocity)
        wheel1 = (vx - vy - wz * (L + W)) / R
        wheel2 = (vx + vy + wz * (L + W)) / R
        wheel3 = (vx + vy - wz * (L + W)) / R
        wheel4 = (vx - vy + wz * (L + W)) / R

        # Scale to PWM range
        pwm_front_left = self.map_to_pwm(wheel1, MAX_VELOCITY / R, MAX_PWM)
        pwm_front_right = self.map_to_pwm(wheel2, MAX_VELOCITY / R, MAX_PWM)
        pwm_back_left = self.map_to_pwm(wheel3, MAX_VELOCITY / R, MAX_PWM)
        pwm_back_right = self.map_to_pwm(wheel4, MAX_VELOCITY / R, MAX_PWM)

        return [pwm_front_left, pwm_front_right, pwm_back_left, pwm_back_right]


    def map_to_pwm(self, value, max_velocity, max_pwm):
        # Convert velocity to PWM range (-max_velocity to max_velocity) -> (-max_pwm to max_pwm)
        pwm_value = int((value / max_velocity) * max_pwm)

        # Ensure PWM stays within valid range
        pwm_value = max(min(pwm_value, max_pwm), -max_pwm)  # Allow negative values if needed

        return pwm_value



def main(args=None):
    rclpy.init(args=args)
    node = MecanumControl()

    rclpy.spin(node)

    # Cleanup on exit
    node.ser.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
