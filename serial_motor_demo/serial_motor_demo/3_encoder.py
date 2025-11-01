import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
import serial


class MecanumRobot(Node):
    def __init__(self):
        super().__init__('mecanum_robot')

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscriptions
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )

        # Parameters
        self.declare_parameter('wheel_radius', 0.05)  # meters
        self.declare_parameter('wheel_base', 0.3)    # meters
        self.declare_parameter('track_width', 0.3)  # meters
        self.declare_parameter('serial_port', '/dev/ttyACM0')  # Default serial port
        self.declare_parameter('baud_rate', 115200)  # Baud rate for communication

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # Motor commands and encoder feedback
        self.wheel_pwm = [0, 0, 0, 0]  # PWM for each wheel
        self.encoder_ticks = [0, 0, 0, 0]  # Encoder feedback

        # Missing encoder index
        self.missing_encoder_index = 0  # Assuming Motor A is missing an encoder

        # Serial connection
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value

        try:
            self.serial_conn = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f"Connected to STM on {self.serial_port} at {self.baud_rate} baud.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to STM: {e}")
            rclpy.shutdown()

        # Timer
        self.create_timer(0.05, self.update_odometry)

    def cmd_vel_callback(self, msg):
        """
        Converts a Twist message to PWM signals for the mecanum wheels.
        """
        vx = msg.linear.x
        vy = msg.linear.y
        omega = msg.angular.z

        r = self.get_parameter('wheel_radius').get_parameter_value().double_value
        L = self.get_parameter('wheel_base').get_parameter_value().double_value
        W = self.get_parameter('track_width').get_parameter_value().double_value

        # Compute wheel velocities
        wheel_velocities = [
            (vx - vy - (L + W) * omega) / r,
            (vx + vy + (L + W) * omega) / r,
            (vx + vy - (L + W) * omega) / r,
            (vx - vy + (L + W) * omega) / r,
        ]

        # Convert to PWM
        self.wheel_pwm = [self.velocity_to_pwm(v) for v in wheel_velocities]
        self.send_pwm_to_stm()

    def velocity_to_pwm(self, velocity):
        """
        Converts a wheel velocity to a PWM signal.
        """
        max_velocity = 1.0  # m/s (example maximum)
        max_pwm = 255  # Example maximum PWM value
        pwm = int(max(-max_pwm, min(max_pwm, velocity / max_velocity * max_pwm)))
        return pwm

    def send_pwm_to_stm(self):
        """
        Sends the PWM signals and directions for all four motors to the STM via serial.
        Format: Dir{dirA}{pwmA}, Dir{dirB}{pwmB}, Dir{dirC}{pwmC}, Dir{dirD}{pwmD}
        """
        try:
            for i in range(4):
                dir_value = 1 if self.wheel_pwm[i] >= 0 else 0
                pwm_value = abs(self.wheel_pwm[i])
                command = f"Dir{dir_value}{pwm_value}\n"
                self.serial_conn.write(command.encode())
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to send PWM data to STM: {e}")

    def read_encoder_from_stm(self):
        """
        Reads encoder values from the STM via serial.
        """
        try:
            # Expecting 4 signed 16-bit integers for encoder values
            data = self.serial_conn.readline().decode().strip()
            lines = data.split('\n')
            self.encoder_ticks = [0,0,0,0]
            for line in lines:
                if line and line[0] in ['A','B','C','D']:
                    label, value = line.split(':')
                    if label.strip() in ['A','B','C','D']:
                        idx = ['A','B','C','D'].index(label)
                        try:
                            self.encoder_ticks[idx] = int(value.strip())
                        except ValueError:
                            self.encoder_ticks[idx] = 0
                            
            if self.encoder_ticks[self.missing_encoder_index] == 0:
                    self.encoder_ticks[self.missing_encoder_index] = 0  # Missing encoder
                    
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to read encoder data from STM: {e}")

    def update_odometry(self):
        """
        Updates and publishes odometry based on encoder feedback.
        """
        self.read_encoder_from_stm()

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Convert encoder ticks to distances
        wheel_distances = self.encoder_ticks_to_distances()

        # Handle missing encoder by estimating its value
        if self.encoder_ticks[self.missing_encoder_index] == 0:
            wheel_distances[self.missing_encoder_index] = sum(wheel_distances[1:]) / 3

        # Compute linear and angular velocities
        vx, vy, omega = self.compute_robot_velocity(wheel_distances, dt)

        # Integrate to find position
        self.x += (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
        self.y += (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt
        self.theta += omega * dt

        # Publish odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2)

        # Velocity
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.angular.z = omega

        self.odom_pub.publish(odom_msg)

        # Broadcast transform
        transform = TransformStamped()
        transform.header.stamp = current_time.to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation.z = odom_msg.pose.pose.orientation.z
        transform.transform.rotation.w = odom_msg.pose.pose.orientation.w
        self.tf_broadcaster.sendTransform(transform)

    def encoder_ticks_to_distances(self):
        """
        Converts encoder ticks to distances traveled by the wheels.
        """
        ticks_per_rev = 4096  # Example value, update as needed
        wheel_circumference = 2 * math.pi * self.get_parameter('wheel_radius').get_parameter_value().double_value
        return [
            (ticks / ticks_per_rev) * wheel_circumference
            for ticks in self.encoder_ticks
        ]

    def compute_robot_velocity(self, wheel_distances, dt):
        """
        Computes the robot's linear and angular velocities from wheel distances.
        """
        r = self.get_parameter('wheel_radius').get_parameter_value().double_value
        L = self.get_parameter('wheel_base').get_parameter_value().double_value
        W = self.get_parameter('track_width').get_parameter_value().double_value

        # Inverse kinematics for mecanum wheels
        vx = r / 4 * (wheel_distances[0] + wheel_distances[1] +
                      wheel_distances[2] + wheel_distances[3]) / dt
        vy = r / 4 * (-wheel_distances[0] + wheel_distances[1] +
                      wheel_distances[2] - wheel_distances[3]) / dt
        omega = r / (4 * (L + W)) * (-wheel_distances[0] +
                                     wheel_distances[1] - wheel_distances[2] + wheel_distances[3]) / dt
        return vx, vy, omega


def main(args=None):
    rclpy.init(args=args)
    robot = MecanumRobot()
    rclpy.spin(robot)
    robot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
