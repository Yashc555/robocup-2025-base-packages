import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String  # For publishing STM data as a string
import serial

class MecanumRobotSender(Node):
    def __init__(self):
        super().__init__('mecanum_robot_sender')

        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')  # Default serial port
        self.declare_parameter('baud_rate', 115200)  # Baud rate for communication
        self.declare_parameter('bytesize', serial.EIGHTBITS)  # Default bytesize
        self.declare_parameter('parity', serial.PARITY_NONE)  # Default parity
        self.declare_parameter('stopbits', serial.STOPBITS_ONE)  # Default stopbits

        # Get parameters
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.bytesize = self.get_parameter('bytesize').get_parameter_value().integer_value
        self.parity = self.get_parameter('parity').get_parameter_value().string_value
        self.stopbits = self.get_parameter('stopbits').get_parameter_value().integer_value

        # Serial connection
        try:
            self.serial_conn = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                bytesize=self.bytesize,
                parity=self.parity,
                stopbits=self.stopbits,
                timeout=1
            )
            self.get_logger().info(f"Connected to STM on {self.serial_port} at {self.baud_rate} baud.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to STM: {e}")
            rclpy.shutdown()

        # Initial PWM commands for each motor
        self.pwm_commands = [50, -50, 50, -50]

        # Publisher for STM data
        self.stm_data_pub = self.create_publisher(String, '/stm_data', 10)

        # Timer for continuous sending and receiving
        self.timer = self.create_timer(0.2, self.send_and_receive_serial)

    def parse_encoder_data(self, response_lines):
        """
        Parses the STM response for encoder values sent line by line.
        Ignores any invalid or unexpected lines.
        """
        encoder_data = {'A': 0, 'B': 0, 'C': 0, 'D': 0}

        for line in response_lines:
            try:
                if line.startswith('A'):
                    encoder_data['A'] = int(line[1:])  # Extract value after 'A'
                elif line.startswith('B'):
                    encoder_data['B'] = int(line[1:])  # Extract value after 'B'
                elif line.startswith('C'):
                    encoder_data['C'] = int(line[1:])  # Extract value after 'C'
                elif line.startswith('D'):
                    encoder_data['D'] = int(line[1:])  # Extract value after 'D'
            except ValueError:
                # Log and skip invalid lines
                self.get_logger().warn(f"Invalid line in response: {line}")

        return encoder_data


    def calculate_pwm(self, encoder_data):
        """
        Calculate the new PWM values based on the encoder data.
        """
        # Example: Adjust PWM based on encoder values
        pwm_threshold = 10  # Some threshold for adjustments
        for i, encoder in enumerate(['A', 'B', 'C', 'D']):
            if encoder_data[encoder] < pwm_threshold:
                self.pwm_commands[i] += 5  # Increase speed if encoder value is small
            elif encoder_data[encoder] > pwm_threshold:
                self.pwm_commands[i] -= 5  # Decrease speed if encoder value is large
            # Clamp PWM commands to the range [-100, 100]
            self.pwm_commands[i] = max(-100, min(100, self.pwm_commands[i]))

    def send_and_receive_serial(self):
        """
        Sends PWM commands to the STM and listens for responses.
        """
        try:
            # Send PWM commands
            command = ",".join([f"{pwm}" for pwm in self.pwm_commands])
            command += ',A'  # Example additional command
            self.serial_conn.write(command.encode())
            self.serial_conn.flush()
            self.get_logger().info(f"Sent Command to STM: {command.strip()}")

            # Check for incoming data
            if self.serial_conn.in_waiting > 0:
                raw_data = self.serial_conn.read(self.serial_conn.in_waiting).decode(errors='replace')
                response_lines = raw_data.strip().splitlines()  # Split response into lines
                self.get_logger().info(f"Received from STM:\n{raw_data}")

                # Parse encoder data from response lines
                encoder_data = self.parse_encoder_data(response_lines)

                # Calculate new PWM values based on encoder data
                self.calculate_pwm(encoder_data)

                # Publish the received data to the /stm_data topic
                msg = String()
                msg.data = raw_data  # Publish the raw STM response
                self.stm_data_pub.publish(msg)

        except serial.SerialException as e:
            self.get_logger().error(f"Failed to send/receive data to/from STM: {e}")
        except UnicodeDecodeError as e:
            self.get_logger().error(f"Failed to decode serial data: {e}")

def main(args=None):
    rclpy.init(args=args)
    robot = MecanumRobotSender()
    rclpy.spin(robot)
    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
