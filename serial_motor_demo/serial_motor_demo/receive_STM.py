import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # For publishing STM data as a string
import serial

class MecanumRobotReceiver(Node):
    def __init__(self):
        super().__init__('mecanum_robot_receiver')

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

        # Publisher for STM data
        self.stm_data_pub = self.create_publisher(String, '/stm_data', 10)

        # Timer for continuous receiving
        self.timer = self.create_timer(0.2, self.receive_serial_data)

    def receive_serial_data(self):
        """
        Receives encoder data from STM and publishes it.
        """
        try:
            # Check for incoming data
            if self.serial_conn.in_waiting > 0:
                raw_data = self.serial_conn.read(self.serial_conn.in_waiting).decode(errors='replace')
                response_lines = raw_data.strip().splitlines()  # Split response into lines
                self.get_logger().info(f"Received from STM:\n{raw_data}")

                # Publish the received data to the /stm_data topic
                msg = String()
                msg.data = raw_data  # Publish the raw STM response
                self.stm_data_pub.publish(msg)

        except serial.SerialException as e:
            self.get_logger().error(f"Failed to receive data from STM: {e}")
        except UnicodeDecodeError as e:
            self.get_logger().error(f"Failed to decode serial data: {e}")

def main(args=None):
    rclpy.init(args=args)
    robot = MecanumRobotReceiver()
    rclpy.spin(robot)
    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
