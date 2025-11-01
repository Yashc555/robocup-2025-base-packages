import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import json
import re

class MecanumRobotReceiver(Node):
    def __init__(self):
        super().__init__('mecanum_robot_receiver')

        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyACM1')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('bytesize', serial.EIGHTBITS)
        self.declare_parameter('parity', serial.PARITY_NONE)
        self.declare_parameter('stopbits', serial.STOPBITS_ONE)

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

        # Publisher for IMU data
        self.imu_data_pub = self.create_publisher(String, '/combined_data', 1000)
        # Timer for continuous receiving
        self.timer = self.create_timer(0.0001, self.receive_serial_data)

    def receive_serial_data(self):
        """
        Receives IMU data from STM and publishes it.
        """
        try:
            if self.serial_conn.in_waiting > 0:
                raw_data = self.serial_conn.readline().decode(errors='replace').strip()
                
                try:
                    parsed_data = json.loads(raw_data)
                    self.get_logger().info(f"Received: {parsed_data}")
                except json.JSONDecodeError:
                    self.get_logger().warn("Invalid JSON format received.")
                    return

                required_keys = {'yaw', 'pitch', 'roll', 'w', 'x', 'y', 'z', 'a', 'b', 'c', 'd'}
                if required_keys.issubset(parsed_data.keys()):
                    msg = String(data=json.dumps(parsed_data))
                    self.imu_data_pub.publish(msg)
                else:
                    self.get_logger().warn("IMU data does not contain all expected fields.")

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
