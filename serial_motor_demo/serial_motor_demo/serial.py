import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import String

class SerialReader(Node):
    def __init__(self):
        super().__init__('serial_reader')
        self.publisher_ = self.create_publisher(String, 'serial_data', 1000)
        self.serial_port = serial.Serial('/dev/ttyUSB0', 9600)
        self.timer = self.create_timer(0.0001, self.read_serial)

    def read_serial(self):
        if self.serial_port.in_waiting > 0:
            msg = String()
            msg.data = self.serial_port.readline().decode().strip()
            print(f"{{{msg.data}}}")
            self.publisher_.publish(msg)

def main():
    rclpy.init()
    node = SerialReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
