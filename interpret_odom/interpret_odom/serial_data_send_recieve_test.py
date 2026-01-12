import rclpy
from rclpy.node import Node
import serial
import json

class SerialJsonNode(Node):
    def __init__(self):
        super().__init__('serial_json_node')
        self.serial_port = '/dev/ttyACM0'
        self.baudrate = 115200
        try:
            self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=1)
            self.get_logger().info(f"Opened serial port {self.serial_port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            raise e
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        send_data = {'a': 1, 'b': 2, 'c': 420, 'd': 100}
        try:
            self.ser.write((json.dumps(send_data) + '\n').encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f"Failed to send data: {e}")
        try:
            if self.ser.in_waiting:
                line = self.ser.readline().decode('utf-8').strip()
                if line:
                    # recv_data = json.loads(line)
                    self.get_logger().info(f"Received: {line}")
        except json.JSONDecodeError:
            self.get_logger().warn("Received invalid JSON.")
        except Exception as e:
            self.get_logger().error(f"Error reading from serial: {e}")

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SerialJsonNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()