import serial
import threading
from std_msgs.msg import String
from rclpy.node import Node
import rclpy
class MeccanumDriveNode(Node):
    def __init__(self):
        super().__init__('meccanum_drive_node')

        # Serial port configuration
        self.serial_port = '/dev/ttyACM0'
        self.baud_rate = 115200
        self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)

        # Start a separate thread to read from serial
        self.serial_thread = threading.Thread(target=self.read_serial_data)
        self.serial_thread.daemon = True
        self.serial_thread.start()

        # Publisher for control commands
        self.command_publisher = self.create_publisher(String, '/control_commands', 10)

        # Variables to store received data
        self.euler_angles = {'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0}
        self.quaternion = {'W': 0.0, 'X': 0.0, 'Y': 0.0, 'Z': 0.0}
        self.encoders = {'A': 0, 'B': 0, 'C': 0, 'D': 0}

    def read_serial_data(self):
        while True:
            try:
                if self.ser.in_waiting > 0:
                    # Read each line of data
                    yaw_pitch_roll_line = self.ser.readline().decode('utf-8').strip()
                    quaternion_line = self.ser.readline().decode('utf-8').strip()
                    encoder_a_line = self.ser.readline().decode('utf-8').strip()
                    encoder_b_line = self.ser.readline().decode('utf-8').strip()
                    encoder_c_line = self.ser.readline().decode('utf-8').strip()
                    encoder_d_line = self.ser.readline().decode('utf-8').strip()

                    # Parse the received data
                    self.parse_euler_angles(yaw_pitch_roll_line)
                    self.parse_quaternion(quaternion_line)
                    self.parse_encoder_data(encoder_a_line, encoder_b_line, encoder_c_line, encoder_d_line)
                    self.process_and_publish()

            except Exception as e:
                self.get_logger().error(f"Error reading serial data: {e}")

    def parse_euler_angles(self, line):
        try:
            # Parse Yaw, Pitch, Roll
            parts = line.split(" ")
            self.euler_angles['Yaw'] = float(parts[0].split(":")[1])
            self.euler_angles['Pitch'] = float(parts[1].split(":")[1])
            self.euler_angles['Roll'] = float(parts[2].split(":")[1])
        except (IndexError, ValueError):
            self.get_logger().error("Invalid Euler angle data received.")

    def parse_quaternion(self, line):
        try:
            # Parse W, X, Y, Z
            parts = line.split(" ")
            self.quaternion['W'] = float(parts[0].split(":")[1])
            self.quaternion['X'] = float(parts[1].split(":")[1])
            self.quaternion['Y'] = float(parts[2].split(":")[1])
            self.quaternion['Z'] = float(parts[3].split(":")[1])
        except (IndexError, ValueError):
            self.get_logger().error("Invalid Quaternion data received.")

    def parse_encoder_data(self, encoder_a, encoder_b, encoder_c, encoder_d):
        try:
            # Parse encoder values (A, B, C, D)
            self.encoders['A'] = int(encoder_a.split(":")[1])
            self.encoders['B'] = int(encoder_b.split(":")[1])
            self.encoders['C'] = int(encoder_c.split(":")[1])
            self.encoders['D'] = int(encoder_d.split(":")[1])
        except (IndexError, ValueError):
            self.get_logger().error("Invalid encoder data received.")

    def process_and_publish(self):
        # Process encoder values to determine direction and PWM
        pwm0, dir0 = self.calculate_pwm_direction(self.encoders['A'])
        pwm1, dir1 = self.calculate_pwm_direction(self.encoders['B'])
        pwm2, dir2 = self.calculate_pwm_direction(self.encoders['C'])
        pwm3, dir3 = self.calculate_pwm_direction(self.encoders['D'])

        # Generate the control command
        command = f"Dir{dir0},{pwm0},Dir{dir1},{pwm1},Dir{dir2},{pwm2},Dir{dir3},{pwm3}"

        # Publish the command to the STM
        self.send_serial_data(command)

        # Publish the command to the ROS topic for monitoring (optional)
        msg = String()
        msg.data = command
        self.command_publisher.publish(msg)
        self.get_logger().info(f"Published Command: {command}")

    def calculate_pwm_direction(self, encoder_value):
        """
        Calculates PWM and direction based on encoder value.
        Direction is 0 (forward) for positive values and 1 (reverse) for negative.
        PWM is the absolute value of the encoder reading, capped at 255.
        """
        max_pwm = 255
        if encoder_value < 0:
            direction = 1  # Reverse
            pwm = min(max(abs(encoder_value), 0), max_pwm)
        else:
            direction = 0  # Forward
            pwm = min(max(encoder_value, 0), max_pwm)

        return pwm, direction

    def send_serial_data(self, command):
        try:
            self.ser.write((command + '\n').encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f"Error sending data to STM: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MeccanumDriveNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
