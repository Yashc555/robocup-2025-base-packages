import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import Joy
import json
import time
import serial
from serial.tools import list_ports

class MecanumOpenLoopJoystickControl(Node):
    def __init__(self):
        super().__init__('mecanum_openloop_joystick_control')

        # Robot parameters
        self.L, self.W, self.R = 0.305, 0.2175, 0.075  # meters
        self.max_pwm = 35

        self.serial_port = None
        selected_port = self._auto_select_serial_port()
        if selected_port:
            try:
                self.serial_port = serial.Serial(selected_port, 115200, timeout=1)
                self.get_logger().info(f"Serial connection established on {selected_port}")
            except serial.SerialException:
                self.get_logger().error(f"Failed to open serial port {selected_port}")
                self.serial_port = None
        else:
            self.get_logger().error("No serial port found with Name == STM_ENCODER.PWM")
            self.serial_port = None

        self.create_subscription(Joy, 'joy', self.joy_callback, 10)

    def _auto_select_serial_port(self, scan_baud=115200, read_timeout=0.5, max_lines=20):
        """
        Scan available serial ports, open each briefly, read up to max_lines JSON lines,
        and select the port that publishes a JSON object with "Name" == "STM_ENCODER.PWM".
        Returns the device string (e.g. '/dev/ttyACM0') or None if none matched.
        """
        try:
            ports = list_ports.comports()
        except Exception as e:
            self.get_logger().warn(f"list_ports.comports() failed: {e}")
            return None

        for p in ports:
            port_name = p.device
            try:
                ser = serial.Serial(port_name, scan_baud, timeout=read_timeout)
                time.sleep(0.02)
            except Exception as e:
                self.get_logger().debug(f"Cannot open {port_name}: {e}")
                continue

            found = False
            try:
                lines_read = 0
                while lines_read < max_lines:
                    raw = ser.readline()
                    if not raw:
                        break
                    lines_read += 1
                    try:
                        s = raw.decode('utf-8', errors='ignore').strip()
                    except Exception:
                        continue
                    if not s:
                        continue
                    try:
                        data = json.loads(s)
                    except Exception:
                        continue
                    name = data.get('Name')
                    if isinstance(name, str) and name.strip() == 'STM_ENCODER.PWM':
                        found = True
                        break
            finally:
                try:
                    ser.close()
                except Exception:
                    pass

            if found:
                return port_name

        return None

    def joy_callback(self, msg: Joy):
        vx = msg.axes[1]  # Forward/Backward
        vy = msg.axes[0]  # Left/Right
        wz = msg.axes[3]  # Rotation

        # Scale joystick input to linear/angular velocities
        vx *= 1.0  # Adjust scaling factor as needed
        vy *= 1.0
        wz *= 1.0

        # Calculate desired wheel speeds (not using radius here because we directly convert to PWM)
        wheel_speeds = [
            vx - vy - wz * (self.L + self.W),  # Front Left
            vx + vy + wz * (self.L + self.W),  # Front Right
            vx + vy - wz * (self.L + self.W),  # Rear Left
            vx - vy + wz * (self.L + self.W)   # Rear Right
        ]
        

        # Convert speeds to PWM in range -255 to 255
        pwm_values = [int(self.clamp(speed * 100, -self.max_pwm, self.max_pwm)) for speed in wheel_speeds]
        # Map PWM to motor order expected by microcontroller
        pwm_message = json.dumps({"pwm1": -pwm_values[0], "pwm2": pwm_values[1], "pwm3": pwm_values[2], "pwm4": pwm_values[3]})
        self.get_logger().info(f"Sent PWM (Open-loop): {pwm_message}")

        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(pwm_message.encode())
            except serial.SerialException:
                self.get_logger().error("Failed to send data over serial port")

    def clamp(self, value, min_value, max_value):
        return max(min(value, max_value), min_value)

    def destroy_node(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MecanumOpenLoopJoystickControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()