#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from threading import Thread, Lock
import serial, time
from serial.tools import list_ports

class SerialArbiter(Node):
    def __init__(self):
        super().__init__('serial_arbiter')
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('idle_close', False)
        self.declare_parameter('idle_timeout', 10.0)

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.idle_close = self.get_parameter('idle_close').get_parameter_value().bool_value
        self.idle_timeout = float(self.get_parameter('idle_timeout').get_parameter_value().double_value)

        self.ser = None
        self.ser_lock = Lock()
        self.last_rx_time = None
        self.running = True

        # ROS API
        self.pub_in = self.create_publisher(String, 'mcu/in', 20)
        self.sub_out = self.create_subscription(String, 'mcu/out', self.cb_out, 20)

        # thread to read serial
        self.thread = Thread(target=self._run, daemon=True)
        self.thread.start()
        self.create_timer(1.0, self._idle_check)

        self.get_logger().info(f"SerialArbiter starting for {self.port} @ {self.baud}")

    def _open_serial(self):
        try:
            ser = serial.Serial(self.port, self.baud, timeout=0.2)
        except Exception as e:
            self.get_logger().error(f"open {self.port} failed: {e}")
            return False
        # assert terminal lines
        try:
            ser.dtr = True
            ser.rts = False
        except Exception:
            pass
        time.sleep(0.5)
        try:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
        except Exception:
            pass
        # poke newline
        try:
            ser.write(b"\n")
            ser.flush()
        except Exception:
            pass
        with self.ser_lock:
            self.ser = ser
        self.get_logger().info(f"Opened {self.port}")
        return True

    def _close_serial(self):
        with self.ser_lock:
            if self.ser:
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
                self.get_logger().info("Closed serial")

    def _run(self):
        while self.running:
            if self.ser is None:
                ok = self._open_serial()
                if not ok:
                    time.sleep(1.0)
                    continue
            try:
                with self.ser_lock:
                    s = self.ser
                if s is None:
                    continue
                raw = s.readline()
                if not raw:
                    continue
                line = raw.decode('utf-8', errors='ignore').strip()
                if not line:
                    continue
                self.last_rx_time = time.time()
                msg = String(); msg.data = line
                self.pub_in.publish(msg)
            except Exception as e:
                self.get_logger().warn(f"Serial read error: {e}")
                self._close_serial()
                time.sleep(0.5)

    def cb_out(self, msg: String):
        payload = msg.data
        if not payload.endswith("\n"):
            payload = payload + "\n"
        with self.ser_lock:
            if self.ser and self.ser.is_open:
                try:
                    self.ser.write(payload.encode())
                    self.ser.flush()
                except Exception as e:
                    self.get_logger().warn(f"Serial write failed: {e}")
            else:
                self.get_logger().warn("Serial not open; dropping outgoing message")

    def _idle_check(self):
        if not self.idle_close:
            return
        if self.last_rx_time is None:
            return
        if time.time() - self.last_rx_time > self.idle_timeout:
            self._close_serial()
            self.last_rx_time = None

    def destroy_node(self):
        self.running = False
        self._close_serial()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SerialArbiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
