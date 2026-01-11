#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from threading import Thread, Lock
import serial
import json
import time

class SerialArbiter(Node):
    def __init__(self):
        super().__init__('serial_arbiter')
        self.get_logger().info("!!! VERSION 2.1 (With MCU/OUT) LOADED !!!")

        # -------- Parameters --------
        self.declare_parameter('imu_port', '/dev/stm_imu_usb')
        self.declare_parameter('enc_port', '/dev/stm_encoder_usb')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('out_topic', 'mcu/in')

        self.imu_port = self.get_parameter('imu_port').value
        self.enc_port = self.get_parameter('enc_port').value
        self.baud = self.get_parameter('baud').value
        self.out_topic = self.get_parameter('out_topic').value

        # -------- State --------
        self.imu_ser = None
        self.enc_ser = None

        self.latest_imu = None
        self.latest_enc = None

        self.lock = Lock()
        self.running = True

        # -------- ROS Publishers & Subscribers --------
        # Publishes merged sensor data
        self.pub = self.create_publisher(String, self.out_topic, 20)
        
        # NEW: Listens for commands (e.g., Motors) and sends to Encoder MCU
        self.sub_out = self.create_subscription(String, 'mcu/out', self.cb_out, 20)

        # -------- Threads --------
        Thread(target=self._imu_reader, daemon=True).start()
        Thread(target=self._enc_reader, daemon=True).start()

        self.get_logger().info("SerialArbiter started (2 STM mode + Motor Control)")

    # ================= SERIAL OPEN =================
    def _open_serial(self, port):
        try:
            ser = serial.Serial(port, self.baud, timeout=0.2)
            time.sleep(0.5)
            ser.reset_input_buffer()
            self.get_logger().info(f"Opened {port}")
            return ser
        except Exception as e:
            self.get_logger().error(f"Failed to open {port}: {e}")
            return None

    # ================= MCU OUT CALLBACK (NEW) =================
    def cb_out(self, msg: String):
        """
        Receives data from ROS 'mcu/out' and writes it to the Serial port.
        Assumes Motor Control is on the Encoder Port (enc_ser).
        """
        payload = msg.data
        if not payload.endswith("\n"):
            payload = payload + "\n"

        # We prioritize writing to the Encoder/Motor MCU
        # If you need to write to IMU instead, change this to self.imu_ser
        target_ser = self.enc_ser

        if target_ser and target_ser.is_open:
            try:
                target_ser.write(payload.encode())
                # specific flush isn't strictly necessary with modern pyserial 
                # but ensures data leaves buffer immediately
                target_ser.flush() 
            except Exception as e:
                self.get_logger().warn(f"Serial write failed: {e}")
        else:
            # We don't log this as error to avoid spamming if disconnected
            pass

    # ================= IMU THREAD =================
    def _imu_reader(self):
        self.imu_ser = self._open_serial(self.imu_port)
        if not self.imu_ser:
            return

        while self.running:
            try:
                raw = self.imu_ser.readline()
                if not raw:
                    continue   

                line = raw.decode(errors='ignore').strip()
                if not line:
                    continue   

                # self.get_logger().info(f"IMU Raw: {line}") # Uncomment for debug

                # --- JSON Parsing ---
                json_start = line.find("{")
                if json_start == -1:
                    continue  

                line = line[json_start:] 
                
                try:
                    data = json.loads(line)
                except json.JSONDecodeError:
                    self.get_logger().warn(f"Bad IMU JSON: {line}")
                    continue   

                with self.lock:
                    self.latest_imu = data

                self._try_publish()

            except Exception as e:
                self.get_logger().error(f"IMU read fatal: {e}")
                time.sleep(1.0)
                # Try to reconnect
                if not self.imu_ser or not self.imu_ser.is_open:
                     self.imu_ser = self._open_serial(self.imu_port)

    # ================= ENCODER THREAD =================
    def _enc_reader(self):
        self.enc_ser = self._open_serial(self.enc_port)
        if not self.enc_ser:
            return

        while self.running:
            try:
                raw = self.enc_ser.readline()
                if not raw:
                    continue

                line = raw.decode(errors='ignore').strip()
                if not line:
                    continue

                # self.get_logger().info(f"ENC Raw: {line}") # Uncomment for debug

                # --- JSON Parsing ---
                json_start = line.find("{")
                if json_start == -1:
                    continue 

                line = line[json_start:]
                
                try:
                    data = json.loads(line)
                except json.JSONDecodeError:
                    self.get_logger().warn(f"Bad ENC JSON: {line}")
                    continue

                with self.lock:
                    self.latest_enc = data

                self._try_publish()

            except Exception as e:
                self.get_logger().error(f"Encoder read fatal: {e}")
                time.sleep(1.0)
                # Try to reconnect
                if not self.enc_ser or not self.enc_ser.is_open:
                     self.enc_ser = self._open_serial(self.enc_port)

    # ================= MERGE & PUBLISH =================
    def _try_publish(self):
        with self.lock:
            # Wait for data from BOTH sensors before publishing
            if self.latest_imu is None or self.latest_enc is None:
                return

            # Combine dictionaries
            merged = {
                **self.latest_enc,
                **self.latest_imu,
                "timestamp": time.time()
            }

            # Clear buffer so we don't publish stale data repeatedly? 
            # (Optional: depends on if you want event-based or latching)
            # self.latest_imu = None 
            # self.latest_enc = None

        msg = String()
        msg.data = json.dumps(merged)
        self.pub.publish(msg)

    # ================= CLEANUP =================
    def destroy_node(self):
        self.running = False
        try:
            if self.imu_ser and self.imu_ser.is_open:
                self.imu_ser.close()
            if self.enc_ser and self.enc_ser.is_open:
                self.enc_ser.close()
        except Exception:
            pass

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
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()