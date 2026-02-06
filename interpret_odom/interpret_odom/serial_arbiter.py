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
       
        # -------- Parameters --------
        self.declare_parameter('imu_port', '/dev/stm_imu_usb')
        self.declare_parameter('enc_port', '/dev/stm_encoder_usb')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('out_topic', 'mcu/in')
        self.declare_parameter('dual_mcu', False)

        self.imu_port = self.get_parameter('imu_port').value
        self.enc_port = self.get_parameter('enc_port').value
        self.baud = self.get_parameter('baud').value
        self.out_topic = self.get_parameter('out_topic').value
        self.dual_mcu = self.get_parameter('dual_mcu').value

        self.get_logger().info(f"!!! ARBITER LOADED (Dual Mode: {self.dual_mcu}) !!!")

        # -------- State --------
        self.imu_ser = None
        self.enc_ser = None
        self.latest_imu = None
        self.latest_enc = None
        self.lock = Lock()
        self.running = True

        # -------- ROS Publishers & Subscribers --------
        self.pub = self.create_publisher(String, self.out_topic, 20)
        self.sub_out = self.create_subscription(String, 'mcu/out', self.cb_out, 20)

        # -------- Threads --------
        if self.dual_mcu:
            Thread(target=self._imu_reader, daemon=True).start()
       
        Thread(target=self._enc_reader, daemon=True).start()

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

    def _send_stop(self, ser_obj):
        """Helper to send 0 PWM command."""
        if ser_obj and ser_obj.is_open:
            try:
                stop_msg = '{"pwm1": 0, "pwm2": 0, "pwm3": 0, "pwm4": 0}\n'
                ser_obj.write(stop_msg.encode())
                ser_obj.flush()
                time.sleep(0.1)
                self.get_logger().info(">> ZERO PWM command sent.")
            except Exception as e:
                self.get_logger().warn(f"Failed to send zero PWM: {e}")

    def cb_out(self, msg: String):
        payload = msg.data if msg.data.endswith("\n") else msg.data + "\n"
        target_ser = self.enc_ser
        if target_ser and target_ser.is_open:
            try:
                target_ser.write(payload.encode())
                target_ser.flush()
            except Exception as e:
                self.get_logger().warn(f"Serial write failed: {e}")

    def _imu_reader(self):
        self.imu_ser = self._open_serial(self.imu_port)
        while self.running and self.imu_ser:
            try:
                line = self.imu_ser.readline().decode(errors='ignore').strip()
                if "{" not in line: continue
                data = json.loads(line[line.find("{"):])
                with self.lock:
                    self.latest_imu = data
                self._try_publish()
            except Exception:
                time.sleep(1.0)
                if not self.imu_ser or not self.imu_ser.is_open:
                     self.imu_ser = self._open_serial(self.imu_port)

    def _enc_reader(self):
        self.enc_ser = self._open_serial(self.enc_port)
        
        # --- SEND 0 PWM ON INIT ---
        self._send_stop(self.enc_ser)
        # --------------------------

        while self.running and self.enc_ser:
            try:
                line = self.enc_ser.readline().decode(errors='ignore').strip()
                if "{" not in line: continue
                data = json.loads(line[line.find("{"):])
                with self.lock:
                    self.latest_enc = data
                self._try_publish()
            except Exception:
                time.sleep(1.0)
                if not self.enc_ser or not self.enc_ser.is_open:
                     self.enc_ser = self._open_serial(self.enc_port)
                     # Optional: Send stop on reconnect as well
                     self._send_stop(self.enc_ser)

    def _try_publish(self):
        with self.lock:
            if self.latest_enc is None:
                return
            if self.dual_mcu and self.latest_imu is None:
                return

            merged = {**self.latest_enc}
            if self.dual_mcu and self.latest_imu:
                merged.update(self.latest_imu)
           
            merged["timestamp"] = time.time()

        msg = String()
        msg.data = json.dumps(merged)
        self.pub.publish(msg)

    def destroy_node(self):
        self.get_logger().info("Shutting down... sending EMERGENCY STOP to MCU.")
        self.running = False
       
        # --- SAFETY SHUTDOWN LOGIC ---
        self._send_stop(self.enc_ser)
        # -----------------------------

        for s in [self.imu_ser, self.enc_ser]:
            if s and s.is_open: s.close()
       
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