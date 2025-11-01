#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from serial_motor_demo_msgs.msg import MotorCommand
from serial_motor_demo_msgs.msg import MotorVels
from serial_motor_demo_msgs.msg import EncoderVals
import time
import math
import serial
from threading import Lock



class MotorDriver(Node):

    def __init__(self):
        super().__init__('motor_driver')


        # Setup parameters

        self.declare_parameter('encoder_cpr', value=0)
        if (self.get_parameter('encoder_cpr').value == 0):
            print("WARNING! ENCODER CPR SET TO 0!!")


        self.declare_parameter('loop_rate', value=0)
        if (self.get_parameter('loop_rate').value == 0):
            print("WARNING! LOOP RATE SET TO 0!!")


        self.declare_parameter('serial_port', value="/dev/ttyACM0")
        self.serial_port = self.get_parameter('serial_port').value


        self.declare_parameter('baud_rate', value=9600)
        self.baud_rate = self.get_parameter('baud_rate').value


        self.declare_parameter('serial_debug', value=False)
        self.debug_serial_cmds = self.get_parameter('serial_debug').value
        if (self.debug_serial_cmds):
            print("Serial debug enabled")



        # Setup topics & services

        self.subscription = self.create_subscription(
            MotorCommand,
            'motor_command',
            self.motor_command_callback,
            10)

        self.speed_pub = self.create_publisher(MotorVels, 'motor_vels', 10)

        self.encoder_pub = self.create_publisher(EncoderVals, 'encoder_vals', 10)
        

        # Member Variables

        self.last_enc_read_time = time.time()
        self.last_m1_enc = 0
        self.last_m2_enc = 0
        self.m1_spd = 0.0
        self.m2_spd = 0.0

        self.mutex = Lock()


        # Open serial comms

        print(f"Connecting to port {self.serial_port} at {self.baud_rate}.")
        self.conn = serial.Serial(self.serial_port, self.baud_rate, timeout=1000.0)
        print(f"Connected to {self.conn}")
        

        


    # Raw serial commands
    
    def send_pwm_motor_command(self, mot_1_pwm, mot_2_pwm):
        self.send_command(f"o {int(mot_1_pwm)} {int(mot_2_pwm)}")

    def send_feedback_motor_command(self, mot_1_ct_per_loop, mot_2_ct_per_loop):
        self.send_command(f"m {int(mot_1_ct_per_loop)} {int(mot_2_ct_per_loop)}")

    def send_encoder_read_command(self):
        resp = self.send_command(f"e")  # Send encoder read command
        if resp:
            try:
                # Assuming the response contains distance data in meters
                # Convert the raw response to a list of floats representing the distances
                distances = [float(raw_enc) for raw_enc in resp.split()]
                return distances
            except ValueError as e:
                self.get_logger().error(f"Failed to parse encoder response: {resp}")
                return []  # Return empty list if parsing fails
        return []


    # More user-friendly functions

    def motor_command_callback(self, motor_command):
        if (motor_command.is_pwm):
            self.send_pwm_motor_command(motor_command.mot_1_req_rad_sec, motor_command.mot_2_req_rad_sec)
        else:
            # counts per loop = req rads/sec X revs/rad X counts/rev X secs/loop 
            scaler = (1 / (2*math.pi)) * self.get_parameter('encoder_cpr').value * (1 / self.get_parameter('loop_rate').value)
            mot1_ct_per_loop = motor_command.mot_1_req_rad_sec * scaler
            mot2_ct_per_loop = motor_command.mot_2_req_rad_sec * scaler
            self.send_feedback_motor_command(mot1_ct_per_loop, mot2_ct_per_loop)

    def check_encoders(self):
        resp = self.send_encoder_read_command()
        if resp:
            new_time = time.time()
            time_diff = new_time - self.last_enc_read_time
            self.last_enc_read_time = new_time

            # Calculate linear velocity (displacement / time) in meters per second
            m1_diff = resp[0] - self.last_m1_enc
            self.last_m1_enc = resp[0]
            m2_diff = resp[1] - self.last_m2_enc
            self.last_m2_enc = resp[1]

            # Calculate speed in meters per second
            if time_diff > 0:
                self.m1_spd = m1_diff / time_diff
                self.m2_spd = m2_diff / time_diff
            else:
                self.m1_spd = 0
                self.m2_spd = 0

            # Publish the linear speed (in meters per second)
            spd_msg = MotorVels()
            spd_msg.mot_1_rad_sec = self.m1_spd  # For linear speed, use mot_1_spd directly
            spd_msg.mot_2_rad_sec = self.m2_spd  # For linear speed, use mot_2_spd directly
            self.speed_pub.publish(spd_msg)

            # Publish encoder values (displacement)
            enc_msg = EncoderVals()
            enc_msg.mot_1_enc_val = self.last_m1_enc
            enc_msg.mot_2_enc_val = self.last_m2_enc
            self.encoder_pub.publish(enc_msg)




    # Utility functions

    def send_command(self, cmd_string):
        self.mutex.acquire()
        try:
            cmd_string += "\r"
            self.conn.write(cmd_string.encode("utf-8"))
            if self.debug_serial_cmds:
                print("Sent:", cmd_string)

            c = ''
            value = ''
            timeout_counter = 0
            while c != '\r' and timeout_counter < 5:
                c = self.conn.read(1).decode("utf-8")
                if c == '':
                    timeout_counter += 1
                    print("Error: Serial timeout, retrying command:", cmd_string)
                else:
                    value += c
            
            if timeout_counter >= 5:
                print("Error: Serial timeout after multiple attempts.")
                return ''
            
            value = value.strip('\r')
            if self.debug_serial_cmds:
                print("Received:", value)
            return value
        finally:
            self.mutex.release()


    def close_conn(self):
        self.conn.close()



def main(args=None):
    
    rclpy.init(args=args)

    motor_driver = MotorDriver()

    rate = motor_driver.create_rate(2)
    while rclpy.ok():
        rclpy.spin_once(motor_driver)
        motor_driver.check_encoders()


    motor_driver.close_conn()
    motor_driver.destroy_node()
    rclpy.shutdown()


