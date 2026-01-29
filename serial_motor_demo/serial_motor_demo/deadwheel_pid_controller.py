#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import json
import time

class CmdvelToMcu(Node):
    def __init__(self):
        super().__init__('cmdvel_to_mcu')

        # --- TUNING PARAMETERS ---
        self.declare_parameter('max_pwm', 35)
        self.declare_parameter('ramp_step', 4)       # Acceleration limit
        self.declare_parameter('kp_straight', 0.3)   # P-Gain for Heading
        self.declare_parameter('ki_straight', 0.1)   # I-Gain for Heading (Fixes persistent drift)
        self.declare_parameter('kp_speed', 0.5)      # P-Gain for Speed
        self.declare_parameter('ki_speed', 0.1)      # I-Gain for Speed
        self.declare_parameter('loop_rate', 20.0)    # Hz (Control Frequency)

        # Robot Geometry
        self.declare_parameter('wheel_L', 0.46)
        self.declare_parameter('wheel_W', 0.52)
        self.declare_parameter('scale_factor', 100.0)

        # Variables
        self.max_pwm = self.get_parameter('max_pwm').value
        self.ramp_step = self.get_parameter('ramp_step').value
        self.kp_straight = self.get_parameter('kp_straight').value
        self.ki_straight = self.get_parameter('ki_straight').value
        self.kp_speed = self.get_parameter('kp_speed').value
        self.ki_speed = self.get_parameter('ki_speed').value
        self.loop_dt = 1.0 / self.get_parameter('loop_rate').value
        
        self.L = self.get_parameter('wheel_L').value
        self.W = self.get_parameter('wheel_W').value
        self.scale_factor = self.get_parameter('scale_factor').value

        # Comms
        self.pub_mcu_out = self.create_publisher(String, 'mcu/out', 10)
        self.create_subscription(Twist, 'cmd_vel_out', self.cb_cmdvel, 10)
        self.create_subscription(Odometry, '/odom', self.cb_odom, 10)

        # --- STATE VARIABLES ---
        self.target_vel = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.actual_vel = {'x': 0.0, 'z': 0.0}
        self.current_pwms = [0, 0, 0, 0]
        self.last_cmd_time = 0.0
        
        # PID Error Accumulators (Integral)
        self.err_sum_x = 0.0
        self.err_sum_z = 0.0

        # Start the Control Loop Timer
        self.create_timer(self.loop_dt, self.control_loop)
        self.get_logger().info("Proper PI Control Loop Active")

    def cb_cmdvel(self, msg: Twist):
        # Just store the target. Do NOT calculate PWM here.
        self.target_vel['x'] = msg.linear.x
        self.target_vel['y'] = msg.linear.y
        self.target_vel['z'] = msg.angular.z
        self.last_cmd_time = time.time()

    def cb_odom(self, msg: Odometry):
        self.actual_vel['x'] = msg.twist.twist.linear.x
        self.actual_vel['z'] = msg.twist.twist.angular.z

    def control_loop(self):
        # 1. Safety Timeout (Idle Check)
        if (time.time() - self.last_cmd_time) > 0.2:
            self.target_vel = {'x': 0.0, 'y': 0.0, 'z': 0.0}
            self.err_sum_x = 0.0 # Reset integrals when stopped
            self.err_sum_z = 0.0

        tgt_x = self.target_vel['x']
        tgt_z = self.target_vel['z']

        # 2. PI CALCULATIONS
        # Forward Speed PI
        error_x = tgt_x - self.actual_vel['x']
        self.err_sum_x += error_x * self.loop_dt
        # Anti-windup: clamp integral
        self.err_sum_x = max(min(self.err_sum_x, 1.0), -1.0) 
        
        adj_vx = (tgt_x * 1.0) + (error_x * self.kp_speed) + (self.err_sum_x * self.ki_speed)

        # Heading PI (The Slant Fixer)
        error_z = tgt_z - self.actual_vel['z']
        self.err_sum_z += error_z * self.loop_dt
        self.err_sum_z = max(min(self.err_sum_z, 1.0), -1.0)

        adj_wz = (tgt_z * 1.0) + (error_z * self.kp_straight) + (self.err_sum_z * self.ki_straight)

        # 3. KINEMATICS (Convert corrected velocities to wheel speeds)
        geom = self.L + self.W
        vy = self.target_vel['y'] # Strafe (Open loop usually fine)

        raw_speeds = [
            adj_vx - vy - adj_wz * geom,  # FL
            adj_vx + vy + adj_wz * geom,  # FR
            adj_vx + vy - adj_wz * geom,  # RL
            adj_vx - vy + adj_wz * geom   # RR
        ]

        # 4. RAMPING & OUTPUT
        final_pwms = []
        for i in range(4):
            target_pwm = int(raw_speeds[i] * self.scale_factor)
            
            # Hard Clamp
            target_pwm = max(min(target_pwm, self.max_pwm), -self.max_pwm)
            
            # Ramp
            diff = target_pwm - self.current_pwms[i]
            if abs(diff) > self.ramp_step:
                step = self.ramp_step if diff > 0 else -self.ramp_step
                self.current_pwms[i] += step
            else:
                self.current_pwms[i] = target_pwm
            
            final_pwms.append(self.current_pwms[i])

        # Publish
        msg = {
            "pwm1": -final_pwms[1],
            "pwm2": final_pwms[2],
            "pwm3": final_pwms[0],
            "pwm4": -final_pwms[3]
        }
        out = String()
        out.data = json.dumps(msg)
        self.pub_mcu_out.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = CmdvelToMcu()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()