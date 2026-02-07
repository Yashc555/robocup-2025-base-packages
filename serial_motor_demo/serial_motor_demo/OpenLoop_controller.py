#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import json
import time

class CmdvelToMcu(Node):
    def __init__(self):
        super().__init__('cmdvel_to_mcu')

        # --- Parameters ---
        self.declare_parameter('wheel_L', 0.305)
        self.declare_parameter('wheel_W', 0.2175)
        self.declare_parameter('max_pwm', 19)
        self.declare_parameter('scale_factor', 100.0)
        self.declare_parameter('min_pwm_threshold_normal', 16)
        self.declare_parameter('min_pwm_threshold_strafe', 28)
        self.declare_parameter('ramp_step', 12) 
        self.declare_parameter('strafe_gain', 1.8) 
        self.declare_parameter('idle_timeout', 0.05)
        self.declare_parameter('cmd_vel_in_topic', 'cmd_vel_out')
        self.declare_parameter('mcu_out_topic', 'mcu/out')
        
        self.declare_parameter('brake_duration', 0.30)     #in seconds
        self.declare_parameter('brake_pwm', 20)           # Standard brake PWM
        self.declare_parameter('brake_pwm_rotation', 13)  # Smaller brake PWM for point turns

        # Load Params
        self.L = float(self.get_parameter('wheel_L').get_parameter_value().double_value)
        self.W = float(self.get_parameter('wheel_W').get_parameter_value().double_value)
        self.max_pwm = int(self.get_parameter('max_pwm').get_parameter_value().integer_value)
        self.strafe_gain = float(self.get_parameter('strafe_gain').get_parameter_value().double_value)
        self.idle_timeout = float(self.get_parameter('idle_timeout').get_parameter_value().double_value)
        
        self.min_normal = int(self.get_parameter('min_pwm_threshold_normal').get_parameter_value().integer_value)
        self.min_strafe = int(self.get_parameter('min_pwm_threshold_strafe').get_parameter_value().integer_value)
        
        self.brake_duration = float(self.get_parameter('brake_duration').get_parameter_value().double_value)
        self.brake_pwm = int(self.get_parameter('brake_pwm').get_parameter_value().integer_value)
        self.brake_pwm_rotation = int(self.get_parameter('brake_pwm_rotation').get_parameter_value().integer_value)

        cmd_topic = self.get_parameter('cmd_vel_in_topic').get_parameter_value().string_value
        mcu_out_topic = self.get_parameter('mcu_out_topic').get_parameter_value().string_value

        self.pub_mcu_out = self.create_publisher(String, mcu_out_topic, 10)
        self.sub_cmd = self.create_subscription(Twist, cmd_topic, self.cb_cmdvel, 20)

        self.last_cmd_time = None
        self.current_pwms = [0, 0, 0, 0] 
        self.last_moving_pwms = [0, 0, 0, 0] 

        # Braking State Variables
        self.is_braking = False
        self.brake_start_time = 0.0
        self.active_brake_pwm = 0            # NEW: Stores the PWM to be used for the current braking session
        self.last_motion_was_rotation = False # NEW: Tracks if the previous move was a point turn

        # High frequency timer for braking and idle logic
        self.create_timer(0.05, self._control_loop)
        
        self.get_logger().info(f"Active. Rotation-Aware Braking Enabled (Std: {self.brake_pwm}, Rot: {self.brake_pwm_rotation})")
        self.send_pwm([0,0,0,0])

    def send_pwm(self, pwms):
        msg = {
            "pwm1": int(pwms[0]),
            "pwm2": int(pwms[1]),
            "pwm3": int(pwms[2]),
            "pwm4": int(pwms[3])
        }
        out = String()
        out.data = json.dumps(msg)
        self.pub_mcu_out.publish(out)

    def cb_cmdvel(self, msg: Twist):
        self.last_cmd_time = time.time()
        
        raw_vx = float(msg.linear.x)
        raw_vy = float(msg.linear.y)
        wz = -float(msg.angular.z)

        total_linear_mag = abs(raw_vx) + abs(raw_vy)
        strafe_ratio = abs(raw_vy) / total_linear_mag if total_linear_mag >= 0.05 else 0.0
        current_vy_gain = 1.0 + (strafe_ratio * (self.strafe_gain - 1.0))
        
        vx = raw_vx
        vy = raw_vy * current_vy_gain
        geom = self.L + self.W
        
        target_speeds_ms = [
            vx - vy - wz * geom,
            vx + vy + wz * geom,
            vx + vy - wz * geom,
            vx - vy + wz * geom
        ]

        active_min_pwm = self.min_normal + (strafe_ratio * (self.min_strafe - self.min_normal))
        normal_max = self.max_pwm
        strafe_max = self.max_pwm * self.strafe_gain
        active_max_pwm = normal_max + (strafe_ratio * (strafe_max - normal_max))
        NAV2_MAX_SPEED_MS = 0.09 

        target_pwms = []
        is_moving_command = False

        for speed_ms in target_speeds_ms:
            abs_speed = abs(speed_ms)
            if abs_speed < 0.01: 
                target_pwms.append(0)
                continue
            
            is_moving_command = True
            if abs_speed < 0.02: abs_speed = 0.02

            speed_ratio = (abs_speed - 0.02) / (NAV2_MAX_SPEED_MS - 0.02)
            speed_ratio = max(0.0, min(1.0, speed_ratio))
            
            pwm_range = active_max_pwm - active_min_pwm
            pwm_mag = active_min_pwm + (speed_ratio * pwm_range)
            
            final_pwm = int(pwm_mag) if speed_ms > 0 else -int(pwm_mag)
            target_pwms.append(final_pwm)

        while len(target_pwms) < 4: target_pwms.append(0)

        # --- MOTION TYPE TRACKING ---
        if is_moving_command:
            # Detect if this is a point turn (no linear, yes angular)
            if abs(raw_vx) < 0.02 and abs(raw_vy) < 0.02 and abs(wz) > 0.01:
                self.last_motion_was_rotation = True
            else:
                self.last_motion_was_rotation = False

            # Cancel braking if we get a new move command
            if self.is_braking:
                self.is_braking = False

            self.current_pwms = [
                -target_pwms[1], 
                target_pwms[2],
                target_pwms[0],
                -target_pwms[3]
            ]
            self.send_pwm(self.current_pwms)

        # --- BRAKING LOGIC ---
        else:
            was_moving = any(abs(p) > 0 for p in self.current_pwms)
            
            if was_moving and not self.is_braking:
                self.is_braking = True
                self.brake_start_time = time.time()
                self.last_moving_pwms = self.current_pwms
                
                # Assign brake strength based on last motion
                if self.last_motion_was_rotation:
                    self.active_brake_pwm = self.brake_pwm_rotation
                    self.get_logger().info(f"Braking: Rotation Mode (PWM {self.active_brake_pwm})")
                else:
                    self.active_brake_pwm = self.brake_pwm
                    self.get_logger().info(f"Braking: Standard Mode (PWM {self.active_brake_pwm})")

    def _control_loop(self):
        # 1. Handle Active Braking
        if self.is_braking:
            elapsed = time.time() - self.brake_start_time
            if elapsed < self.brake_duration:
                brake_pwms = []
                for p in self.last_moving_pwms:
                    if p > 0: brake_pwms.append(-self.active_brake_pwm)
                    elif p < 0: brake_pwms.append(self.active_brake_pwm)
                    else: brake_pwms.append(0)
                self.send_pwm(brake_pwms)
            else:
                self.is_braking = False
                self.current_pwms = [0,0,0,0]
                self.send_pwm([0,0,0,0])
                self.get_logger().info("Braking Complete. Stopped.")

        # 2. Handle Idle/Timeout (Joystick release)
        elif self.last_cmd_time is not None:
             if (time.time() - self.last_cmd_time) > self.idle_timeout:
                was_moving = any(abs(p) > 0 for p in self.current_pwms)
                
                if was_moving:
                    self.is_braking = True
                    self.brake_start_time = time.time()
                    self.last_moving_pwms = self.current_pwms
                    # For safety, assume standard brake on timeout unless already flagged
                    self.active_brake_pwm = self.brake_pwm_rotation if self.last_motion_was_rotation else self.brake_pwm
                    self.get_logger().warn("Timeout: Applying Emergency Brake")
                else:
                    self.send_pwm([0,0,0,0])
                
                self.last_cmd_time = None

def main(args=None):
    rclpy.init(args=args)
    node = CmdvelToMcu()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send_pwm([0,0,0,0])
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()