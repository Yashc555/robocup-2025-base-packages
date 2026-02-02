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

        # robot geometry params
        self.declare_parameter('wheel_L', 0.305)
        self.declare_parameter('wheel_W', 0.2175)
        
        # pwm limits and scaling
        self.declare_parameter('max_pwm', 18)
        self.declare_parameter('scale_factor', 100.0)
        
        # --- NEW: Dual Thresholds ---
        # Threshold for standard movement (forward/turn)
        self.declare_parameter('min_pwm_threshold_normal', 15)
        # Threshold specifically for strafing (sideways)
        self.declare_parameter('min_pwm_threshold_strafe', 28)
        
        # RAMPING PARAMETER
        self.declare_parameter('ramp_step', 4) 

        # STRAFING GAIN
        self.declare_parameter('strafe_gain',1.9) 

        self.declare_parameter('idle_timeout', 0.05)
        self.declare_parameter('cmd_vel_in_topic', 'cmd_vel_out')
        self.declare_parameter('mcu_out_topic', 'mcu/out')

        self.L = float(self.get_parameter('wheel_L').get_parameter_value().double_value)
        self.W = float(self.get_parameter('wheel_W').get_parameter_value().double_value)
        self.max_pwm = int(self.get_parameter('max_pwm').get_parameter_value().integer_value)
        self.scale_factor = float(self.get_parameter('scale_factor').get_parameter_value().double_value)
        self.strafe_gain = float(self.get_parameter('strafe_gain').get_parameter_value().double_value)
        self.idle_timeout = float(self.get_parameter('idle_timeout').get_parameter_value().double_value)
        self.ramp_step = int(self.get_parameter('ramp_step').get_parameter_value().integer_value)
        
        # Retrieve separate thresholds
        self.min_normal = int(self.get_parameter('min_pwm_threshold_normal').get_parameter_value().integer_value)
        self.min_strafe = int(self.get_parameter('min_pwm_threshold_strafe').get_parameter_value().integer_value)
        
        cmd_topic = self.get_parameter('cmd_vel_in_topic').get_parameter_value().string_value
        mcu_out_topic = self.get_parameter('mcu_out_topic').get_parameter_value().string_value

        self.pub_mcu_out = self.create_publisher(String, mcu_out_topic, 10)
        self.sub_cmd = self.create_subscription(Twist, cmd_topic, self.cb_cmdvel, 20)

        self.last_cmd_time = None
        self.current_pwms = [0, 0, 0, 0] 

        self.create_timer(0.1, self._idle_check)
        self.get_logger().info(f"Active. MAX_PWM : {self.max_pwm} STrafe Gain : {self.strafe_gain} Normal Min: {self.min_normal}, Strafe Min: {self.min_strafe}")

    def cb_cmdvel(self, msg: Twist):
        vx = float(msg.linear.x)
        vy = float(msg.linear.y) * self.strafe_gain
        wz = -float(msg.angular.z)

        # 1. Detect Strafing to pick the correct hardware floor
        is_strafing = abs(vy) > 0.01
        
        # ACTIVE MIN: The minimum PWM required to overcome friction
        active_min_pwm = self.min_strafe if is_strafing else self.min_normal
        # ACTIVE MAX: The highest allowed PWM
        active_max_pwm = int(self.max_pwm * self.strafe_gain) if is_strafing else self.max_pwm

        geom = self.L + self.W
        
        # Calculate raw wheel speeds (m/s)
        # Note: These values will be small, e.g., 0.02, 0.05, 0.1
        target_speeds_ms = [
            vx - vy - wz * geom,  # FL
            vx + vy + wz * geom,  # FR
            vx + vy - wz * geom,  # RL
            vx - vy + wz * geom   # RR
        ]

        # THE FIX: Map m/s directly to the PWM range [min, max]
        # We assume 0.1 m/s is your "Max Speed" based on your Nav2 config.
        NAV2_MAX_SPEED_MS = 0.1  
        
        target_pwms = []
        for speed_ms in target_speeds_ms:
            abs_speed = abs(speed_ms)
            
            # Noise filter: If speed is practically zero, send 0 PWM
            if abs_speed < 0.005: 
                target_pwms.append(0)
                continue

            # LINEAR MAPPING FORMULA:
            # PWM = Min_Floor + (Speed / Max_Speed) * (Max_Ceiling - Min_Floor)
            
            # Ratio: How fast are we going relative to max? (e.g. 0.5 = half speed)
            ratio = abs_speed / NAV2_MAX_SPEED_MS
            if ratio > 1.0: ratio = 1.0  # Cap at max
            
            # Calculate PWM starting from the floor (20) up to ceiling (30)
            pwm_range = active_max_pwm - active_min_pwm
            pwm_mag = active_min_pwm + (ratio * pwm_range)
            
            # Restore sign
            final_pwm = int(pwm_mag) if speed_ms > 0 else -int(pwm_mag)
            target_pwms.append(final_pwm)

        # Ramping Logic (Apply to the calculated targets)
        for i in range(4):
            diff = target_pwms[i] - self.current_pwms[i]
            if abs(diff) > self.ramp_step:
                step = self.ramp_step if diff > 0 else -self.ramp_step
                self.current_pwms[i] += step
            else:
                self.current_pwms[i] = target_pwms[i]

        pwm_message = {
            "pwm1": -self.current_pwms[1],
            "pwm2": self.current_pwms[2],
            "pwm3": self.current_pwms[0],
            "pwm4": -self.current_pwms[3]
        }

        out = String()
        out.data = json.dumps(pwm_message)
        self.pub_mcu_out.publish(out)
        self.last_cmd_time = time.time()    
        
    def _idle_check(self):
        now = time.time()
        if self.last_cmd_time is None:
            return
        if (now - self.last_cmd_time) > self.idle_timeout:
            self.current_pwms = [0, 0, 0, 0] 
            stop_msg = {"pwm1": 0, "pwm2": 0, "pwm3": 0, "pwm4": 0}
            out = String()
            out.data = json.dumps(stop_msg)
            self.pub_mcu_out.publish(out)
            self.last_cmd_time = None

def main(args=None):
    rclpy.init(args=args)
    node = CmdvelToMcu()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()