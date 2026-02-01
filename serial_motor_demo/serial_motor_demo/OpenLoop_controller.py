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
        self.declare_parameter('max_pwm', 30)
        self.declare_parameter('scale_factor', 100.0)
        
        # --- NEW: Dual Thresholds ---
        # Threshold for standard movement (forward/turn)
        self.declare_parameter('min_pwm_threshold_normal', 20)
        # Threshold specifically for strafing (sideways)
        self.declare_parameter('min_pwm_threshold_strafe', 20)
        
        # RAMPING PARAMETER
        self.declare_parameter('ramp_step', 4) 

        # STRAFING GAIN
        self.declare_parameter('strafe_gain',1.2) 

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
        self.get_logger().info(f"Active. Normal Min: {self.min_normal}, Strafe Min: {self.min_strafe}")

    def cb_cmdvel(self, msg: Twist):
        vx = float(msg.linear.x)
        vy = float(msg.linear.y) * self.strafe_gain
        wz = -float(msg.angular.z)

        # Detect Strafing and select Threshold/Limits
        is_strafing = abs(vy) > 0.01
        
        if is_strafing:
            current_limit = int(self.max_pwm * self.strafe_gain)
            active_min_threshold = self.min_strafe
        else:
            current_limit = self.max_pwm
            active_min_threshold = self.min_normal

        geom = self.L + self.W
        
        target_speeds = [
            vx - vy - wz * geom,  # FL
            vx + vy + wz * geom,  # FR
            vx + vy - wz * geom,  # RL
            vx - vy + wz * geom   # RR
        ]

        for i in range(4):
            # 1. Calculate ideal target PWM
            target_pwm = int(round(target_speeds[i] * self.scale_factor))
            
            # 2. Clamp target against max limits
            if target_pwm > current_limit: target_pwm = current_limit
            elif target_pwm < -current_limit: target_pwm = -current_limit

            # 3. Ramping
            diff = target_pwm - self.current_pwms[i]
            if abs(diff) > self.ramp_step:
                step = self.ramp_step if diff > 0 else -self.ramp_step
                self.current_pwms[i] += step
            else:
                self.current_pwms[i] = target_pwm

        # 4. Final Output Processing: Dynamic Threshold Logic
        final_pwms = []
        for val in self.current_pwms:
            abs_val = abs(val)
            # Use the active_min_threshold determined by is_strafing
            if 0 < abs_val <= active_min_threshold:
                new_val = active_min_threshold + 1
                final_pwms.append(new_val if val > 0 else -new_val)
            else:
                final_pwms.append(val)

        pwm_message = {
            "pwm1": -final_pwms[1],
            "pwm2": final_pwms[2],
            "pwm3": final_pwms[0],
            "pwm4": -final_pwms[3]
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