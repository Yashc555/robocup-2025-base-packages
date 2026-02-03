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

        self.declare_parameter('wheel_L', 0.305)
        self.declare_parameter('wheel_W', 0.2175)
        
        self.declare_parameter('max_pwm', 20)
        self.declare_parameter('scale_factor', 100.0)
        
        self.declare_parameter('min_pwm_threshold_normal', 16)
        self.declare_parameter('min_pwm_threshold_strafe', 28)
        
        self.declare_parameter('ramp_step', 12) 
        self.declare_parameter('strafe_gain', 1.9) 

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
        
        self.min_normal = int(self.get_parameter('min_pwm_threshold_normal').get_parameter_value().integer_value)
        self.min_strafe = int(self.get_parameter('min_pwm_threshold_strafe').get_parameter_value().integer_value)
        
        cmd_topic = self.get_parameter('cmd_vel_in_topic').get_parameter_value().string_value
        mcu_out_topic = self.get_parameter('mcu_out_topic').get_parameter_value().string_value

        self.pub_mcu_out = self.create_publisher(String, mcu_out_topic, 10)
        self.sub_cmd = self.create_subscription(Twist, cmd_topic, self.cb_cmdvel, 20)

        self.last_cmd_time = None
        self.current_pwms = [0, 0, 0, 0] 

        self.create_timer(0.1, self._idle_check)
        self.get_logger().info(f"Active. Dynamic Strafing Enabled.")
        
        self.send_zero_pwm()

    def send_zero_pwm(self):
        stop_msg = {"pwm1": 0, "pwm2": 0, "pwm3": 0, "pwm4": 0}
        out = String()
        out.data = json.dumps(stop_msg)
        self.pub_mcu_out.publish(out)

    def cb_cmdvel(self, msg: Twist):
        self.last_cmd_time = time.time()

        raw_vx = float(msg.linear.x)
        raw_vy = float(msg.linear.y)
        wz = -float(msg.angular.z)

        total_linear_mag = abs(raw_vx) + abs(raw_vy)
        
        if total_linear_mag < 0.05: 
            strafe_ratio = 0.0
        else:
            strafe_ratio = abs(raw_vy) / total_linear_mag

        current_vy_gain = 1.0 + (strafe_ratio * (self.strafe_gain - 1.0))
        vx = raw_vx
        vy = raw_vy * current_vy_gain

        active_min_pwm = self.min_normal + (strafe_ratio * (self.min_strafe - self.min_normal))
        
        normal_max = self.max_pwm
        strafe_max = self.max_pwm * self.strafe_gain
        active_max_pwm = normal_max + (strafe_ratio * (strafe_max - normal_max))

        geom = self.L + self.W
        
        target_speeds_ms = [
            vx - vy - wz * geom,
            vx + vy + wz * geom,
            vx + vy - wz * geom,
            vx - vy + wz * geom
        ]

        NAV2_MAX_SPEED_MS = 0.09  
        
        target_pwms = []
        for speed_ms in target_speeds_ms:
            abs_speed = abs(speed_ms)
            
            if abs_speed > 0.01 and abs_speed < 0.02:
                abs_speed = 0.02
            
            if abs_speed < 0.01: 
                target_pwms.append(0)
                continue

            speed_ratio = (abs_speed - 0.02) / (NAV2_MAX_SPEED_MS - 0.02)
            speed_ratio = max(0.0, min(1.0, speed_ratio))
            
            pwm_range = active_max_pwm - active_min_pwm
            pwm_mag = active_min_pwm + (speed_ratio * pwm_range)
            
            final_pwm = int(pwm_mag) if speed_ms > 0 else -int(pwm_mag)
            target_pwms.append(final_pwm)

        if len(target_pwms) == 4:
            pwm_msg = {
                "pwm1": -target_pwms[1],
                "pwm2": target_pwms[2],
                "pwm3": target_pwms[0],
                "pwm4": -target_pwms[3]
            }
            out_msg = String()
            out_msg.data = json.dumps(pwm_msg)
            self.pub_mcu_out.publish(out_msg)
            self.current_pwms = target_pwms

    def _idle_check(self):
        now = time.time()
        if self.last_cmd_time is None:
            return
        if (now - self.last_cmd_time) > self.idle_timeout:
            self.current_pwms = [0, 0, 0, 0] 
            self.send_zero_pwm()
            self.last_cmd_time = None

def main(args=None):
    rclpy.init(args=args)
    node = CmdvelToMcu()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send_zero_pwm()
        time.sleep(0.1) 
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()