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
        self.declare_parameter('max_pwm', 35)
        self.declare_parameter('scale_factor', 100.0)
        
        # RAMPING PARAMETER (Max PWM change per callback)
        # Example: If set to 2, it takes longer to reach max speed than if set to 10.
        self.declare_parameter('ramp_step', 4) 

        # STRAFING GAIN
        self.declare_parameter('strafe_gain', 0.0) 

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
        
        cmd_topic = self.get_parameter('cmd_vel_in_topic').get_parameter_value().string_value
        mcu_out_topic = self.get_parameter('mcu_out_topic').get_parameter_value().string_value

        self.pub_mcu_out = self.create_publisher(String, mcu_out_topic, 10)
        self.sub_cmd = self.create_subscription(Twist, cmd_topic, self.cb_cmdvel, 20)

        self.last_cmd_time = None
        
        # Initialize current state for ramping [FL, FR, RL, RR]
        self.current_pwms = [0, 0, 0, 0] 

        self.create_timer(0.1, self._idle_check)

        self.get_logger().info(f"Active. Max PWM: {self.max_pwm}, Ramp Step: {self.ramp_step}")

    def cb_cmdvel(self, msg: Twist):
        vx = float(msg.linear.x)
        vy = float(msg.linear.y) * self.strafe_gain
        wz = float(msg.angular.z)

        # Dynamic Limit Logic
        current_limit = self.max_pwm
        if abs(vy) > 0.01: 
            current_limit = int(self.max_pwm * self.strafe_gain)

        geom = self.L + self.W
        
        # Raw Target Calculation
        target_speeds = [
            vx - vy - wz * geom,  # FL
            vx + vy + wz * geom,  # FR
            vx + vy - wz * geom,  # RL
            vx - vy + wz * geom   # RR
        ]

        # Calculate Targets and Apply Ramp
        for i in range(4):
            # 1. Calculate ideal target PWM
            target_pwm = int(round(target_speeds[i] * self.scale_factor))
            
            # 2. Clamp target against limits first
            if target_pwm > current_limit: target_pwm = current_limit
            elif target_pwm < -current_limit: target_pwm = -current_limit

            # 3. Ramping: Move self.current_pwms[i] towards target_pwm by max ramp_step
            diff = target_pwm - self.current_pwms[i]
            
            if abs(diff) > self.ramp_step:
                step = self.ramp_step if diff > 0 else -self.ramp_step
                self.current_pwms[i] += step
            else:
                self.current_pwms[i] = target_pwm

        # Construct message using the RAMPED values (self.current_pwms)
        # Mapping: FL=0, FR=1, RL=2, RR=3
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
            # Reset internal state on timeout
            self.current_pwms = [0, 0, 0, 0] 
            
            stop_msg = {"pwm1": 0, "pwm2": 0, "pwm3": 0, "pwm4": 0}
            out = String()
            out.data = json.dumps(stop_msg)
            self.pub_mcu_out.publish(out)
            self.last_cmd_time = None
            self.last_pwm_values = [0, 0, 0, 0] 

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