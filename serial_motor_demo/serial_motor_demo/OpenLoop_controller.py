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
        self.declare_parameter('scale_factor', 100.0)   # multiplies wheel speed (m/s) -> pwm units
        # idle timeout (seconds) after which we send zeros
        self.declare_parameter('idle_timeout', 0.05)
        # topic names
        self.declare_parameter('cmd_vel_in_topic', 'cmd_vel_out')
        self.declare_parameter('mcu_out_topic', 'mcu/out')

        self.L = float(self.get_parameter('wheel_L').get_parameter_value().double_value)
        self.W = float(self.get_parameter('wheel_W').get_parameter_value().double_value)
        self.max_pwm = int(self.get_parameter('max_pwm').get_parameter_value().integer_value)
        self.scale_factor = float(self.get_parameter('scale_factor').get_parameter_value().double_value)
        self.idle_timeout = float(self.get_parameter('idle_timeout').get_parameter_value().double_value)
        cmd_topic = self.get_parameter('cmd_vel_in_topic').get_parameter_value().string_value
        mcu_out_topic = self.get_parameter('mcu_out_topic').get_parameter_value().string_value

        # publisher to arbiter
        self.pub_mcu_out = self.create_publisher(String, mcu_out_topic, 10)
        # subscribe to merged cmd_vel
        self.sub_cmd = self.create_subscription(Twist, cmd_topic, self.cb_cmdvel, 20)

        # internal state
        self.last_cmd_time = None
        # timer to check idle and publish zero if needed
        self.create_timer(0.1, self._idle_check)

        self.get_logger().info(f"cmdvel_to_mcu listening on '{cmd_topic}', publishing to '{mcu_out_topic}'")

    def cb_cmdvel(self, msg: Twist):
        # Twist: linear.x forward, linear.y left, angular.z CCW
        vx = float(msg.linear.x)
        vy = float(msg.linear.y)
        wz = float(msg.angular.z)

        # mecanum wheel mixing (order: FL, FR, RL, RR)
        wheel_speeds = [
            vx - vy - wz * (self.L + self.W),  # Front Left
            vx + vy + wz * (self.L + self.W),  # Front Right
            vx + vy - wz * (self.L + self.W),  # Rear Left
            vx - vy + wz * (self.L + self.W)   # Rear Right
        ]

        # scale to PWM range and clamp
        pwm_values = []
        for speed in wheel_speeds:
            pwm = int(round(speed * self.scale_factor))
            if pwm > self.max_pwm:
                pwm = self.max_pwm
            elif pwm < -self.max_pwm:
                pwm = -self.max_pwm
            pwm_values.append(pwm)

        # map to MCU order and sign as before (keep existing mapping)
        pwm_message = {
            "pwm1": -pwm_values[1],
            "pwm2": -pwm_values[0],
            "pwm3": -pwm_values[3],
            "pwm4": -pwm_values[2]
        }

        out = String()
        out.data = json.dumps(pwm_message)
        self.pub_mcu_out.publish(out)
        self.get_logger().debug(f"Published PWM to mcu/out: {out.data}")

        # update last seen time
        self.last_cmd_time = time.time()

    def _idle_check(self):
        now = time.time()
        if self.last_cmd_time is None:
            return
        if (now - self.last_cmd_time) > self.idle_timeout:
            # publish zero PWM once and clear timestamp so we don't spam
            stop_msg = {"pwm1": 0, "pwm2": 0, "pwm3": 0, "pwm4": 0}
            out = String()
            out.data = json.dumps(stop_msg)
            self.pub_mcu_out.publish(out)
            self.get_logger().info("Idle timeout reached: sent stop PWM to MCU")
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
