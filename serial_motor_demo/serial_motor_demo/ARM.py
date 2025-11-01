#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import json
import time

class JoystickToSerial(Node):
    def __init__(self):
        super().__init__('joystick_to_serial')

        # Movement parameters
        self.a = self.b = self.c = self.d = self.e = self.x = self.y = self.o = 0
        self.bol = 0

        # Movement list (based on button logic)
        self.movement_list = [
            {"d1":self.y,"d2":self.a,"d3":self.b,"d4":self.c,"d5":self.d,"d6":self.e,"h":self.bol},
            {"angle_dof_1":0,"angle_dof_2":0,"angle_dof_3":0,"angle_dof_4":0,"angle_dof_5":0,"angle_dof_6":-5,"angle_gripper":0,"hold":False},
            {"angle_dof_1":0,"angle_dof_2":0,"angle_dof_3":0,"angle_dof_4":0,"angle_dof_5":0,"angle_dof_6":0,"angle_gripper":0,"hold":True},
            {"angle_dof_1":0,"angle_dof_2":0,"angle_dof_3":0,"angle_dof_4":0,"angle_dof_5":0,"angle_dof_6":0,"angle_gripper":90,"hold":False},
            {"angle_dof_1":0,"angle_dof_2":self.a,"angle_dof_3":0,"angle_dof_4":0,"angle_dof_5":180-self.a,"angle_dof_6":0,"angle_gripper":0,"hold":False},
        ]

        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

    def send_json(self, data):
        json_string = json.dumps(data)
        print(f"Sent: {json_string}")  # Print instead of sending via serial
        self.get_logger().info(f"Sent: {json_string}")

    def joy_callback(self, msg: Joy):
        # PS4 Buttons
        if msg.buttons[0]:  # Cross
            self.b = 1
            self.a = 0
        if msg.buttons[2]:  # Triangle
            self.o = 0
            self.y += 5
        if msg.buttons[3]:  # Square
            self.o = 1
            self.y -= 5
        if msg.buttons[1]:  # Circle
            self.b = 1
            self.a = 1
        if msg.buttons[4]:  # L1
            self.e -= 5
        if msg.buttons[5]:  # R1
            self.e += 5
        if msg.buttons[11]:  # L2
            self.bol = 1
        if msg.buttons[12]:
            self.bol = 0

        # D-pad (hat) using axes[6] and axes[7]
        hat_x = msg.axes[6]
        hat_y = msg.axes[7]
        if hat_x == 1.0:  # Right
            self.c += 5
        if hat_x == -1.0:  # Left
            self.c -= 5
        if hat_y == -1.0:  # Up
            self.d += 5
        if hat_y == 1.0:  # Down
            self.d -= 5

        self.send_json({"d1":self.y,"d2":self.a,"d3":self.b,"d4":self.c,"d5":self.d,"d6":self.e,"h":self.bol})
        self.b = 0  # reset
        
def main(args=None):
    rclpy.init(args=args)
    node = JoystickToSerial()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
