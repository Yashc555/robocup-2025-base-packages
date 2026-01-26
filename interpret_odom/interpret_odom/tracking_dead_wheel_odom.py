#!/usr/bin/env python3
#
"""
tracking_dead_wheel_node.py  -- dead-wheel odometry using the simple 3-wheel formula.
Reads MCU JSON lines from ROS topic `mcu/in` (std_msgs/String).
"""
import numpy as np
if not hasattr(np, "float"):
    np.float = float
if not hasattr(np, "maximum_sctype"):
    np.maximum_sctype = lambda t: np.float64

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
import tf_transformations
import tf2_ros
import json
import time
import math
import pprint
from collections import deque
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import String

def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

class DeadWheelOdomNode(Node):
    def __init__(self):
        super().__init__('dead_wheel_odom')

        # params
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('ticks_per_rev_front', 1200)
        self.declare_parameter('ticks_per_rev_left', 1200)
        self.declare_parameter('ticks_per_rev_right', 1200)
        self.declare_parameter('wheel_radius', 0.03)
        self.declare_parameter('frame_odom', 'odom')
        self.declare_parameter('frame_base', 'base_link')
        self.declare_parameter('publish_marker', True)
        self.declare_parameter('marker_scale', [0.2, 0.2, 0.08])

        # ---------------------------------------------------------
        # CRITICAL: 'pods' MUST be defined in this specific order:
        # [ POD_FRONT, POD_LEFT, POD_RIGHT ]
        # ---------------------------------------------------------
        self.declare_parameter(
            #with base upside down
            # 'pods',
            # [
            #     0.0,  0.106, math.pi/2.0,
            #     -0.07, 0.064,  0.0,
            #     0.075,  0.064, 0.0
            # ]
            #with glue
            # 'pods',
            # [
            #     0.0,  0.124, math.pi/2.0,
            #     -0.066, -0.065,  0.0,
            #     0.073,  -0.065, 0.0
            # ]
            # #with marker on acrylic
            # 'pods',
            # [
            #     0.0,  0.124, math.pi/2.0,
            #     -0.066, -0.06,  0.0,
            #     0.07,  -0.06, 0.0
            # ]


            #my bs
            #0.03 radians
                        'pods',
            [
<<<<<<< HEAD:interpretOdom/interpretOdom/tracking_dead_wheel_odom.py
                0.0,  0.106, math.pi/2.0,
                -0.07, 0.064,  0.0,
                0.075,  0.064, 0.0
            ]
        )

        self.declare_parameter('encoder_map', [0, 1, 2])
        self.declare_parameter('encoder_signs', [-1, 1, -1])
=======
                0.0,  0.124, (math.pi/2.0),
                -0.066, -0.06,  0.0,
                0.07,  -0.06, 0.0
            ]
        )

        # MAPPING PARAMETERS
        # Defines which input key (a,b,c) goes to which physical wheel.
        # Order: [Key for Front, Key for Left, Key for Right]
        self.declare_parameter('inputs_front_left_right', ['a', 'c', 'b'])

        # Signs apply to the INPUT KEYS [a, b, c] directly
        self.declare_parameter('encoder_signs', [1, -1, 1])
>>>>>>> c08b57bedbf8fe37f296aafaad7a51a286fda3e0:interpret_odom/interpret_odom/tracking_dead_wheel_odom.py

        # --------------------------------
        # load params
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.tpr_front = float(self.get_parameter('ticks_per_rev_front').get_parameter_value().integer_value)
        self.tpr_left  = float(self.get_parameter('ticks_per_rev_left').get_parameter_value().integer_value)
        self.tpr_right = float(self.get_parameter('ticks_per_rev_right').get_parameter_value().integer_value)
        self.wheel_r = float(self.get_parameter('wheel_radius').get_parameter_value().double_value)
        self.frame_odom = self.get_parameter('frame_odom').get_parameter_value().string_value
        self.frame_base = self.get_parameter('frame_base').get_parameter_value().string_value
        self.publish_marker = self.get_parameter('publish_marker').get_parameter_value().bool_value
        ms = self.get_parameter('marker_scale').get_parameter_value().double_array_value
        self.marker_scale = Vector3(x=ms[0], y=ms[1], z=ms[2])

        pods_param = self.get_parameter('pods').get_parameter_value().double_array_value
        self.pods = []
        for i in range(0, len(pods_param), 3):
            self.pods.append((pods_param[i], pods_param[i+1], pods_param[i+2]))

        self.input_map = self.get_parameter('inputs_front_left_right').get_parameter_value().string_array_value
        self.encoder_signs = self.get_parameter('encoder_signs').get_parameter_value().integer_array_value

        self.get_logger().info(f"pods (Fixed: Front, Left, Right): {self.pods}")
        self.get_logger().info(f"Mapping: Front='{self.input_map[0]}', Left='{self.input_map[1]}', Right='{self.input_map[2]}'")
        self.get_logger().info(f"Signs (for a,b,c): {self.encoder_signs}")

        # publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        if self.publish_marker:
            self.marker_pub = self.create_publisher(Marker, 'robot_marker', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)

        # state
        self.prev_ticks = None
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.prev_time = None

        # HARDCODED ROLES per user request
        # 0 is always Front, 1 is always Left, 2 is always Right
        self.lateral_idx = 0
        self.left_idx = 1
        self.right_idx = 2

        # after existing publishers
        self.path_pub = self.create_publisher(Path, 'odom_path', 10)
        self.path_history = deque(maxlen=1000)

        # subscribe to arbiter input
        self.create_subscription(String, 'mcu/in', self._mcu_line_cb, 40)

    def _is_tick_jump(self, d_ticks, dt):
        max_rate_ticks_per_sec = 50000.0
        safety = 4.0
        max_allowed = max_rate_ticks_per_sec * max(dt, 1e-3) * safety
        if np.any(np.abs(d_ticks) > max_allowed):
            return True
        if np.any(np.abs(d_ticks) > 1e7):
            return True
        return False

    def _handle_sample(self, d_front_raw, d_left_raw, d_right_raw, sample_time_sec):
        # Construct the tick vector [Front, Left, Right]
        ticks = np.array([d_front_raw, d_left_raw, d_right_raw], dtype=np.int64)

        # --- baseline / init ---
        if self.prev_ticks is None:
            self.prev_ticks = ticks
            self.prev_time = sample_time_sec
            self.get_logger().info("Initialized encoder baseline (first sample)")
            return True

        # delta ticks
        d_ticks = ticks - self.prev_ticks
        dt = sample_time_sec - self.prev_time
        if dt <= 0:
            dt = 1e-3

        # Unpack deltas
        delta_front_ticks = d_ticks[0]
        delta_left_ticks  = d_ticks[1]
        delta_right_ticks = d_ticks[2]

        # ticks → meters
        circ = 2.0 * math.pi * self.wheel_r
        d_front = delta_front_ticks * (circ / self.tpr_front)
        d_left  = delta_left_ticks  * (circ / self.tpr_left)
        d_right = delta_right_ticks * (circ / self.tpr_right)

        # --- GM0 math ---
        # Baseline: Dist between Right X and Left X
        baseline = self.pods[self.right_idx][0] - self.pods[self.left_idx][0]
        
        # F: Lateral offset (Y) of the Front wheel
        F = self.pods[self.lateral_idx][1]

        # --- compute wheel distances -> body motion ---
        dy_fwd = 0.5 * (d_left + d_right)
        dtheta = (d_right - d_left) / baseline
        
        # Fixed sign for Front/Lateral wheel
        dx_lat = d_front + F * dtheta

        # body frame displacements (robot frame: x forward, y left)
        # Mapping: Robot Y (Forward) -> ROS X, Robot X (Lateral) -> ROS Y
        body_dx = dy_fwd
        body_dy = -dx_lat

        # rotate to world using average heading during the step
        avg_heading = self.yaw + dtheta / 2.0
        cos_h, sin_h = math.cos(avg_heading), math.sin(avg_heading)
        step_dx_world = body_dx * cos_h - body_dy * sin_h
        step_dy_world = body_dx * sin_h + body_dy * cos_h

        # integrate pose (use step deltas)
        self.x += step_dx_world
        self.y += step_dy_world
        self.yaw = (self.yaw + dtheta + math.pi) % (2.0 * math.pi) - math.pi

        # velocities
        odom_forward_vel = body_dx / dt
        odom_left_vel = body_dy / dt
        odom_angular_vel = dtheta / dt

        # Debug: show both per-step motion and cumulative pose
        self.get_logger().info(
            f"ticks={d_ticks.tolist()} Δf={d_front:.5f} ΔL={d_left:.5f} ΔR={d_right:.5f} "
            f"dθ={math.degrees(dtheta):.2f}° dt={dt:.4f}s | "
            f"step_world_dx={step_dx_world:.4f}m step_world_dy={step_dy_world:.4f}m "
            f"pose_x={self.x:.4f}m pose_y={self.y:.4f}m yaw={math.degrees(self.yaw):.2f}°"
        )

        # publish odometry using ROS system time
        now = self.get_clock().now().to_msg()
        hdr = Header()
        hdr.stamp = now
        hdr.frame_id = self.frame_odom

        odom = Odometry()
        odom.header = hdr
        odom.child_frame_id = self.frame_base
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = yaw_to_quaternion(self.yaw)
        odom.twist.twist.linear.x = odom_forward_vel
        odom.twist.twist.linear.y = odom_left_vel
        odom.twist.twist.angular.z = odom_angular_vel
        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header = hdr
        t.header.stamp = hdr.stamp
        t.header.frame_id = self.frame_odom
        t.child_frame_id = self.frame_base
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = yaw_to_quaternion(self.yaw)
        self.tf_broadcaster.sendTransform(t)

        # append to path and publish
        ps = PoseStamped()
        ps.header = hdr
        ps.pose = odom.pose.pose
        self.path_history.append(ps)

        path_msg = Path()
        path_msg.header = hdr
        path_msg.poses = list(self.path_history)
        self.path_pub.publish(path_msg)

        # update baseline
        self.prev_ticks = ticks
        self.prev_time = sample_time_sec
        return True

    def _mcu_line_cb(self, msg: String):
        line = msg.data.strip()
        try:
            data = json.loads(line)
        except Exception:
            self.get_logger().warn(f"Bad JSON (ignored): {line}")
            return

        # sample timestamp
        sample_time_sec = None
        try:
            if 'time_ms' in data:
                sample_time_sec = float(data.get('time_ms')) * 1e-3
        except Exception:
            sample_time_sec = None

        if sample_time_sec is None:
            now_time = self.get_clock().now().to_msg()
            sample_time_sec = now_time.sec + now_time.nanosec * 1e-9

        # ---------------- IMU publishing ----------------
        try:
            imu_msg = Imu()
            sec = int(sample_time_sec)
            nsec = int((sample_time_sec - sec) * 1e9)
            imu_msg.header.stamp.sec = sec
            imu_msg.header.stamp.nanosec = nsec
            imu_msg.header.frame_id = self.frame_base
            imu_msg.orientation.x = float(data.get("orientation_x", 0.0))
            imu_msg.orientation.y = float(data.get("orientation_y", 0.0))
            imu_msg.orientation.z = float(data.get("orientation_z", 0.0))
            imu_msg.orientation.w = float(data.get("orientation_w", 1.0))
            imu_msg.angular_velocity.x = float(data.get("angular_velocity_x", 0.0))
            imu_msg.angular_velocity.y = float(data.get("angular_velocity_y", 0.0))
            imu_msg.angular_velocity.z = float(data.get("angular_velocity_z", 0.0))
            imu_msg.linear_acceleration.x = float(data.get("linear_acceleration_x", 0.0))
            imu_msg.linear_acceleration.y = float(data.get("linear_acceleration_y", 0.0))
            imu_msg.linear_acceleration.z = float(data.get("linear_acceleration_z", 0.0))
            self.imu_pub.publish(imu_msg)
        except Exception as e:
            self.get_logger().warn(f"IMU publish error: {e}")

        # ---------------- HARDCODED MAPPING LOGIC ----------------
        try:
            # 1. Read Raw Values
            raw_a = int(data.get('a', 0))
            raw_b = int(data.get('b', 0))
            raw_c = int(data.get('c', 0))
            
            # 2. Apply Signs (Encoder signs apply to a, b, c)
            val_a = raw_a * self.encoder_signs[0]
            val_b = raw_b * self.encoder_signs[1]
            val_c = raw_c * self.encoder_signs[2]
            
            # 3. Create a lookup dict
            values = {'a': val_a, 'b': val_b, 'c': val_c}
            
            # 4. Map to roles using the param 'inputs_front_left_right'
            # Default param is ['a', 'b', 'c']
            key_front = self.input_map[0]
            key_left  = self.input_map[1]
            key_right = self.input_map[2]
            
            ticks_front = values.get(key_front, 0)
            ticks_left  = values.get(key_left, 0)
            ticks_right = values.get(key_right, 0)
            
            self._handle_sample(ticks_front, ticks_left, ticks_right, sample_time_sec)
            
        except Exception as e:
            self.get_logger().warn(f"Encoder mapping error: {e}")
            return


def main(args=None):
    rclpy.init(args=args)
    node = DeadWheelOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()