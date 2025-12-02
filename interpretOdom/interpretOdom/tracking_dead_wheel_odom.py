#!/usr/bin/env python3
#
"""
tracking_dead_wheel_node.py  -- dead-wheel odometry using the simple 3-wheel formula.
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
import serial
from serial.tools import list_ports
import json
import time
import math
import pprint
from collections import deque
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from builtin_interfaces.msg import Time as BTime

def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

class DeadWheelOdomNode(Node):
    def __init__(self):
        super().__init__('dead_wheel_odom')

        # params
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('ticks_per_rev_front', 2400)
        self.declare_parameter('ticks_per_rev_left', 2400)
        self.declare_parameter('ticks_per_rev_right', 2400)
        # wheel_r is the radius of the dead wheel (meters)
        self.declare_parameter('wheel_radius', 0.03)
        self.declare_parameter('frame_odom', 'odom')
        self.declare_parameter('frame_base', 'base_link')
        self.declare_parameter('publish_marker', True)
        self.declare_parameter('marker_scale', [0.2, 0.2, 0.08])

        self.declare_parameter(
            'pods',
            [
                0.0,  0.095, math.pi/2.0,
                -0.1, 0.0,  0.0,
                0.095,  0.0,0.0
            ]
        )

        self.declare_parameter('encoder_map', [0, 1, 2])
        self.declare_parameter('encoder_signs', [1, 1, 1])

        # --------------------------------
        # load params
        param_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        # default to configured param, may be overridden by auto-detect below
        self.serial_port = param_port
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

        self.encoder_map = self.get_parameter('encoder_map').get_parameter_value().integer_array_value
        self.encoder_signs = self.get_parameter('encoder_signs').get_parameter_value().integer_array_value

        self.get_logger().info(f"pods: {self.pods}")
        self.get_logger().info(f"encoder_map: {self.encoder_map}, encoder_signs: {self.encoder_signs}")

        # publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        if self.publish_marker:
            self.marker_pub = self.create_publisher(Marker, 'robot_marker', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # serial
        try:
            self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=1)
            time.sleep(0.05)
            self.get_logger().info(f"Opened serial {self.serial_port} @ {self.baudrate}")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial {self.serial_port}: {e}")
            raise

        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)

        # state
        self.prev_ticks = None
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.prev_time = None

        # computed once from pods: identify lateral pod index and left/right indices
        self.lateral_idx = None
        self.left_idx = None
        self.right_idx = None
        self._identify_roles_from_pods()

        # try to auto-detect the encoder serial port by listening for one JSON message
        try:
            detected = self._auto_select_serial_port()
            if detected:
                self.get_logger().info(f"Auto-selected serial port: {detected}")
                self.serial_port = detected
            else:
                self.get_logger().info(f"No auto-detected serial device matched; using configured port: {self.serial_port}")
        except Exception as e:
            self.get_logger().warn(f"Auto-detect serial port failed: {e}; falling back to {self.serial_port}")

        # after existing publishers
        self.path_pub = self.create_publisher(Path, 'odom_path', 10)
        self.path_history = deque(maxlen=1000)

        # timer
        self.create_timer(0.005, self.read_loop)  # 200 Hz

    def _identify_roles_from_pods(self):
        lateral = None
        for i, (_, _, phi) in enumerate(self.pods):
            p = ((phi + math.pi) % (2*math.pi)) - math.pi
            if abs(abs(p) - math.pi/2) < 0.6:
                lateral = i
                break
        if lateral is None:
            maxy = -1.0
            for i, (_, y, _) in enumerate(self.pods):
                if abs(y) > maxy:
                    maxy = abs(y); lateral = i

        others = [i for i in range(len(self.pods)) if i != lateral]
        if len(others) != 2:
            self.get_logger().error("Pod count != 3, can't auto-identify roles reliably.")
            self.lateral_idx = lateral
            self.left_idx = others[0] if others else 0
            self.right_idx = others[1] if len(others) > 1 else 1
            return

        x0 = self.pods[others[0]][0]
        x1 = self.pods[others[1]][0]
        if x0 < x1:
            left, right = others[0], others[1]
        else:
            left, right = others[1], others[0]

        self.lateral_idx = lateral
        self.left_idx = left
        self.right_idx = right

        self.get_logger().info(f"Identified roles -> lateral_idx={self.lateral_idx}, left_idx={self.left_idx}, right_idx={self.right_idx}")

    def _is_tick_jump(self, d_ticks, dt):
        max_rate_ticks_per_sec = 50000.0
        safety = 4.0
        max_allowed = max_rate_ticks_per_sec * max(dt, 1e-3) * safety
        if np.any(np.abs(d_ticks) > max_allowed):
            return True
        if np.any(np.abs(d_ticks) > 1e7):
            return True
        return False

    def _handle_sample(self, mapped_ticks, sample_time_sec):
        ticks = np.array(mapped_ticks, dtype=np.int64)

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

        # ticks → meters
        circ = 2.0 * math.pi * self.wheel_r
        d_front = d_ticks[self.lateral_idx] * (circ / self.tpr_front)
        d_left  = d_ticks[self.left_idx]    * (circ / self.tpr_left)
        d_right = d_ticks[self.right_idx]   * (circ / self.tpr_right)

        # --- GM0 math ---
        baseline = self.pods[self.right_idx][0] - self.pods[self.left_idx][0]
        F = self.pods[self.lateral_idx][1]

        # --- compute wheel distances -> body motion ---
        dy_fwd = 0.5 * (d_left + d_right)
        dtheta = (d_right - d_left) / baseline
        dx_lat = d_front - F * dtheta

        # body frame displacements (robot frame: x forward, y left)
        body_dx = dy_fwd
        body_dy = dx_lat

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

        # publish odometry using MCU timestamp
        # publish odometry stamped with ROS system time (keeps timestamps consistent with LiDAR)
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

    def read_loop(self):
        try:
            waiting = self.ser.in_waiting
        except Exception as e:
            self.get_logger().warn(f"Serial in_waiting error: {e}")
            waiting = 0

        if waiting == 0:
            try:
                line_raw = self.ser.readline()
            except Exception as e:
                self.get_logger().warn(f"Serial read error: {e}")
                return
            if not line_raw:
                return
            try:
                line = line_raw.decode('utf-8', errors='ignore').strip()
            except Exception:
                return
            lines = [line]
        else:
            lines = []
            max_lines = 100
            while self.ser.in_waiting > 0 and len(lines) < max_lines:
                try:
                    raw_line = self.ser.readline()
                except Exception as e:
                    self.get_logger().warn(f"Serial read error while draining: {e}")
                    break
                if not raw_line:
                    break
                try:
                    s = raw_line.decode('utf-8', errors='ignore').strip()
                except Exception:
                    s = ''
                if s:
                    lines.append(s)
            self.get_logger().debug(f"Drained {len(lines)} lines from serial (in_waiting was {waiting})")

        processed_any = False
        for line in lines:
            try:
                data = json.loads(line)
            except Exception:
                self.get_logger().warn(f"Bad JSON (ignored): {line}")
                continue

            # sample timestamp: prefer MCU provided ms timestamp "time_ms"
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
                # put MCU timestamp into IMU header so all sensors share same time base
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

            # expected numeric encoder fields a,b,c
            try:
                raw = [int(data.get('a', 0)), int(data.get('b', 0)), int(data.get('c', 0))]
            except Exception:
                self.get_logger().warn(f"Encoder values not ints (ignored): {data}")
                continue

            # map + sign
            mapped = [0] * len(self.pods)
            for i_in, val in enumerate(raw):
                pod_idx = self.encoder_map[i_in] if i_in < len(self.encoder_map) else i_in
                sign = int(self.encoder_signs[i_in]) if i_in < len(self.encoder_signs) else 1
                mapped[pod_idx] = int(sign * val)

            ok = self._handle_sample(mapped, sample_time_sec)
            if ok:
                processed_any = True

        if not processed_any:
            return

    def _auto_select_serial_port(self, scan_baud=None, read_timeout=0.5, max_lines=20):
        """
        Scan available serial ports, open each briefly, read up to max_lines JSON lines,
        and select the port that publishes a JSON object with "name" == "STM_ENCODER.PWM".
        Returns the device string (e.g. '/dev/ttyACM0') or None if none matched.
        """
        try:
            ports = list_ports.comports()
        except Exception as e:
            self.get_logger().warn(f"list_ports.comports() failed: {e}")
            return None

        for p in ports:
            port_name = p.device
            try:
                ser = serial.Serial(port_name, scan_baud if scan_baud else self.baudrate, timeout=read_timeout)
                # small settle
                time.sleep(0.02)
            except Exception as e:
                # cannot open this port; skip
                self.get_logger().debug(f"Cannot open {port_name}: {e}")
                continue

            found = False
            try:
                lines_read = 0
                while lines_read < max_lines:
                    raw = ser.readline()
                    if not raw:
                        break
                    lines_read += 1
                    try:
                        s = raw.decode('utf-8', errors='ignore').strip()
                    except Exception:
                        continue
                    if not s:
                        continue
                    try:
                        data = json.loads(s)
                    except Exception:
                        continue
                    name = data.get('Name')
                    if isinstance(name, str) and name.strip() == 'STM_ENCODER.PWM':
                        found = True
                        break
            finally:
                try:
                    ser.close()
                except Exception:
                    pass

            if found:
                return port_name

        return None

def main(args=None):
    rclpy.init(args=args)
    node = DeadWheelOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down")
        try:
            node.ser.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
