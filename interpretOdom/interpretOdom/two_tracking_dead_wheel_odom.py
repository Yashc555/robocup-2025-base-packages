#!/usr/bin/env python3
"""
two_dead_wheels_with_imu.py

Two dead-wheel odometry node using IMU yaw in DEGREES (0..360).
 - expects JSON lines on serial with fields:
     e1, e2   (encoder counts)
     yaw      (IMU yaw in DEGREES, 0..360, wraps back to 0)
 - pods: two pods [(x,y,phi), (x,y,phi)] (meters, radians)
 - ticks_per_rev: one value per pod (TPR)
 - rotation artifact per wheel: coeff = (-cosφ * y + sinφ * x); rot_comp = coeff * dtheta
 - integrate body deltas (x = forward, y = left) into world using avg-heading
"""
import math
import time
import json
from collections import deque

import numpy as np
if not hasattr(np, "float"):
    np.float = float
if not hasattr(np, "maximum_sctype"):
    np.maximum_sctype = lambda t: np.float64

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import TransformStamped, Quaternion, PoseStamped
import tf_transformations
import tf2_ros
import serial

def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

def wrap_to_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi

class TwoDeadWheelImuNode(Node):
    def __init__(self):
        super().__init__('two_dead_wheel_imu_odom')

        # --- params (tweak to match your hardware) ---
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('ticks_per_rev', [2400, 2400])  # one per pod
        self.declare_parameter('wheel_radius', 0.050)          # meters
        # pods as flat [x,y,phi, x,y,phi] (meters, radians). Use the two pods you have.
        self.declare_parameter('pods', [0.0, 0.1325, 0.0,   0.113, 0.0, math.pi/2.0])
        self.declare_parameter('frame_odom', 'odom')
        self.declare_parameter('frame_base', 'base_link')
        self.declare_parameter('encoder_map', [0, 1])   # map e1->pod0, e2->pod1
        self.declare_parameter('encoder_signs', [1, 1]) # sign flips if needed
        self.declare_parameter('publish_path', True)

        # --- load params ---
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        tpr_param = self.get_parameter('ticks_per_rev').get_parameter_value().integer_array_value
        self.tpr = [float(v) for v in tpr_param]
        self.wheel_r = float(self.get_parameter('wheel_radius').get_parameter_value().double_value)
        pods_param = self.get_parameter('pods').get_parameter_value().double_array_value
        if len(pods_param) % 3 != 0:
            raise RuntimeError("pods parameter must be a flat list of 3*N doubles")
        self.pods = [(pods_param[i], pods_param[i+1], pods_param[i+2]) for i in range(0, len(pods_param), 3)]
        if len(self.pods) < 2:
            raise RuntimeError("Expect at least 2 pods for two-wheel odometry")
        self.pods = self.pods[:2]  # only first two pods used

        self.encoder_map = self.get_parameter('encoder_map').get_parameter_value().integer_array_value
        self.encoder_signs = self.get_parameter('encoder_signs').get_parameter_value().integer_array_value
        self.frame_odom = self.get_parameter('frame_odom').get_parameter_value().string_value
        self.frame_base = self.get_parameter('frame_base').get_parameter_value().string_value
        self.publish_path = self.get_parameter('publish_path').get_parameter_value().bool_value

        self.get_logger().info(f"pods={self.pods}")
        self.get_logger().info(f"tpr={self.tpr}, encoder_map={self.encoder_map}, encoder_signs={self.encoder_signs}")

        # --- pubs / tf ---
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        if self.publish_path:
            self.path_pub = self.create_publisher(Path, 'odom_path', 10)
            self.path_history = deque(maxlen=1000)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # --- serial ---
        try:
            self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=1)
            time.sleep(0.05)
            self.get_logger().info(f"Opened serial {self.serial_port} @ {self.baudrate}")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial {self.serial_port}: {e}")
            raise

        # --- state ---
        self.prev_ticks = None
        self.prev_time = None
        self.prev_yaw = None       # radians (we store converted value)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # identify which is lateral and which is forward by phi
        self.lateral_idx = None
        self.forward_idx = None
        self._identify_roles_from_pods()

        # start timer to read serial
        self.create_timer(0.005, self.read_loop)  # 200 Hz

    def _identify_roles_from_pods(self):
        lateral = None
        forward = None
        for i, (_, _, phi) in enumerate(self.pods):
            p = wrap_to_pi(phi)
            if abs(p) < math.pi/4:
                lateral = i
            if abs(abs(p) - math.pi/2) < math.pi/4:
                forward = i
        if lateral is None:
            lateral = 0
        if forward is None:
            forward = 1 if lateral == 0 else 0
        self.lateral_idx = lateral
        self.forward_idx = forward
        self.get_logger().info(f"identified roles: lateral={self.lateral_idx}, forward={self.forward_idx}")

    def _simple_tick_jump(self, d_ticks, dt):
        max_rate_ticks_per_sec = 100000.0
        if np.any(np.abs(d_ticks) > max_rate_ticks_per_sec * max(dt, 1e-6)):
            return True
        return False

    def _deg_to_rad_normalized(self, deg_val):
        """Convert degrees (any range) to radians and normalize to [-pi,pi]."""
        try:
            d = float(deg_val) % 360.0
        except Exception:
            return None
        return wrap_to_pi(math.radians(d))

    def _handle_sample(self, mapped_ticks, imu_yaw_deg, sample_time_sec):
        """
        mapped_ticks: list len 2 (already mapped & signed)
        imu_yaw_deg: yaw in DEGREES (0..360 wrap)
        sample_time_sec: seconds (float)
        """
        ticks = np.array(mapped_ticks, dtype=np.int64)

        # convert imu yaw degrees -> radians (normalized)
        imu_yaw_rad = self._deg_to_rad_normalized(imu_yaw_deg)
        if imu_yaw_rad is None:
            self.get_logger().warn("Bad IMU yaw value, skipping sample")
            return False

        # init baseline
        if self.prev_ticks is None:
            self.prev_ticks = ticks
            self.prev_time = sample_time_sec
            self.prev_yaw = imu_yaw_rad
            self.yaw = imu_yaw_rad
            self.get_logger().info("Initialized encoders + imu baseline")
            return True

        d_ticks = ticks - self.prev_ticks
        dt = sample_time_sec - self.prev_time
        if dt <= 0:
            dt = 1e-3

        if self._simple_tick_jump(d_ticks, dt):
            self.get_logger().warn("Tick jump detected - rebaseline")
            self.prev_ticks = ticks
            self.prev_time = sample_time_sec
            self.prev_yaw = imu_yaw_rad
            self.yaw = imu_yaw_rad
            return False

        # ticks -> meters per pod
        circ = 2.0 * math.pi * self.wheel_r
        d_meters = np.zeros_like(d_ticks, dtype=float)
        for i in range(len(d_ticks)):
            tpr_i = float(self.tpr[i]) if i < len(self.tpr) else float(self.tpr[0])
            d_meters[i] = float(d_ticks[i]) * (circ / tpr_i)

        # dtheta from IMU (wrapped difference)
        dtheta = wrap_to_pi(imu_yaw_rad - self.prev_yaw)

        # compute rotation coeffs and remove rotational contribution
        rot_coeff = []
        for i, (xi, yi, phi) in enumerate(self.pods):
            c = math.cos(phi)
            s = math.sin(phi)
            rot_coeff.append(-c * yi + s * xi)
        rot_coeff = np.array(rot_coeff, dtype=float)
        rot_comp = rot_coeff * dtheta
        d_trans = d_meters - rot_comp   # corrected per-wheel translation

        # components (lateral index gives X-wheel, forward index gives Y-wheel)
        dx_corrected = d_trans[self.lateral_idx]
        dy_corrected = d_trans[self.forward_idx]

        # body frame -> ROS body: x = forward, y = left
        body_dx = dy_corrected
        body_dy = -dx_corrected

        # world integration using average heading
        avg_h = self.yaw + dtheta / 2.0
        cos_h = math.cos(avg_h)
        sin_h = math.sin(avg_h)
        dx_world = body_dx * cos_h - body_dy * sin_h
        dy_world = body_dx * sin_h + body_dy * cos_h

        self.x += dx_world
        self.y += dy_world
        # trust IMU absolute yaw
        self.yaw = wrap_to_pi(imu_yaw_rad)

        # publish odom
        sec = int(sample_time_sec)
        nsec = int((sample_time_sec - sec) * 1e9)
        hdr = Header()
        hdr.stamp.sec = sec
        hdr.stamp.nanosec = nsec
        hdr.frame_id = self.frame_odom

        odom = Odometry()
        odom.header = hdr
        odom.child_frame_id = self.frame_base
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = yaw_to_quaternion(self.yaw)
        odom.twist.twist.linear.x = float(body_dx / dt)
        odom.twist.twist.linear.y = float(body_dy / dt)
        odom.twist.twist.angular.z = float(dtheta / dt)
        self.odom_pub.publish(odom)

        # path + tf
        if self.publish_path:
            ps = PoseStamped()
            ps.header = hdr
            ps.pose = odom.pose.pose
            self.path_history.append(ps)
            path_msg = Path()
            path_msg.header = hdr
            path_msg.poses = list(self.path_history)
            self.path_pub.publish(path_msg)

        t = TransformStamped()
        t.header = hdr
        t.child_frame_id = self.frame_base
        t.header.frame_id = self.frame_odom
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = yaw_to_quaternion(self.yaw)
        self.tf_broadcaster.sendTransform(t)

        # store baseline
        self.prev_ticks = ticks
        self.prev_time = sample_time_sec
        self.prev_yaw = imu_yaw_rad
        return True

    def read_loop(self):
        # read available lines (drain lightly)
        try:
            waiting = self.ser.in_waiting
        except Exception as e:
            self.get_logger().warn(f"Serial in_waiting error: {e}")
            waiting = 0

        lines = []
        if waiting == 0:
            try:
                raw = self.ser.readline()
            except Exception as e:
                self.get_logger().warn(f"Serial read error: {e}")
                return
            if not raw:
                return
            try:
                s = raw.decode('utf-8', errors='ignore').strip()
            except Exception:
                return
            if s:
                lines = [s]
        else:
            max_lines = 500
            while self.ser.in_waiting > 0 and len(lines) < max_lines:
                try:
                    raw = self.ser.readline()
                except Exception:
                    break
                if not raw:
                    break
                try:
                    s = raw.decode('utf-8', errors='ignore').strip()
                except Exception:
                    s = ''
                if s:
                    lines.append(s)

        processed_any = False
        for line in lines:
            try:
                data = json.loads(line)
            except Exception:
                continue

            # parse two encoders (fall back keys a,b if you use those)
            try:
                raw_enc = [int(data.get('e1', data.get('a', 0))), int(data.get('e2', data.get('b', 0)))]
            except Exception:
                continue

            # parse yaw (degrees, 0..360)
            raw_y = data.get('yaw', data.get('imu_yaw', None))
            if raw_y is None:
                imu_y_deg = 0.0 if self.prev_yaw is None else math.degrees(self.prev_yaw)
            else:
                try:
                    imu_y_deg = float(raw_y)
                except Exception:
                    imu_y_deg = 0.0 if self.prev_yaw is None else math.degrees(self.prev_yaw)

            # map + sign to pod order (length 2)
            mapped = [0] * len(self.pods)
            for i_in, val in enumerate(raw_enc):
                pod_idx = self.encoder_map[i_in] if i_in < len(self.encoder_map) else i_in
                sign = int(self.encoder_signs[i_in]) if i_in < len(self.encoder_signs) else 1
                if pod_idx >= len(mapped):
                    continue
                mapped[pod_idx] = int(sign * val)

            now_time = self.get_clock().now().to_msg()
            sample_time_sec = now_time.sec + now_time.nanosec * 1e-9

            ok = self._handle_sample(mapped, imu_y_deg, sample_time_sec)
            if ok:
                processed_any = True

        if not processed_any:
            return

def main(args=None):
    rclpy.init(args=args)
    node = TwoDeadWheelImuNode()
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
