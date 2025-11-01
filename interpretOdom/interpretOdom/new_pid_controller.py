#!/usr/bin/env python3
"""
DOCUMENTATION:

Combined node for:
 - Dead-wheel odometry (3 omni encoders)
 - Mecanum 4-wheel PID joystick teleop (closed-loop using /odometry_filtered)
 - Single serial JSON I/O to/from MCU

JSON format expected from MCU (one line per sample, newline-terminated):
{'a': 0, 'b': 0, 'c': 0, 'd': 0,
 'orientation_x': 0.0, 'orientation_y': 0.0, 'orientation_z': 0.0, 'orientation_w': 0.0,
 'angular_velocity_x': 0.0, 'angular_velocity_y': 0.0, 'angular_velocity_z': 0.0,
 'linear_acceleration_x': 0.0, 'linear_acceleration_y': 0.0, 'linear_acceleration_z': 0.0,
 'yaw_deg': 0.0, 'roll_deg': 0.0, 'pitch_deg': 0.0}

Notes:
 - You said you have only 3 encoders (dead wheels). 'd' is a placeholder and ignored for control.
 - Closed-loop control is implemented at the **body level**: PID controllers run on body velocities
   (vx, vy, wz) using feedback from `/odometry_filtered` (robot_localization EKF). PID outputs
   modify the commanded body velocities, which are converted to wheel radial speeds and then
   into PWM values sent to the MCU.
 - This avoids per-wheel feedback (you don't have drive encoders) and uses odometry for the loop.

Tune these ROS parameters (defaults provided):
 - pid_vx_kp, pid_vx_ki, pid_vx_kd
 - pid_vy_kp, ...
 - pid_wz_kp, ...
 - max_wheel_speed (rad/s) and max_pwm (int) — used to scale wheel rad/s -> PWM.
 - control_rate_hz (default 50 Hz)

"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Joy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3, PoseStamped
from std_msgs.msg import Header
from collections import deque
import tf_transformations
import tf2_ros
import serial
import json
import math
import time
import numpy as np

# numpy compatibility
if not hasattr(np, "float"):
    np.float = float
if not hasattr(np, "maximum_sctype"):
    np.maximum_sctype = lambda t: np.float64


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


class PIDController:
    """Simple PID that returns float output (no int cast).
    max_output/min_output are saturation limits on the *output units* (e.g., m/s or rad/s).
    """
    def __init__(self, kp, ki, kd, max_output=float('inf'), min_output=-float('inf')):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.min_output = min_output
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, error, dt):
        if dt < 1e-4:
            return 0.0
        self.integral += error * dt
        # small deadband to limit integral windup
        if abs(error) < 1e-4:
            self.integral = 0.0
        derivative = (error - self.prev_error) / (dt + 1e-9)
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        output = max(min(output, self.max_output), self.min_output)
        self.prev_error = error
        return float(output)


class CombinedMecanumDeadWheelNode(Node):
    def __init__(self):
        super().__init__('combined_mecanum_deadwheel')

        # ---------------- parameters ----------------
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)

        # mecanum geometry
        self.declare_parameter('L', 0.32)
        self.declare_parameter('W', 0.24)
        self.declare_parameter('R', 0.05)     # drive wheel radius (m)

        # dead wheel odom params
        self.declare_parameter('ticks_per_rev_front', 2400)
        self.declare_parameter('ticks_per_rev_left', 2400)
        self.declare_parameter('ticks_per_rev_right', 2400)
        self.declare_parameter('wheel_radius', 0.0515)
        self.declare_parameter('frame_odom', 'odom')
        self.declare_parameter('frame_base', 'base_link')
        self.declare_parameter('pods', [0.0,  0.1345, math.pi/2.0, -0.109, 0.0,  0.0, 0.109,  0.0,0.0])
        self.declare_parameter('encoder_map', [0,1,2])
        self.declare_parameter('encoder_signs', [1,-1,1])

        # body-level PID gains (for closed-loop on odometry_filtered velocities)
        self.declare_parameter('pid_vx_kp', 1.2)
        self.declare_parameter('pid_vx_ki', 0.0)
        self.declare_parameter('pid_vx_kd', 0.02)
        self.declare_parameter('pid_vy_kp', 1.2)
        self.declare_parameter('pid_vy_ki', 0.0)
        self.declare_parameter('pid_vy_kd', 0.02)
        self.declare_parameter('pid_wz_kp', 4.0)
        self.declare_parameter('pid_wz_ki', 0.0)
        self.declare_parameter('pid_wz_kd', 0.1)

        # control scaling
        self.declare_parameter('max_wheel_speed', 20.0)   # rad/s (tune to your motors)
        self.declare_parameter('max_pwm', 255)
        self.declare_parameter('control_rate_hz', 50.0)

        # load parameters
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        self.L = float(self.get_parameter('L').get_parameter_value().double_value)
        self.W = float(self.get_parameter('W').get_parameter_value().double_value)
        self.R = float(self.get_parameter('R').get_parameter_value().double_value)

        self.tpr_front = float(self.get_parameter('ticks_per_rev_front').get_parameter_value().integer_value)
        self.tpr_left  = float(self.get_parameter('ticks_per_rev_left').get_parameter_value().integer_value)
        self.tpr_right = float(self.get_parameter('ticks_per_rev_right').get_parameter_value().integer_value)
        self.wheel_r = float(self.get_parameter('wheel_radius').get_parameter_value().double_value)
        self.frame_odom = self.get_parameter('frame_odom').get_parameter_value().string_value
        self.frame_base = self.get_parameter('frame_base').get_parameter_value().string_value

        pods_param = self.get_parameter('pods').get_parameter_value().double_array_value
        self.pods = []
        for i in range(0, len(pods_param), 3):
            self.pods.append((pods_param[i], pods_param[i+1], pods_param[i+2]))

        self.encoder_map = self.get_parameter('encoder_map').get_parameter_value().integer_array_value
        self.encoder_signs = self.get_parameter('encoder_signs').get_parameter_value().integer_array_value

        # body PID controllers
        kp = float(self.get_parameter('pid_vx_kp').get_parameter_value().double_value)
        ki = float(self.get_parameter('pid_vx_ki').get_parameter_value().double_value)
        kd = float(self.get_parameter('pid_vx_kd').get_parameter_value().double_value)
        self.pid_vx = PIDController(kp, ki, kd, max_output=2.0, min_output=-2.0)

        kp = float(self.get_parameter('pid_vy_kp').get_parameter_value().double_value)
        ki = float(self.get_parameter('pid_vy_ki').get_parameter_value().double_value)
        kd = float(self.get_parameter('pid_vy_kd').get_parameter_value().double_value)
        self.pid_vy = PIDController(kp, ki, kd, max_output=2.0, min_output=-2.0)

        kp = float(self.get_parameter('pid_wz_kp').get_parameter_value().double_value)
        ki = float(self.get_parameter('pid_wz_ki').get_parameter_value().double_value)
        kd = float(self.get_parameter('pid_wz_kd').get_parameter_value().double_value)
        self.pid_wz = PIDController(kp, ki, kd, max_output=4.0, min_output=-4.0)

        self.max_wheel_speed = float(self.get_parameter('max_wheel_speed').get_parameter_value().double_value)
        self.max_pwm = int(self.get_parameter('max_pwm').get_parameter_value().integer_value)
        self.control_rate_hz = float(self.get_parameter('control_rate_hz').get_parameter_value().double_value)

        # publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.path_pub = self.create_publisher(PoseStamped, 'odom_path_simple', 10)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # joystick
        self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        # subscribe to EKF filtered odom for closed-loop feedback
        self.create_subscription(Odometry, '/odometry_filtered', self.ekf_odom_callback, 10)

        # serial
        try:
            self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=0.1)
            time.sleep(0.05)
            self.get_logger().info(f"Opened serial {self.serial_port} @ {self.baudrate}")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial {self.serial_port}: {e}")
            raise

        # state for odom and control
        self.last_encoders_dead = [0,0,0]
        self.prev_ticks = None
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.prev_time = None
        self.lateral_idx = None
        self.left_idx = None
        self.right_idx = None
        self._identify_roles_from_pods()
        self.path_history = deque(maxlen=1000)

        # odometry_filtered feedback (body velocities)
        self.actual_vx = 0.0
        self.actual_vy = 0.0
        self.actual_wz = 0.0
        self.odom_received = False

        # desired body velocities from joystick
        self.desired_vx = 0.0
        self.desired_vy = 0.0
        self.desired_wz = 0.0

        # timing
        self.last_control_time = time.time()

        # start timers: serial reader + control loop (fixed rate)
        self.create_timer(0.005, self.read_loop)
        self.create_timer(1.0 / max(1.0, self.control_rate_hz), self.control_loop)

    # --------- odom helper (copied/adapted) ----------
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
        if self.prev_ticks is None:
            self.prev_ticks = ticks
            self.prev_time = sample_time_sec
            self.get_logger().info("Initialized encoder baseline (first sample)")
            return True
        d_ticks = ticks - self.prev_ticks
        dt = sample_time_sec - self.prev_time
        if dt <= 0:
            dt = 1e-3
        circ = 2.0 * math.pi * self.wheel_r
        d_front = d_ticks[self.lateral_idx] * (circ / self.tpr_front)
        d_left  = d_ticks[self.left_idx]    * (circ / self.tpr_left)
        d_right = d_ticks[self.right_idx]   * (circ / self.tpr_right)
        baseline = self.pods[self.right_idx][0] - self.pods[self.left_idx][0]
        F = self.pods[self.lateral_idx][1]
        dy_fwd = 0.5 * (d_left + d_right)
        dtheta = (d_right - d_left) / baseline
        dx_lat = d_front - F * dtheta
        body_dx = dy_fwd
        body_dy = dx_lat
        avg_heading = self.yaw + dtheta / 2.0
        cos_h, sin_h = math.cos(avg_heading), math.sin(avg_heading)
        dx_world = body_dx * cos_h - body_dy * sin_h
        dy_world = body_dx * sin_h + body_dy * cos_h
        self.x += dx_world
        self.y += dy_world
        self.yaw = (self.yaw + dtheta + math.pi) % (2.0 * math.pi) - math.pi
        odom_forward_vel = body_dx / dt
        odom_left_vel = body_dy / dt
        odom_angular_vel = dtheta / dt
        # publish odometry
        time_msg = self.get_clock().now().to_msg()
        hdr = Header()
        hdr.stamp = time_msg
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
        try:
            self.tf_broadcaster.sendTransform(t)
        except Exception:
            pass
        # append to path
        ps = PoseStamped()
        ps.header = hdr
        ps.pose = odom.pose.pose
        self.path_history.append(ps)
        # update baseline
        self.prev_ticks = ticks
        self.prev_time = sample_time_sec
        return True

    # ---------- serial read / process (STRICT FORMAT) ----------
    def read_loop(self):
        try:
            waiting = self.ser.in_waiting
        except Exception as e:
            self.get_logger().warn(f"Serial in_waiting error: {e}")
            waiting = 0
        lines = []
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
            max_lines = 200
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
        processed_any = False
        for line in lines:
            try:
                data = json.loads(line)
            except Exception:
                self.get_logger().warn(f"Bad JSON (ignored): {line}")
                continue

            # Strict validation: expect the exact keys present
            required_keys = {'a','b','c','d','orientation_x','orientation_y','orientation_z','orientation_w',
                             'angular_velocity_x','angular_velocity_y','angular_velocity_z',
                             'linear_acceleration_x','linear_acceleration_y','linear_acceleration_z',
                             'yaw_deg','roll_deg','pitch_deg'}
            if not required_keys.issubset(set(data.keys())):
                self.get_logger().warn(f"JSON missing required keys (ignored): {list(data.keys())}")
                continue

            # ----- IMU publish -----
            try:
                imu_msg = Imu()
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.header.frame_id = self.frame_base

                imu_msg.orientation.x = float(data['orientation_x'])
                imu_msg.orientation.y = float(data['orientation_y'])
                imu_msg.orientation.z = float(data['orientation_z'])
                imu_msg.orientation.w = float(data['orientation_w'])

                imu_msg.angular_velocity.x = float(data['angular_velocity_x'])
                imu_msg.angular_velocity.y = float(data['angular_velocity_y'])
                imu_msg.angular_velocity.z = float(data['angular_velocity_z'])

                imu_msg.linear_acceleration.x = float(data['linear_acceleration_x'])
                imu_msg.linear_acceleration.y = float(data['linear_acceleration_y'])
                imu_msg.linear_acceleration.z = float(data['linear_acceleration_z'])

                self.imu_pub.publish(imu_msg)
            except Exception as e:
                self.get_logger().warn(f"IMU publish error: {e}")

            # ----- dead-wheel encoders (a,b,c) -----
            try:
                dead_raw = [int(data['a']), int(data['b']), int(data['c'])]
            except Exception:
                self.get_logger().warn(f"Dead-wheel encoder parse error: {data}")
                continue
            # map + sign
            mapped = [0] * len(self.pods)
            for i_in, val in enumerate(dead_raw):
                pod_idx = self.encoder_map[i_in] if i_in < len(self.encoder_map) else i_in
                sign = int(self.encoder_signs[i_in]) if i_in < len(self.encoder_signs) else 1
                mapped[pod_idx] = int(sign * val)
            now_time = self.get_clock().now().to_msg()
            sample_time_sec = now_time.sec + now_time.nanosec * 1e-9
            try:
                self._handle_sample(mapped, sample_time_sec)
            except Exception as e:
                self.get_logger().warn(f"_handle_sample error: {e}")

            # ----- note: drive wheel encoders NOT PRESENT (d is placeholder) -----
            # we ignore 'd' and do NOT compute per-wheel speeds. Closed-loop uses /odometry_filtered instead.

            processed_any = True

        if not processed_any:
            return

    # EKF odom subscriber for closed-loop feedback
    def ekf_odom_callback(self, msg: Odometry):
        # odometry_filtered should provide body-frame velocities in child_frame
        try:
            self.actual_vx = float(msg.twist.twist.linear.x)
            self.actual_vy = float(msg.twist.twist.linear.y)
            self.actual_wz = float(msg.twist.twist.angular.z)
            self.odom_received = True
        except Exception:
            pass

    # ---------- control loop (body-level PID) ----------
    def control_loop(self):
        # wait for odometry before attempting closed-loop control
        if not self.odom_received:
            return

        now = time.time()
        dt = max(now - self.last_control_time, 1e-4)
        self.last_control_time = now

        # compute PID corrections on body velocities
        err_vx = self.desired_vx - self.actual_vx
        err_vy = self.desired_vy - self.actual_vy
        err_wz = self.desired_wz - self.actual_wz

        corr_vx = self.pid_vx.compute(err_vx, dt)
        corr_vy = self.pid_vy.compute(err_vy, dt)
        corr_wz = self.pid_wz.compute(err_wz, dt)

        # corrected command = desired + correction (units: m/s and rad/s)
        cmd_vx = self.desired_vx + corr_vx
        cmd_vy = self.desired_vy + corr_vy
        cmd_wz = self.desired_wz + corr_wz

        # convert to wheel radial speeds (rad/s)
        wheel_rads = self.calculate_wheel_speeds(cmd_vx, cmd_vy, cmd_wz)

        # map rad/s -> PWM using linear scaling
        pwm_vals = []
        for w in wheel_rads:
            # clamp wheel speed
            w_clamped = max(min(w, self.max_wheel_speed), -self.max_wheel_speed)
            pwm = int((w_clamped / self.max_wheel_speed) * self.max_pwm)
            pwm_vals.append(pwm)

        # pack and send PWM JSON (order preserved from earlier code)
        pwm_message = json.dumps({"pwm1": pwm_vals[3], "pwm2": pwm_vals[1], "pwm3": pwm_vals[0], "pwm4": pwm_vals[2]})
        try:
            self.ser.write((pwm_message + "\n").encode())
        except Exception as e:
            self.get_logger().error(f"Failed to send PWM over serial: {e}")

    # ---------- joystick callback ----------
    def joy_callback(self, msg: Joy):
        # direct mapping from joystick axes to desired body velocities
        # user can tune axis->velocity scaling externally (e.g. via joystick remapping node)
        self.desired_vx = msg.axes[1]
        self.desired_vy = msg.axes[0]
        self.desired_wz = msg.axes[3]

    def calculate_wheel_speeds(self, vx, vy, wz):
        # returns wheel rad/s [fl, fr, rl, rr]
        return [
            (vx - vy - wz * (self.L + self.W)) / self.R,
            (vx + vy + wz * (self.L + self.W)) / self.R,
            (vx + vy - wz * (self.L + self.W)) / self.R,
            (vx - vy + wz * (self.L + self.W)) / self.R
        ]


def main(args=None):
    rclpy.init(args=args)
    node = CombinedMecanumDeadWheelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down combined node")
        try:
            node.ser.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
