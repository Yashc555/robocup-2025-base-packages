import rclpy
from rclpy.node import Node
import serial
import json
import math
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker  # Import for RViz visualization

class IMUYawProcessor(Node):
    def __init__(self):
        super().__init__('imu_yaw_processor')

        # Open Serial Port
        try:
            self.serial_port = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        except serial.SerialException as e:
            self.get_logger().error(f"Serial port error: {e}")
            return
        
        # Create ROS2 publishers
        self.imu_publisher = self.create_publisher(Imu, 'imu/data', 10)
        self.marker_publisher = self.create_publisher(Marker, 'imu_marker', 10)  # For cube visualization

        # TF broadcaster for IMU frame
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer to process serial data
        self.timer = self.create_timer(0.01, self.process_serial_data)  # Read every 10ms

    def process_serial_data(self):
        if self.serial_port.in_waiting > 0:
            raw_data = self.serial_port.readline().decode().strip()

            try:
                # Parse JSON from serial data
                imu_data = json.loads(raw_data)

                # Extract quaternion components
                q = Quaternion(
                    w=imu_data.get("w", 1.0),
                    x=imu_data.get("x", 0.0),
                    y=imu_data.get("y", 0.0),
                    z=imu_data.get("z", 0.0)
                )

                # Compute yaw from quaternion (converted to degrees)
                computed_yaw = -math.degrees(self.quaternion_to_yaw(q))  # Negating to match IMU convention
                imu_yaw = imu_data.get("yaw", 0.0)  # IMU yaw is already in degrees

                # Ensure the difference is in the [-180, 180] range
                yaw_difference = (computed_yaw - imu_yaw + 180) % 360 - 180  

                # Publish IMU message
                imu_msg = Imu()
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.header.frame_id = "imu_link"
                imu_msg.orientation = q  # Send quaternion directly

                self.imu_publisher.publish(imu_msg)

                # Publish TF transform (for visualization in RViz)
                self.publish_tf(q)

                # Publish cube marker for visualization
                self.publish_marker(q)

                # Print results
                print(f"Computed Yaw: {computed_yaw:.2f}°")
                print(f"IMU Yaw: {imu_yaw:.2f}°")
                print(f"Difference: {yaw_difference:.2f}°\n")

            except json.JSONDecodeError:
                self.get_logger().error(f"Invalid JSON received: {raw_data}")
            except Exception as e:
                self.get_logger().error(f"Error processing data: {e}")

    def quaternion_to_yaw(self, q):
        """ Extract yaw from quaternion (assuming ZYX Euler convention). """
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)  # Returns in radians

    def publish_tf(self, q):
        """ Publishes the IMU orientation as a TF transform. """
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"  # IMU is attached to base_link
        t.child_frame_id = "imu_link"
        t.transform.rotation = q

        self.tf_broadcaster.sendTransform(t)

    def publish_marker(self, q):
        """ Publishes a small cube marker for IMU visualization. """
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = "imu_link"  # Attach marker to IMU frame
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        # Cube size (small)
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        # Color (red)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Fully visible

        # Orientation (matches IMU rotation)
        marker.pose.orientation = q

        # Position (keep at origin of IMU frame)
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        self.marker_publisher.publish(marker)

def main():
    rclpy.init()
    node = IMUYawProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()