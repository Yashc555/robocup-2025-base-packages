import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.duration import Duration

class ScanTimestampFixer(Node):
    def __init__(self):
        super().__init__('scan_timestamp_fixer')
        self.sub = self.create_subscription(LaserScan, '/scan_raw_filtered', self.scan_callback, 1000)
        self.pub = self.create_publisher(LaserScan, '/scan', 1000)
        self.delay = Duration(seconds=0.05)  # Delay to avoid extrapolation

    def scan_callback(self, msg: LaserScan):
        # Subtract delay from current time to avoid extrapolation
        corrected_time = self.get_clock().now() - self.delay
        msg.header.stamp = corrected_time.to_msg()
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ScanTimestampFixer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
