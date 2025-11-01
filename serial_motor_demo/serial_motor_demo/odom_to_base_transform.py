import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class OdomToBaseTransform(Node):
    def __init__(self):
        super().__init__('odom_to_base_transform')
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

    def odom_callback(self, msg):
        transform = TransformStamped()

        # Header
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_footprint'

        # Translation from odometry
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z

        # Rotation from odometry
        transform.transform.rotation = msg.pose.pose.orientation

        # Publish the transform
        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    node = OdomToBaseTransform()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
