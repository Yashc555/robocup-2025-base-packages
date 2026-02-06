# ...existing code...
#!/usr/bin/env python3
"""
Publish a fake BoundingBoxes msg to simulate an ML detection.

Behavior:
- If a /map (nav_msgs/OccupancyGrid) message is received, publish a box centered inside the map (frame_id='map').
- Otherwise publish a box in a camera frame (frame_id='camera_link') using the original example coordinates.

Run after sourcing your workspace:
python3 publish_box_coordinate.py
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from base_custom_interfaces.msg import BoundingBoxes, BoundingBox


class FakeBoxesPublisher(Node):
    def __init__(self):
        super().__init__('fake_boxes_publisher')
        self.pub = self.create_publisher(BoundingBoxes, '/ml_tape_detections', 10)
        self.map_msg = None
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_cb, 1)
        self.timer = self.create_timer(1.0, self.publish_box)
        self.get_logger().info('FakeBoxesPublisher started. Waiting for /map (optional).')

    def map_cb(self, msg: OccupancyGrid):
        # store latest map for use when publishing boxes in map frame
        self.map_msg = msg
        # log map info once
        if not hasattr(self, '_map_logged'):
            info = msg.info
            min_x = info.origin.position.x
            min_y = info.origin.position.y
            max_x = min_x + info.width * info.resolution
            max_y = min_y + info.height * info.resolution
            self.get_logger().info(f"Got map: x[{min_x:.2f},{max_x:.2f}] y[{min_y:.2f},{max_y:.2f}] res={info.resolution:.3f} size={info.width}x{info.height}")
            self._map_logged = True
    def publish_box(self):
        bb = BoundingBox()

        # Force publish in camera frame so TF->map is used (ignore self.map_msg)
        bb.frame_id = 'camera_link'
        # forward (x) in meters from camera, left/right (y), z height
        bb.x1, bb.y1, bb.z1 = 2.75, -0.25, 1.2
        bb.x2, bb.y2, bb.z2 = 2.75,  0.25, 1.2
        bb.x3, bb.y3, bb.z3 = 3.75,  0.25, 1.2
        bb.x4, bb.y4, bb.z4 = 3.75, -0.25, 1.2
        self.get_logger().info('Published BoundingBoxes in camera_link frame (forced)')

        msg = BoundingBoxes()
        msg.boxes = [bb]
        self.pub.publish(msg)
    # def publish_box(self):
    #     bb = BoundingBox()

    #     # Larger box for better visibility on the occupancy grid
    #     if self.map_msg is None:
    #         # fallback: publish in camera frame (bigger, more visible)
    #         bb.frame_id = 'camera_link'
    #         bb.x1, bb.y1, bb.z1 = 15, -0.25, 1.2
    #         bb.x2, bb.y2, bb.z2 = 1.5,  0.25, 1.2
    #         bb.x3, bb.y3, bb.z3 = 2.5,  0.25, 1.2
    #         bb.x4, bb.y4, bb.z4 = 2.5, -0.25, 1.2
    #         self.get_logger().info('Published enlarged fake BoundingBoxes in camera_link frame (map not available)')
    #     else:
    #         # place larger box at center of map (safe inside bounds) so occupancy update is visible
    #         info = self.map_msg.info
    #         center_x = info.origin.position.x + (info.width * info.resolution) / 2.0
    #         center_y = info.origin.position.y + (info.height * info.resolution) / 2.0

    #         # make the test box scale with map resolution so it's visible:
    #         # length = max(1.0 m, 10 * resolution) and half width = max(0.2 m, 5 * resolution)
    #         length = max(1.0, 10 * info.resolution)
    #         half_w = max(0.2, 5 * info.resolution)

    #         x1 = center_x - length / 2.0
    #         x2 = center_x + length / 2.0
    #         y1 = center_y - half_w
    #         y2 = center_y + half_w
    #         bb.frame_id = 'map'
    #         bb.x1, bb.y1, bb.z1 = x1, y1, 0.0
    #         bb.x2, bb.y2, bb.z2 = x1, y2, 0.0
    #         bb.x3, bb.y3, bb.z3 = x2, y2, 0.0
    #         bb.x4, bb.y4, bb.z4 = x2, y1, 0.0
    #         self.get_logger().info(f'Published enlarged fake BoundingBoxes in map frame at ~({center_x:.2f},{center_y:.2f}) size ({length:.2f}x{half_w*2:.2f} m)')

    #     msg = BoundingBoxes()
    #     msg.boxes = [bb]
    #     self.pub.publish(msg)


def main():
    rclpy.init()
    node = FakeBoxesPublisher()
    print("fake_boxes_publisher: started")
    node.get_logger().info("fake_boxes_publisher: spinning")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
# ...existing code...