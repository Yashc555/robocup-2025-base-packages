#!/usr/bin/env python3
"""
tape_map_modifier_node.py

Listens to:
  - /map (nav_msgs/OccupancyGrid)
  - /ml_tape_detections (custom_interfaces/msg/BoundingBoxes)
and permanently marks detected tape regions as occupied (100) in the map.

Publishes:
  - /map_with_tapes (nav_msgs/OccupancyGrid)

Clear with:
  ros2 service call /clear_tape_obstacles std_srvs/srv/Empty
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
from rclpy.duration import Duration

from custom_interfaces.msg import BoundingBoxes


class TapeMapModifier(Node):
    def __init__(self):
        super().__init__('tape_map_modifier')

        # Parameters
        self.declare_parameter('boxes_topic', '/ml_tape_detections')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('output_topic', '/map_with_tapes')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('tf_timeout_sec', 0.25)
        # If False: use only x1/x2 (bottom edge) and create a thin rectangle around it
        self.declare_parameter('use_all_corners', True)
        # half-width (m) when using only 2 corners to form a thin rectangle
        self.declare_parameter('tape_half_width', 0.05)
        # self.declare_parameter('use_sim_time', True)

        self.boxes_topic = self.get_parameter('boxes_topic').value
        self.map_topic = self.get_parameter('map_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.tf_timeout = float(self.get_parameter('tf_timeout_sec').value)
        self.use_all_corners = bool(self.get_parameter('use_all_corners').value)
        self.tape_half_width = float(self.get_parameter('tape_half_width').value)

        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Map storage
        self.map_msg = None
        self.map_lock = False
        self.tape_cells = set()  # persistent occupied cells (i,j)

        # Subscribers
        self.create_subscription(OccupancyGrid, self.map_topic, self._map_cb, 10)
        self.create_subscription(BoundingBoxes, self.boxes_topic, self._boxes_cb, 10)

        # Publisher
        self.map_pub = self.create_publisher(OccupancyGrid, self.output_topic, 10)

        # Clear service
        self.create_service(Empty, 'clear_tape_obstacles', self._clear_srv)

        self.get_logger().info(f"TapeMapModifier active. Listening to {self.boxes_topic} and {self.map_topic}")

        # Timer to republish updated map
        self.create_timer(1.0, self._publish_modified_map)

    def _clear_srv(self, request, response):
        self.tape_cells.clear()
        self.get_logger().info("Cleared all tape obstacles from map overlay.")
        return response

    

    def _boxes_cb(self, msg: BoundingBoxes):
        if self.map_msg is None:
            self.get_logger().warn("No map received yet.")
            return

        if not msg.boxes:
            return

        self.get_logger().info(f"Received {len(msg.boxes)} tape boxes.")

        for box in msg.boxes:
            # Build list of points (in box frame)
            if self.use_all_corners:
                pts_cam = [
                    (box.x1, box.y1, box.z1),
                    (box.x2, box.y2, box.z2),
                    (box.x3, box.y3, box.z3),
                    (box.x4, box.y4, box.z4),
                ]
            else:
                # Use only bottom edge (x1,x2) and create a thin rect around it
                x1, y1, z1 = box.x1, box.y1, box.z1
                x2, y2, z2 = box.x2, box.y2, box.z2
                dx = x2 - x1
                dy = y2 - y1
                length = math.hypot(dx, dy) + 1e-12
                nx = -dy / length
                ny = dx / length
                hw = self.tape_half_width
                # four corners in camera frame
                pts_cam = [
                    (x1 + nx * hw, y1 + ny * hw, z1),
                    (x2 + nx * hw, y2 + ny * hw, z2),
                    (x2 - nx * hw, y2 - ny * hw, z2),
                    (x1 - nx * hw, y1 - ny * hw, z1),
                ]

            # transform corners to map frame
            map_pts = []
            for (x, y, z) in pts_cam:
                p = PointStamped()
                p.header.frame_id = box.frame_id
                p.header.stamp = self.get_clock().now().to_msg()
                p.point.x = float(x)
                p.point.y = float(y)
                p.point.z = float(z)
                try:
                    p_map = self.tf_buffer.transform(p, self.map_frame, timeout=Duration(seconds=self.tf_timeout))
                    map_pts.append((p_map.point.x, p_map.point.y))
                except Exception as e:
                    # If TF fails, warn and skip this box
                    self.get_logger().warn(f"TF transform failed for box (frame={box.frame_id}): {e}")
                    map_pts = []
                    break

            if len(map_pts) != 4:
                continue

            # Rasterize polygon in map and add cells
            self.get_logger().info(
                f"Drawing polygon (map coords) {[(round(px,3), round(py,3)) for px,py in map_pts]}"
            )
            self._mark_tape_polygon(map_pts)

    def _map_cb(self, msg: OccupancyGrid):
        # store a reference to latest map (we modify a copy when publishing)
        if self.map_lock:
            return
        self.map_msg = msg
        # helpful debug once in a while
        if hasattr(self, '_map_debug_count'):
            self._map_debug_count += 1
        else:
            self._map_debug_count = 1
        if self._map_debug_count % 10 == 1:  # every ~10 messages
            info = msg.info
            self.get_logger().info(
                f"Map info: origin=({info.origin.position.x:.3f},{info.origin.position.y:.3f}), "
                f"res={info.resolution:.4f}, size=({info.width}x{info.height}), "
                f"origin.orientation={info.origin.orientation}"
            )

    def _mark_tape_polygon(self, poly_xy):
        """
        Rasterize polygon poly_xy (list of (x,y) in map frame) and add grid cells to self.tape_cells.
        Vectorized point-in-polygon scan within the polygon's bounding box.
        """
        if self.map_msg is None:
            self.get_logger().warn("No map to draw on (map_msg is None)")
            return

        info = self.map_msg.info
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y
        res = info.resolution
        width = info.width
        height = info.height

        # Polygon bounding box in world coords
        xs = [float(p[0]) for p in poly_xy]
        ys = [float(p[1]) for p in poly_xy]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)

        # Convert bbox to cell indices (clamped)
        i_min = int(math.floor((min_x - origin_x) / res))
        i_max = int(math.ceil((max_x - origin_x) / res))
        j_min = int(math.floor((min_y - origin_y) / res))
        j_max = int(math.ceil((max_y - origin_y) / res))

        self.get_logger().info(
            f"Poly bounds world: x[{min_x:.3f},{max_x:.3f}] y[{min_y:.3f},{max_y:.3f}] -> "
            f"cells i[{i_min},{i_max}] j[{j_min},{j_max}] (map size {width}x{height})"
        )

        # quick outside check
        if i_max < 0 or j_max < 0 or i_min >= width or j_min >= height:
            self.get_logger().warn("Polygon outside map bounds — skipping draw")
            return

        # clamp
        i_min = max(0, i_min)
        i_max = min(width - 1, i_max)
        j_min = max(0, j_min)
        j_max = min(height - 1, j_max)

        # Create grid of cell centers
        i_vals = np.arange(i_min, i_max + 1)
        j_vals = np.arange(j_min, j_max + 1)
        if i_vals.size == 0 or j_vals.size == 0:
            self.get_logger().warn("After clamping there are no cells to check")
            return

        xs_centers = origin_x + (i_vals + 0.5) * res
        ys_centers = origin_y + (j_vals + 0.5) * res

        XX, YY = np.meshgrid(xs_centers, ys_centers)
        pts_x = XX.ravel()
        pts_y = YY.ravel()

        poly = np.array(poly_xy, dtype=float)
        px = poly[:, 0]
        py = poly[:, 1]
        n = len(px)
        inside = np.zeros(pts_x.shape, dtype=bool)

        for k in range(n):
            xi = px[k]; yi = py[k]
            xj = px[(k + 1) % n]; yj = py[(k + 1) % n]
            cond = ((yi > pts_y) != (yj > pts_y)) & (
                pts_x < (xj - xi) * (pts_y - yi) / (yj - yi + 1e-12) + xi
            )
            inside ^= cond

        inside_idx = np.nonzero(inside)[0]
        if inside_idx.size == 0:
            self.get_logger().info("No cells fell inside polygon bounding box")
            return

        # convert linear indices back to grid i,j and add to tape_cells
        Ni = i_vals.size
        rows = inside_idx // Ni
        cols = inside_idx % Ni
        added = 0
        sample = []
        for r, c in zip(rows, cols):
            j = int(j_vals[r])
            i = int(i_vals[c])
            if 0 <= i < width and 0 <= j < height:
                self.tape_cells.add((i, j))
                added += 1
                if len(sample) < 8:
                    sample.append((i, j))
        self.get_logger().info(f"Marked {added} cells inside polygon. sample indices: {sample}")

    def _publish_modified_map(self):
        if self.map_msg is None:
            return

        self.map_lock = True
        modified_map = OccupancyGrid()
        modified_map.header = self.map_msg.header
        modified_map.info = self.map_msg.info
        data = np.array(self.map_msg.data, dtype=np.int8).reshape(
            (self.map_msg.info.height, self.map_msg.info.width)
        )

        # Mark tape cells as occupied (100)
        for (i, j) in self.tape_cells:
            if 0 <= i < self.map_msg.info.width and 0 <= j < self.map_msg.info.height:
                data[j, i] = 100

        modified_map.data = np.array(data, dtype=np.int8).flatten().tolist()
        modified_map.header.stamp = self.get_clock().now().to_msg()
        self.map_pub.publish(modified_map)
        self.map_lock = False


def main(args=None):
    rclpy.init(args=args)
    node = TapeMapModifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
