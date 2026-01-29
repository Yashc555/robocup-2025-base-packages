#!/usr/bin/env python3
"""
Publish a saved map (yaml + image produced by map_server) as nav_msgs/OccupancyGrid on /map.
Usage: python3 publish_map.py /path/to/map.yaml
"""
import sys
import yaml
import os
import numpy as np
from PIL import Image
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
import math
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

def load_map_from_yaml(yaml_path):
    with open(yaml_path, 'r') as f:
        info = yaml.safe_load(f)

    img_path = info['image']
    # expand user (~) and resolve relative paths relative to the YAML file
    img_path = os.path.expanduser(img_path)
    if not os.path.isabs(img_path):
        yaml_dir = os.path.dirname(os.path.abspath(yaml_path))
        img_path = os.path.join(yaml_dir, img_path)

    if not os.path.exists(img_path):
        raise FileNotFoundError(f"Map image not found: {img_path}\n"
                                f"Resolved from YAML 'image' entry: {info['image']}")

    resolution = float(info['resolution'])
    origin = info.get('origin', [0.0, 0.0, 0.0])
    negate = int(info.get('negate', 0))
    occ_thresh = float(info.get('occupied_thresh', 0.65))
    free_thresh = float(info.get('free_thresh', 0.196))
    img = Image.open(img_path).convert('L')
    arr = np.array(img)
    if negate:
        arr = 255 - arr
    norm = arr.astype(np.float32) / 255.0
    # classify: >occ -> 100, <free -> 0, else -1
    data = np.full(norm.shape, -1, dtype=np.int8)
    data[norm > occ_thresh] = 0
    data[norm < free_thresh] = 100
    # map_server uses origin at bottom-left; PGM rows top->bottom so flip vertically
    data = np.flipud(data)
    return data, resolution, origin
    return data, resolution, origin

class MapPublisher(Node):
    def __init__(self, grid, resolution, origin):
        super().__init__('test_map_publisher')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        self.pub = self.create_publisher(OccupancyGrid, '/map', qos_profile)
        self.grid = grid
        h, w = grid.shape
        self.meta = MapMetaData()
        self.meta.resolution = resolution
        self.meta.width = int(w)
        self.meta.height = int(h)
        # origin: [x, y, yaw]
        ox = float(origin[0])
        oy = float(origin[1])
        oyaw = float(origin[2]) if len(origin) > 2 else 0.0
        self.meta.origin.position.x = ox
        self.meta.origin.position.y = oy
        # set valid quaternion from yaw
        qz = math.sin(oyaw / 2.0)
        qw = math.cos(oyaw / 2.0)
        self.meta.origin.orientation.z = qz
        self.meta.origin.orientation.w = qw
        # origin[2] is yaw (orientation) - leave orientation.w=1 unless needed
        self.timer = self.create_timer(1.0, self.publish_map)
# ...existing code...

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info = self.meta
        msg.data = self.grid.flatten().astype(np.int8).tolist()
        self.pub.publish(msg)
        self.get_logger().info('Published saved map (/map)')

def main():
    if len(sys.argv) < 2:
        print("Usage: publish_map.py /path/to/map.yaml")
        return
    yaml_path = sys.argv[1]
    grid, res, origin = load_map_from_yaml(yaml_path)
    rclpy.init()

    node = MapPublisher(grid, res, origin)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()