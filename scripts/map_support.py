#!/usr/bin/env python3
"""Provide a padded Nav2 map and a WSL-compatible SLAM map display."""

import copy
import json
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2, PointField

from map_grid import pad_grid, padded_origin, preview_points


class MapSupport(Node):
    def __init__(self):
        super().__init__("parking_map_support", parameter_overrides=[
            Parameter("use_sim_time", value=True)])
        root = Path(__file__).resolve().parents[1]
        config = json.loads((root/"config/patrol.json").read_text())
        self.padding = config.get("navigation_map_padding_m", 4.0)
        self.max_points = config.get("map_preview_max_points", 60000)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.map_pub = self.create_publisher(OccupancyGrid, "/navigation_map", latched)
        self.preview_pub = self.create_publisher(PointCloud2, "/map_preview", latched)
        self.create_subscription(OccupancyGrid, "/map", self.map_received, latched)
        self.get_logger().info(
            f"Navigation map adds {self.padding:.1f} m of unknown border. Raw SLAM /map is preserved.")

    def map_received(self, message):
        info = message.info
        q = info.origin.orientation
        angle = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        try:
            grid = pad_grid(info.width, info.height, info.resolution, message.data, self.padding)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return
        navigation = OccupancyGrid(header=message.header, info=copy.deepcopy(info), data=grid.data)
        navigation.info.width, navigation.info.height = grid.width, grid.height
        navigation.info.origin.position.x, navigation.info.origin.position.y = padded_origin(
            info.origin.position.x, info.origin.position.y, angle, grid.border, info.resolution)
        self.map_pub.publish(navigation)
        # Use a regular colored point cloud rather than RViz's indexed map
        # texture, which fails on the user's WSL graphics driver.
        data, count = preview_points(info.width, info.height, info.resolution, message.data,
                                    (info.origin.position.x, info.origin.position.y, angle),
                                    self.max_points)
        fields = [PointField(name=name, offset=offset, datatype=kind, count=1)
                  for name, offset, kind in (("x", 0, PointField.FLOAT32),
                                             ("y", 4, PointField.FLOAT32),
                                             ("z", 8, PointField.FLOAT32),
                                             ("rgb", 12, PointField.UINT32))]
        cloud = PointCloud2(header=message.header, height=1, width=count, fields=fields,
                            is_bigendian=False, point_step=16, row_step=16*count,
                            data=data, is_dense=True)
        self.preview_pub.publish(cloud)


def main():
    rclpy.init()
    node = MapSupport()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
