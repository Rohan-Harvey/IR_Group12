#!/usr/bin/env python3
"""Read live navigation data into results/navigation_diagnostic.json; never drive."""

import json
import math
from pathlib import Path
import time


def scan_summary(message, robot):
    valid = [r for r in message.ranges if math.isfinite(r) and message.range_min <= r <= message.range_max]
    forward = [r for i, r in enumerate(message.ranges)
               if abs(message.angle_min+i*message.angle_increment) < math.radians(30)
               and math.isfinite(r) and message.range_min <= r <= message.range_max]
    inside = 0
    scale = robot["uniform_scale_from_official_xacro"]
    lidar_x = 0.1236*scale
    length, width = robot["footprint_size_m"]
    cx = robot["footprint_center_offset_m"][0]
    for i, distance in enumerate(message.ranges):
        if not math.isfinite(distance) or distance < message.range_min:
            continue
        angle = message.angle_min+i*message.angle_increment
        x, y = lidar_x+distance*math.cos(angle), distance*math.sin(angle)
        inside += int(abs(x-cx) < length/2 and abs(y) < width/2)
    return {"frame": message.header.frame_id, "rays": len(message.ranges),
            "finite_valid_returns": len(valid), "nearest_return_m": min(valid) if valid else None,
            "forward_30deg_nearest_m": min(forward) if forward else None,
            "returns_inside_body_rectangle": inside}


def main():
    # Keep ROS imports inside main so summary helpers remain testable without ROS.
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
    from rclpy.time import Time
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import OccupancyGrid, Odometry
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String, Int8
    from tf2_ros import Buffer, TransformListener

    root = Path(__file__).resolve().parents[1]
    robot = json.loads((root/"config/limo.json").read_text())
    rclpy.init()
    node = Node("parking_navigation_diagnostic", parameter_overrides=[Parameter("use_sim_time", value=True)])
    samples = {}
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
    subscriptions = []
    map_topics = ("/map", "/navigation_map", "/global_costmap/costmap", "/local_costmap/costmap")
    for topic in map_topics:
        subscriptions.append(node.create_subscription(
            OccupancyGrid, topic, lambda msg, key=topic: samples.__setitem__(key, msg), latched))
    for kind, topic, qos in ((LaserScan, "/scan", qos_profile_sensor_data),
                            (Odometry, "/odom", 10), (Twist, "/nav_cmd_vel", 10),
                            (Twist, "/cmd_vel", 10), (String, "/patrol/status", latched),
                            (Int8, "/patrol/direction", 10)):
        subscriptions.append(node.create_subscription(
            kind, topic, lambda msg, key=topic: samples.__setitem__(key, msg), qos))
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    print("Reading live sensors and maps for eight seconds. No motion commands will be sent.")
    try:
        deadline = time.monotonic()+8.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        report = {"received_topics": sorted(samples), "maps": {}}
        for topic in map_topics:
            if topic not in samples:
                continue
            message = samples[topic]
            origin = message.info.origin
            report["maps"][topic] = {
                "frame": message.header.frame_id, "width": message.info.width,
                "height": message.info.height, "resolution": message.info.resolution,
                "origin_xyz": [origin.position.x, origin.position.y, origin.position.z],
                "origin_xyzw": [origin.orientation.x, origin.orientation.y, origin.orientation.z, origin.orientation.w],
                "data": list(message.data)}
        if "/scan" in samples:
            scan = samples["/scan"]
            report["scan_summary"] = scan_summary(scan, robot)
            report["scan"] = {"frame": scan.header.frame_id, "angle_min": scan.angle_min,
                              "angle_increment": scan.angle_increment,
                              "ranges": [float(r) if math.isfinite(r) else None for r in scan.ranges]}
        for topic in ("/nav_cmd_vel", "/cmd_vel"):
            if topic in samples:
                report[topic] = {"linear_x": samples[topic].linear.x, "angular_z": samples[topic].angular.z}
        if "/odom" in samples:
            report["odom_linear_x"] = samples["/odom"].twist.twist.linear.x
        for topic in ("/patrol/status", "/patrol/direction"):
            if topic in samples:
                report[topic] = samples[topic].data
        try:
            transform = buffer.lookup_transform("map", "rear_axle", Time()).transform
            report["rear_axle_in_map"] = {
                "xyz": [transform.translation.x, transform.translation.y, transform.translation.z],
                "xyzw": [transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w]}
        except Exception as exc:
            report["transform_error"] = str(exc)
        output = root/"results/navigation_diagnostic.json"
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, allow_nan=False), encoding="utf-8")
        brief = {k: v for k, v in report.items() if k not in ("maps", "scan")}
        print(json.dumps(brief, indent=2, ensure_ascii=False))
        print(f"Full diagnostic saved to {output}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
