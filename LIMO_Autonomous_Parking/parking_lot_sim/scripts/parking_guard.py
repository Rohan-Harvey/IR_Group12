#!/usr/bin/env python3
"""Final motion authorization, independent of the planner and path follower."""

import json
import math
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from parking_core import Footprint, MovingReturns, can_stop, inverse, scan_rays
from parking_ros import SCAN_TOPICS, from_pose, scan_pose, spin_until_stopped, stamp_seconds, twist
from patrol_route import limit_command


class ParkingGuard(Node):
    def __init__(self):
        super().__init__("parking_motion_guard", parameter_overrides=[Parameter("use_sim_time", value=True)])
        config = Path(__file__).resolve().parents[1]/"config"
        self.settings = json.loads((config/"parking.json").read_text())
        self.forward_limit = max(self.settings[key] for key in
                                 ("survey_speed_m_s", "mapping_speed_m_s", "inspection_transit_speed_m_s"))
        robot = json.loads((config/"limo.json").read_text())
        self.footprint = Footprint.from_robot(robot)
        self.curvature = 0.98*math.sin(robot["steering_limit_rad"])/robot["wheelbase_m"]
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.raw = self.odom = None
        self.raw_time = self.previous_time = -math.inf
        self.issued, self.issued_curve = 0.0, 0.0
        self.pending_scans, self.scans = {}, {}
        self.moving_returns = MovingReturns()
        self.contact = False
        self.status = None
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.status_pub = self.create_publisher(String, "/parking/guard",
                                                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Twist, "/parking/requested_cmd", self.on_command, 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Contacts, "/bumper_contacts", self.on_contact, qos_profile_sensor_data)
        for topic in SCAN_TOPICS:
            self.create_subscription(LaserScan, topic, lambda msg, t=topic: self.on_scan(t, msg),
                                     qos_profile_sensor_data)
        self.create_timer(0.05, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds*1e-9

    def on_odom(self, msg):
        self.odom = msg

    def on_command(self, msg):
        self.raw, self.raw_time = msg, self.now()

    def on_contact(self, msg):
        if msg.contacts:
            self.contact = True
            self.stop("chassis_contact")

    def on_scan(self, topic, msg):
        if msg.ranges and sum(n >= msg.range_min for n in msg.ranges) >= 0.65*len(msg.ranges):
            self.pending_scans[topic] = msg

    def scan_points(self, now):
        for topic, scan in list(self.pending_scans.items()):
            age = now-stamp_seconds(scan.header.stamp)
            if age > 0.4:
                del self.pending_scans[topic]
                continue
            pose = scan_pose(self.buffer, scan)
            if pose is not None:
                points = [(x, y) for x, y, hit in scan_rays(scan, pose, 10.0) if hit]
                hits = np.asarray(points, dtype=float).reshape(-1, 2)
                self.scans[topic] = (now-age, hits)
                if topic == SCAN_TOPICS[0]:
                    self.moving_returns.update(hits, now-age)
                del self.pending_scans[topic]
        if not all(topic in self.scans and 0 <= now-self.scans[topic][0] <= 0.42
                   for topic in SCAN_TOPICS):
            return None
        odom = from_pose(self.odom.pose.pose)
        x, y, angle = inverse((odom.x, odom.y, odom.yaw))
        c, s = math.cos(angle), math.sin(angle)
        points = np.concatenate([self.scans[topic][1] for topic in SCAN_TOPICS])
        rotation = np.array(((c, s), (-s, c)))
        offset = np.array((x, y))
        moving = self.moving_returns.active(now)
        if moving is not None:
            moving[:, 0] = moving[:, 0] @ rotation + offset
            moving[:, 1] = moving[:, 1] @ rotation
        return points @ rotation+offset, moving

    def report(self, reason):
        if reason != self.status:
            self.status = reason
            self.status_pub.publish(String(data=reason))
            if reason in ("braking_clearance", "chassis_contact"):
                self.get_logger().warning(f"Motion stopped: {reason}")

    def stop(self, reason):
        self.issued = 0.0
        self.publisher.publish(twist())
        self.report(reason)

    def tick(self):
        now = self.now()
        dt = max(0, min(0.1, now-self.previous_time))
        self.previous_time = now
        if self.contact:
            self.stop("chassis_contact")
            return
        if (self.raw is None or self.odom is None or not 0 <= now-self.raw_time < 0.32
                or not 0 <= now-stamp_seconds(self.odom.header.stamp) < 0.32):
            self.stop("waiting_for_command_or_odometry")
            return
        readings = self.scan_points(now)
        if readings is None:
            self.stop("waiting_for_current_scan_transforms")
            return
        points, moving = readings
        v, w = limit_command(self.raw.linear.x, self.raw.angular.z, self.forward_limit,
                             self.settings["survey_return_speed_m_s"], self.curvature)
        if not v:
            self.stop("commanded_stop")
            return
        actual = self.odom.twist.twist
        speed = actual.linear.x
        if not math.isfinite(speed) or not math.isfinite(actual.angular.z):
            self.stop("invalid_odometry")
            return
        if speed*v < 0 and abs(speed) > 0.035:
            self.stop("stopping_before_reverse")
            return
        requested_curve = w/v
        measured_curve = actual.angular.z/speed if abs(speed) > 0.08 else self.issued_curve
        measured_curve = max(-self.curvature, min(self.curvature, measured_curve))
        curves = (requested_curve, measured_curve, self.issued_curve, 0.0)
        # Test prospective motion even from standstill, without forcing a
        # vehicle travelling slowly to reserve the full distance at cruise speed.
        candidate = math.copysign(min(abs(v), abs(self.issued)+0.42*dt), v)
        speed_to_test = math.copysign(max(abs(candidate), abs(speed)), v)
        if not can_stop(points, self.footprint, speed_to_test, curves,
                        self.settings["braking_m_s2"], self.settings["reaction_time_s"],
                        self.settings["runtime_clearance_m"], self.settings["pedestrian_expansion_m"], moving):
            self.stop("braking_clearance")
            return
        self.publisher.publish(twist(candidate, candidate*requested_curve))
        self.issued, self.issued_curve = candidate, requested_curve
        self.report("motion_clear")


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = ParkingGuard()
    try:
        spin_until_stopped(node)
    finally:
        node.publisher.publish(twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
