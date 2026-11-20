#!/usr/bin/env python3
"""Single final velocity publisher: readiness, stale-data and steering guards."""

import json
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Int8

from patrol_route import command_is_fresh, command_matches_direction, limit_command


class DriveGuard(Node):
    def __init__(self):
        super().__init__("parking_drive_guard", parameter_overrides=[
            Parameter("use_sim_time", value=True)])
        root = Path(__file__).resolve().parents[1]
        self.config = json.loads((root/"config/patrol.json").read_text())
        robot = json.loads((root/"config/limo.json").read_text())
        self.curvature = 0.98*math.sin(robot["steering_limit_rad"])/robot["wheelbase_m"]
        self.command = Twist()
        self.raw_time = self.scan_time = self.odom_time = self.enable_time = -math.inf
        self.allowed = False
        self.direction = 0
        self.direction_time = -math.inf
        self.last_direction_log = -math.inf
        self.output_v = 0.0
        self.previous = self.now()
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Twist, "/nav_cmd_vel", self.command_received, 10)
        self.create_subscription(LaserScan, "/scan", self.scan_received, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self.odom_received, 10)
        self.create_subscription(Bool, "/patrol/enabled", self.enable_received, 10)
        self.create_subscription(Int8, "/patrol/direction", self.direction_received, 10)
        self.create_timer(0.05, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds/1e9

    def command_received(self, message):
        self.command, self.raw_time = message, self.now()

    def scan_received(self, message):
        valid = sum(not math.isnan(v) and v >= message.range_min for v in message.ranges)
        stamp = message.header.stamp.sec+message.header.stamp.nanosec/1e9
        if message.ranges and valid/len(message.ranges) > 0.8 and abs(self.now()-stamp) < 0.6:
            self.scan_time = self.now()

    def odom_received(self, message):
        stamp = message.header.stamp.sec+message.header.stamp.nanosec/1e9
        if abs(self.now()-stamp) < 0.6:
            self.odom_time = self.now()

    def enable_received(self, message):
        self.allowed, self.enable_time = message.data, self.now()

    def direction_received(self, message):
        direction = message.data if message.data in (-1, 1) else 0
        if direction != self.direction:
            # Commands from before a phase change cannot authorize movement
            # in the new phase. Wait for a newly received Nav2 command.
            self.command = Twist()
            self.raw_time = -math.inf
            self.output_v = 0.0
            self.publisher.publish(Twist())
        self.direction, self.direction_time = direction, self.now()

    def tick(self):
        now = self.now()
        dt = max(0.0, min(0.1, now-self.previous))
        self.previous = now
        fresh = command_is_fresh(self.allowed, now, self.raw_time, self.scan_time,
                                 self.odom_time, self.enable_time)
        fresh = fresh and 0 <= now-self.direction_time < 2.0
        v, w = limit_command(self.command.linear.x, self.command.angular.z,
                             self.config["cruise_speed_m_s"],
                             self.config["reverse_speed_m_s"], self.curvature)
        matching = command_matches_direction(v, self.direction)
        if fresh and abs(v) >= 1e-4 and not matching and now-self.last_direction_log > 3.0:
            phase = "forward survey" if self.direction == 1 else "reverse return"
            self.get_logger().info(f"Stopped: navigation requested the wrong driving direction for {phase}.")
            self.last_direction_log = now
        if not fresh or not matching or abs(v) < 1e-4:
            self.output_v = 0.0
            self.publisher.publish(Twist())
            return
        # The Gazebo system also limits acceleration. Preserve curvature while
        # ramping so steering never exceeds the physical steering envelope.
        if v*self.output_v < 0:
            v = 0.0
        ramp = max(self.output_v-0.5*dt, min(self.output_v+0.5*dt, v))
        curvature = w/v if abs(v) > 1e-4 else 0.0
        self.output_v = ramp
        output = Twist()
        output.linear.x, output.angular.z = ramp, ramp*curvature
        self.publisher.publish(output)


def main():
    rclpy.init()
    node = DriveGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
