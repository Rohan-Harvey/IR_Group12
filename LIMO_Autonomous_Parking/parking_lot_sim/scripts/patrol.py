#!/usr/bin/env python3
"""Send rolling survey goals to Nav2 while SLAM builds the map."""

import json
import math
from pathlib import Path
import subprocess
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
from std_msgs.msg import Bool, Int8, String
from rcl_interfaces.srv import SetParameters
from tf2_ros import Buffer, TransformListener, TransformException

from patrol_route import (build_route, goal_index, nearest_progress, rear_point,
                          route_lengths, transform, wrap)


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


class Patrol(Node):
    def __init__(self):
        super().__init__("parking_patrol", parameter_overrides=[
            Parameter("use_sim_time", value=True)])
        self.root = Path(__file__).resolve().parents[1]
        self.config = json.loads((self.root/"config/patrol.json").read_text())
        self.robot = json.loads((self.root/"config/limo.json").read_text())
        self.legs = build_route(self.config, self.robot["wheelbase_m"])
        self.lengths = [route_lengths(points) for _, points in self.legs]
        self.home = None
        self.map = None
        self.map_time = -math.inf
        self.leg = self.progress = 0
        self.last_target = -1
        self.last_send = self.hold_until = self.last_log = -math.inf
        # Node.handle belongs to rclpy; keep the action goal separately.
        self.goal_handle = None
        self.sending = False
        self.generation = 0
        self.active = False
        self.speed = 0.0
        self.finished = False
        self.saver = None
        self.save_reported = False
        self.save_at = math.inf
        self.save_started_wall = None
        self.mode_direction = 0
        self.mode_pending = None
        self.mode_requested = 0
        self.mode_started = -math.inf
        self.mode_error = None
        self.planner_parameters = self.create_client(SetParameters, "/planner_server/set_parameters")
        self.controller_parameters = self.create_client(SetParameters, "/controller_server/set_parameters")
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.route_pub = self.create_publisher(RosPath, "/patrol/route", latched)
        self.enabled_pub = self.create_publisher(Bool, "/patrol/enabled", 10)
        self.direction_pub = self.create_publisher(Int8, "/patrol/direction", 10)
        self.status_pub = self.create_publisher(String, "/patrol/status", latched)
        self.create_subscription(OccupancyGrid, "/navigation_map", self.map_received, latched)
        self.create_subscription(Odometry, "/odom", self.odom_received, 10)
        self.create_timer(0.5, self.tick)
        self.report("Waiting for SLAM map, transforms and Nav2. The robot remains stopped.")

    def now(self):
        return self.get_clock().now().nanoseconds/1e9

    def report(self, text):
        self.get_logger().info(text)
        self.status_pub.publish(String(data=text))

    def enable(self, enabled):
        direction = self.legs[self.leg][1][0].direction if enabled and self.leg < len(self.legs) else 0
        enabled = bool(enabled and direction in (-1, 1) and self.mode_direction == direction
                       and self.mode_error is None)
        if not enabled:
            direction = 0
        self.direction_pub.publish(Int8(data=direction))
        self.enabled_pub.publish(Bool(data=enabled))

    def ensure_direction(self, direction):
        """Change planner/controller mode only while stopped; require both acks.

        One Smac instance is used because its motion-table assets are static.
        Jazzy supports changing motion_model_for_search via its parameter API.
        """
        if self.mode_error is not None:
            self.wait(self.mode_error)
            return False
        if self.mode_direction == direction:
            return True
        self.enable(False)
        if self.speed > 0.05:
            return False
        if self.mode_pending is None:
            if not (self.planner_parameters.service_is_ready()
                    and self.controller_parameters.service_is_ready()):
                self.wait("Waiting for navigation parameter services before authorizing a driving direction.")
                return False
            model = "DUBIN" if direction == 1 else "REEDS_SHEPP"
            planner = SetParameters.Request(parameters=[
                Parameter("GridBased.motion_model_for_search", value=model).to_parameter_msg()])
            controller = SetParameters.Request(parameters=[
                Parameter("FollowPath.allow_reversing", value=(direction == -1)).to_parameter_msg()])
            self.mode_requested, self.mode_started = direction, self.now()
            self.mode_pending = (
                self.planner_parameters.call_async(planner),
                self.controller_parameters.call_async(controller))
            self.report("Configuring forward-only survey navigation." if direction == 1
                        else "Stopped. Configuring navigation for the reverse return.")
            return False
        if self.now()-self.mode_started > 10.0:
            # Do not retry unknown-outcome mutations: late replies could apply
            # out of order. A timeout leaves driving disabled until restart.
            self.mode_error = "Navigation direction setup timed out. Driving is disabled; restart the mission."
            self.report(self.mode_error)
            return False
        if not all(future.done() for future in self.mode_pending):
            return False
        try:
            for future in self.mode_pending:
                results = future.result().results
                if len(results) != 1 or not results[0].successful:
                    reason = results[0].reason if results else "empty parameter reply"
                    raise RuntimeError(reason)
            if self.mode_requested != direction:
                raise RuntimeError("mission direction changed during setup")
        except Exception as exc:
            self.mode_error = f"Navigation direction setup failed: {exc}. Driving is disabled."
            self.report(self.mode_error)
            return False
        self.mode_direction, self.mode_pending = direction, None
        self.report("Forward-only survey navigation confirmed." if direction == 1
                    else "Reverse-return navigation confirmed.")
        return True

    def wait(self, message):
        self.enable(False)
        if self.now()-self.last_log >= 5.0:
            self.report(message)
            self.last_log = self.now()

    def cancel_navigation(self):
        # Invalidate pending replies too: a late accepted goal must be cancelled.
        self.generation += 1
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.goal_handle = None
        self.active = self.sending = False
        self.last_target = -1
        self.enable(False)

    def map_received(self, message):
        self.map, self.map_time = message, self.now()

    def odom_received(self, message):
        self.speed = abs(message.twist.twist.linear.x)

    def robot_pose(self):
        t = self.buffer.lookup_transform("map", "base_footprint", Time())
        stamp = t.header.stamp.sec+t.header.stamp.nanosec/1e9
        if abs(self.now()-stamp) > 0.6:
            raise RuntimeError("Robot transform is stale")
        return t.transform.translation.x, t.transform.translation.y, yaw(t.transform.rotation)

    def in_map(self, point):
        origin = self.map.info.origin
        angle = yaw(origin.orientation)
        cx = self.robot["footprint_center_offset_m"][0]+self.robot["wheelbase_m"]/2
        half_x = self.robot["footprint_size_m"][0]/2+0.12
        half_y = self.robot["steered_envelope_width_m"]/2+0.12
        c, s = math.cos(point.yaw), math.sin(point.yaw)
        for x_offset in (cx-half_x, cx+half_x):
            for y_offset in (-half_y, half_y):
                dx = point.x+c*x_offset-s*y_offset-origin.position.x
                dy = point.y+s*x_offset+c*y_offset-origin.position.y
                x = math.cos(angle)*dx+math.sin(angle)*dy
                y = -math.sin(angle)*dx+math.cos(angle)*dy
                if not (0.1 < x < self.map.info.width*self.map.info.resolution-0.1
                        and 0.1 < y < self.map.info.height*self.map.info.resolution-0.1):
                    return False
        return True

    def message_pose(self, point):
        msg = PoseStamped()
        msg.header.frame_id, msg.header.stamp = "map", self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = point.x, point.y
        msg.pose.orientation.z, msg.pose.orientation.w = math.sin(point.yaw/2), math.cos(point.yaw/2)
        return msg

    def publish_route(self):
        msg = RosPath()
        msg.header.frame_id, msg.header.stamp = "map", self.get_clock().now().to_msg()
        for _, points in self.legs:
            msg.poses.extend(self.message_pose(transform(p, self.home)) for p in points[::3])
        self.route_pub.publish(msg)

    def send_target(self, target):
        self.sending, self.generation = True, self.generation+1
        serial = self.generation
        goal = NavigateToPose.Goal()
        goal.pose = self.message_pose(target)
        goal.behavior_tree = str(self.root/"config/navigate.xml")
        future = self.client.send_goal_async(goal)
        future.add_done_callback(lambda f: self.goal_response(f, serial))
        self.last_send = self.now()

    def goal_response(self, future, serial):
        try:
            handle = future.result()
            if serial != self.generation:
                if handle.accepted:
                    handle.cancel_goal_async()
                return
            self.sending = False
            if not handle.accepted:
                self.cancel_navigation()
                self.hold_until = self.now()+2.0
                self.report("Nav2 rejected the target; stopped and waiting to retry.")
                return
            self.goal_handle, self.active = handle, True
            handle.get_result_async().add_done_callback(lambda f: self.goal_result(f, serial))
        except Exception as exc:
            if serial != self.generation:
                return
            self.cancel_navigation()
            self.hold_until = self.now()+2.0
            self.report(f"Navigation request failed: {exc}")

    def goal_result(self, future, serial):
        if serial != self.generation:
            return
        self.active, self.goal_handle, self.last_target = False, None, -1
        self.enable(False)
        try:
            status = future.result().status
            if status != GoalStatus.STATUS_SUCCEEDED:
                self.report("Navigation stopped; waiting before a sensor-based replan.")
                self.hold_until = self.now()+2.0
        except Exception as exc:
            self.report(f"Navigation result failed: {exc}")
            self.hold_until = self.now()+2.0

    def next_leg(self):
        self.cancel_navigation()
        self.leg, self.progress, self.last_target = self.leg+1, 0, -1
        self.hold_until = self.now()+1.0
        if self.leg == len(self.legs):
            self.finished = True
            self.save_at = self.now()+3.0
            self.report("Survey complete. Returned to the entrance; waiting for the final map update.")
        else:
            self.report(f"Starting {self.legs[self.leg][0]} leg after stopping.")

    def tick(self):
        if self.finished:
            self.enable(False)
            if self.saver is None and not self.save_reported and self.now() >= self.save_at:
                output = (self.root/self.config["map_output"]).resolve()
                if not output.is_relative_to(self.root):
                    raise ValueError("Map output must stay inside the project")
                output.parent.mkdir(parents=True, exist_ok=True)
                try:
                    self.saver = subprocess.Popen([
                        "ros2", "run", "nav2_map_server", "map_saver_cli", "-f", str(output),
                        "--ros-args", "-p", "use_sim_time:=true", "-p", "save_map_timeout:=15.0"])
                    self.save_started_wall = time.monotonic()
                except OSError as exc:
                    self.save_reported = True
                    self.report(f"Map saver could not start: {exc}")
            if (self.saver and self.saver.poll() is None
                    and time.monotonic()-self.save_started_wall > 30.0
                    and not self.save_reported):
                self.saver.terminate()
                self.save_reported = True
                self.report("Map save timed out. Keep mapping open and retry the map saver.")
            if self.saver and self.saver.poll() is not None and not self.save_reported:
                self.save_reported = True
                self.report("Map saved under maps/parking_lot." if self.saver.returncode == 0
                            else "Map save failed. Keep mapping open and retry the map saver.")
            return
        if self.map is None or not self.map.info.width:
            self.wait("Waiting for /navigation_map from the SLAM map adapter; the robot remains stopped.")
            return
        if not 0 <= self.now()-self.map_time < 5.0:
            self.wait("SLAM map is stale; the robot remains stopped until mapping recovers.")
            return
        if not self.client.server_is_ready():
            self.wait("Waiting for the /navigate_to_pose action server.")
            return
        if self.sending and self.now()-self.last_send > 10.0:
            self.cancel_navigation()
            self.hold_until = self.now()+2.0
            self.report("Navigation request timed out; stopped before retrying.")
            return
        try:
            pose = self.robot_pose()
        except (TransformException, RuntimeError):
            self.wait("Waiting for a current map-to-robot transform.")
            return
        if self.home is None:
            self.home = pose
            self.publish_route()
            self.report("Map and navigation ready. Entering the lot autonomously.")
        dx, dy = pose[0]-self.home[0], pose[1]-self.home[1]
        c, s = math.cos(self.home[2]), math.sin(self.home[2])
        local = (c*dx+s*dy, -s*dx+c*dy, wrap(pose[2]-self.home[2]))
        name, points = self.legs[self.leg]
        self.progress = nearest_progress(points, local, self.progress)
        lengths = self.lengths[self.leg]
        end = points[-1]
        rear = (local[0]-self.robot["wheelbase_m"]/2*math.cos(local[2]),
                local[1]-self.robot["wheelbase_m"]/2*math.sin(local[2]))
        end_rear = rear_point(end, self.robot["wheelbase_m"])
        if (self.progress >= len(points)-5
                and math.hypot(rear[0]-end_rear.x, rear[1]-end_rear.y)
                    < self.config["goal_position_tolerance_m"]+0.02
                and abs(wrap(local[2]-end.yaw))
                    < self.config["goal_heading_tolerance_rad"]+0.02):
            self.next_leg()
            return
        if self.now() < self.hold_until:
            self.enable(False)
            return
        # Always stop fully before changing from forward survey to reverse return.
        if self.last_target < 0 and not self.active and self.speed > 0.05:
            self.enable(False)
            return
        if not self.ensure_direction(points[0].direction):
            return
        target_i = goal_index(lengths, self.progress, self.config["goal_horizon_m"])
        target = None
        while target_i >= self.progress:
            target = transform(rear_point(points[target_i], self.robot["wheelbase_m"]), self.home)
            if self.in_map(target):
                break
            target_i -= 1
        # Nearest-point progress can reach the final sample while the car is
        # still laterally outside goal tolerance. Keep that final goal enabled.
        if target_i < self.progress or (target_i == self.progress and target_i != len(points)-1):
            self.wait("Waiting for the laser map to cover the next route target.")
            return
        enough_change = (self.last_target < 0 or
                         lengths[target_i]-lengths[self.last_target]
                         >= self.config["goal_refresh_distance_m"])
        if not self.sending and enough_change and self.now()-self.last_send >= 1.0:
            self.last_target = target_i
            self.send_target(target)
        self.enable(self.active)


def main():
    rclpy.init()
    node = Patrol()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.cancel_navigation()
        if node.saver is not None and node.saver.poll() is None:
            node.saver.terminate()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
