#!/usr/bin/env python3
"""Sensor-driven exploration, parking-space comparison and supervised execution."""

import csv
from collections import deque
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time
import numpy as np

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point as RosPoint, Twist
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import Imu, JointState, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from parking_core import (Footprint, MovingReturns, OccupancyEvidence, PathFollower, aligned_aisle, analytical_parking,
                          audit_path, candidate_poses, course_speed_profile, distance, inverse, measured_entry_width,
                          observation_maneuver, parking_observation_path,
                          retreat_path, row_axis_vote, scan_rays, slot_score, split_path,
                          survey_goals, surveyed_bay_score, tapered_bay_rejection)
from parking_ros import (SCAN_TOPICS, from_pose, pose_message, scan_pose,
                         spin_until_stopped, stamp_seconds, twist, xyz_yaw, yaw)
from parking_localization import CourseLocalization, ScanLikelihood, localization_hits
from patrol_route import Point, build_route, goal_index, rear_point, route_lengths, transform, wrap


class ParkingMission(Node):
    def __init__(self):
        super().__init__("parking_mission", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.declare_parameter("output_root", "")
        self.declare_parameter("entrance_start", True)
        self.declare_parameter("localization_offset", [0.0, 0.0, 0.0])
        project = Path(__file__).resolve().parents[1]
        self.settings = json.loads((project/"config/parking.json").read_text())
        self.robot = json.loads((project/"config/limo.json").read_text())
        self.footprint = Footprint.from_robot(self.robot)
        self.course_config = json.loads((project/"config/patrol.json").read_text())
        output_root = self.get_parameter("output_root").value or str(project/"results")
        name = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
        self.output = Path(output_root)/name
        self.output.mkdir(parents=True)
        self.samples_file = (self.output/"run.csv").open("w", newline="", encoding="utf-8")
        self.samples = csv.writer(self.samples_file)
        self.samples.writerow(("sim_s", "phase", "guard", "odom_x", "odom_y", "odom_heading",
                               "odom_v", "odom_yaw_rate", "request_v", "request_yaw_rate",
                               "final_v", "final_yaw_rate", "steering_left", "steering_right",
                               "imu_yaw_rate", "world_rear_x", "world_rear_y", "world_heading",
                               "odom_stamp_s", "joint_stamp_s", "imu_stamp_s", "world_stamp_s",
                               "navigation_source", "nav_map_x", "nav_map_y", "nav_map_heading"))
        self.events_file = (self.output/"decisions.jsonl").open("w", encoding="utf-8")
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.client = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.command = self.create_publisher(Twist, "/parking/requested_cmd", 10)
        self.status = self.create_publisher(String, "/parking/status", latched)
        self.path_publisher = self.create_publisher(RosPath, "/parking/active_path", latched)
        self.course_publisher = self.create_publisher(RosPath, "/parking/inspection_path_debug", latched)
        self.inspection_publisher = self.create_publisher(MarkerArray, "/parking/inspection_markers", latched)
        self.map_publisher = self.create_publisher(OccupancyGrid, "/parking/evidence_map", latched)
        self.layers = {topic: OccupancyEvidence(self.settings["grid_resolution_m"]) for topic in SCAN_TOPICS}
        self.pending_scans, self.scan_stamps = {}, {}
        self.axis_vote = 0j
        for topic in SCAN_TOPICS:
            self.create_subscription(LaserScan, topic, lambda msg, t=topic: self.on_scan(t, msg),
                                     qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self.on_odom, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, latched)
        self.create_subscription(String, "/parking/guard", self.on_guard, latched)
        self.create_subscription(JointState, "/joint_states", self.on_joints, qos_profile_sensor_data)
        self.create_subscription(Imu, "/imu", self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/parking/evaluation_pose", self.on_truth, qos_profile_sensor_data)
        self.create_subscription(Twist, "/cmd_vel", self.on_final_command, qos_profile_sensor_data)
        self.odom = self.slam_map = self.truth = self.imu = self.joints = self.final_command = None
        self.slam_map_time = -math.inf
        self.guard = "starting"
        self.phase = "initializing"
        self.epoch = None
        self.boot_wall = time.monotonic()
        self.next_choice = 0.0
        self.last_sample = self.last_grid = -math.inf
        self.route_started = not self.get_parameter("entrance_start").value
        self.entrance_start = not self.route_started
        self.course = deque()
        self.course_start = 0
        self.course_blocked = False
        self.course_rejoins = 0
        self.course_speeds = None
        self.inspection_since = None
        self.inspection_outcomes = {}
        self.probe_used = False
        self.inspection_marker_counts = {}
        self.probe_return = None
        self.localization = None
        self.localization_stamp = -math.inf
        self.localization_report = -math.inf
        self.localized_at = -math.inf
        self.phase_before_localizing = None
        self.using_particle_pose = False
        self.navigation_alignment = None
        self.active_station = None
        self.survey_finished = False
        self.moving_returns = MovingReturns(minimum_interval=0.09)
        self.motion_scan = None
        self.last_motion_scan = -math.inf
        self.motion_history = deque()
        self.visited, self.rejected = [], []
        self.known_slots = []
        self.survey_failures = 0
        self.traveled = []
        self.survey_trace = []
        self.retreat_requested = False
        self.recover_until = -math.inf
        self.failed_parking = []
        self.follower = self.plan = self.grid = self.target = self.plan_goal = None
        self.staged = False
        self.kind = ""
        self.plan_id = 0
        self.path_start = None
        self.requested = (0.0, 0.0)
        self.last_motion = self.stopped_since = None
        self.last_motion_pose = None
        self.park_anchor = None
        self.complete = False
        self.create_timer(0.10, self.tick)
        self.create_timer(0.32, self.insert_scans)
        self.create_timer(0.10, self.track_motion)
        self.create_timer(0.32, self.localize_course)
        self.create_timer(1.0, self.check_startup_timeout, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(f"Parking run evidence: {self.output}")

    def now(self):
        return self.get_clock().now().nanoseconds*1e-9

    def check_startup_timeout(self):
        if not self.complete and self.epoch is None and time.monotonic()-self.boot_wall > 100.0:
            self.finish("failed", "startup_timed_out_waiting_for_sensors_slam_or_planner")

    def on_scan(self, topic, message):
        self.pending_scans[topic] = message
        if topic == SCAN_TOPICS[0] and self.entrance_start:
            self.motion_scan = message

    def track_motion(self):
        if self.complete or self.motion_scan is None:
            return
        scan = self.motion_scan
        stamp = stamp_seconds(scan.header.stamp)
        if stamp <= self.last_motion_scan or not 0 <= self.now()-stamp <= 0.35:
            return
        pose = scan_pose(self.buffer, scan)
        if pose is None:
            return
        self.last_motion_scan = stamp
        rays = list(scan_rays(scan, pose, 14.0))
        if not rays or len(rays) < 0.65*len(scan.ranges):
            return
        hits = np.asarray([(x, y) for x, y, hit in rays if hit]).reshape(-1, 2)
        self.moving_returns.update(hits, stamp)
        moving = self.moving_returns.active(stamp)
        if moving is not None and moving.size:
            self.motion_history.append((stamp, moving[:, 0].copy()))
        while self.motion_history and stamp-self.motion_history[0][0] > 5.0:
            self.motion_history.popleft()

    def insert_scans(self):
        if self.complete:
            return
        for topic, scan in list(self.pending_scans.items()):
            stamp = stamp_seconds(scan.header.stamp)
            if self.now()-stamp > 0.7:
                del self.pending_scans[topic]
                continue
            pose = scan_pose(self.buffer, scan)
            if pose is None:
                continue
            del self.pending_scans[topic]
            rays = list(scan_rays(scan, pose, 18.0 if topic == "/scan" else 6.0))
            if len(rays) < 0.65*len(scan.ranges) or not rays:
                continue
            self.layers[topic].insert((pose.x, pose.y), rays)
            if topic == SCAN_TOPICS[0]:
                self.axis_vote = self.axis_vote*0.97+row_axis_vote(rays)
            self.scan_stamps[topic] = stamp

    def on_odom(self, msg):
        self.odom = msg

    def on_map(self, msg):
        self.slam_map, self.slam_map_time = msg, self.now()

    def localize_course(self):
        if self.complete or self.localization is None or self.motion_scan is None:
            return
        scan = self.motion_scan
        stamp = stamp_seconds(scan.header.stamp)
        if stamp <= self.localization_stamp or not 0 <= self.now()-stamp <= 0.42:
            return
        pose = scan_pose(self.buffer, scan)
        if pose is None:
            return
        self.localization_stamp = stamp
        hits = localization_hits(scan)
        if len(hits) < 12:
            if stamp-self.localization_report > 3.0:
                self.localization_report = stamp
                self.event("localization_waiting_for_laser_hits", valid_hits=len(hits))
            return
        was_ready = self.localization.alignment is not None
        event, estimate, mass, match = self.localization.update(pose, hits)
        if event == "particle_tracking":
            self.localized_at = stamp
        if (event in ("markov_initialized_particles", "particle_relocalizing")
                or (not was_ready and event == "particle_tracking") or stamp-self.localization_report > 2.0):
            self.localization_report = stamp
            self.event(event, sensor_stamp=stamp, map_laser_pose=estimate,
                       odom_laser_pose=[pose.x, pose.y, pose.yaw], cluster_mass=mass, scan_match=match,
                       active_map_from_odom=self.localization.alignment)

    def on_guard(self, msg):
        self.guard = msg.data

    def on_joints(self, msg):
        self.joints = msg

    def on_imu(self, msg):
        self.imu = msg

    def on_truth(self, msg):
        self.truth = msg

    def on_final_command(self, msg):
        self.final_command = msg

    def publish(self, v=0.0, w=0.0):
        self.requested = (v, w)
        self.command.publish(twist(v, w))

    def event(self, event, **values):
        record = {"sim_s": round(self.now(), 3), "event": event, **values}
        self.events_file.write(json.dumps(record, allow_nan=False)+"\n")
        self.events_file.flush()

    def transition(self, phase, detail):
        if phase != self.phase:
            self.phase = phase
            self.status.publish(String(data=f"{phase}: {detail}"))
            self.get_logger().info(f"{phase}: {detail}")
            self.event(phase, detail=detail)

    def log_sample(self, now):
        if not self.odom or now-self.last_sample < 0.2:
            return
        self.last_sample = now
        odom = from_pose(self.odom.pose.pose)
        odom_twist = self.odom.twist.twist
        truth = from_pose(self.truth.pose.pose) if self.truth else None
        navigation = (transform(odom, self.navigation_alignment)
                      if self.navigation_alignment is not None else None)
        source = ("particle_filter" if self.using_particle_pose else "slam_toolbox") if navigation else None
        joints = dict(zip(self.joints.name, self.joints.position)) if self.joints else {}
        self.samples.writerow((round(now, 3), self.phase, self.guard, odom.x, odom.y, odom.yaw,
                               odom_twist.linear.x, odom_twist.angular.z, *self.requested,
                               self.final_command.linear.x if self.final_command else None,
                               self.final_command.angular.z if self.final_command else None,
                               joints.get("front_left_steering_joint"), joints.get("front_right_steering_joint"),
                               self.imu.angular_velocity.z if self.imu else None,
                               truth.x if truth else None, truth.y if truth else None, truth.yaw if truth else None,
                               stamp_seconds(self.odom.header.stamp),
                               stamp_seconds(self.joints.header.stamp) if self.joints else None,
                               stamp_seconds(self.imu.header.stamp) if self.imu else None,
                               stamp_seconds(self.truth.header.stamp) if self.truth else None, source,
                               navigation.x if navigation else None, navigation.y if navigation else None,
                               navigation.yaw if navigation else None))
        self.samples_file.flush()

    def snapshot(self, now):
        grid = self.layers["/scan"].snapshot(self.layers[topic] for topic in SCAN_TOPICS[1:])
        if grid is not None and now-self.last_grid >= 1.0:
            msg = OccupancyGrid()
            msg.header.frame_id, msg.header.stamp = "odom", self.get_clock().now().to_msg()
            msg.info.resolution, msg.info.width, msg.info.height = grid.resolution, grid.width, grid.height
            msg.info.origin.position.x = grid.origin[0]*grid.resolution
            msg.info.origin.position.y = grid.origin[1]*grid.resolution
            msg.info.origin.orientation.w = 1.0
            msg.data = grid.probabilities()
            self.map_publisher.publish(msg)
            self.last_grid = now
        return grid

    def map_alignment(self):
        alignment = self.buffer.lookup_transform("map", "odom", Time())
        if abs(self.now()-stamp_seconds(alignment.header.stamp)) > 3.0:
            raise RuntimeError("stale SLAM alignment")
        return xyz_yaw(alignment.transform)

    def request_plan(self, pose, target, kind, grid, alignment, staging=None):
        goal = ComputePathToPose.Goal()
        now = self.get_clock().now().to_msg()
        self.plan_goal = target if staging is None else staging
        self.staged = staging is not None
        goal.goal = pose_message(transform(self.plan_goal, alignment), "map", now)
        goal.start = pose_message(transform(pose, alignment), "map", now)
        goal.use_start, goal.planner_id = True, "GridBased"
        self.plan_id += 1
        token = self.plan_id
        self.plan = (token, self.now(), alignment, pose, None)
        self.kind, self.target, self.grid = kind, target, grid
        self.transition(f"planning_{kind}", "The vehicle is held while the route is planned.")
        self.client.send_goal_async(goal).add_done_callback(lambda result: self.plan_response(result, token))

    def plan_response(self, result, token):
        try:
            handle = result.result()
            if not self.plan or token != self.plan[0]:
                if handle.accepted:
                    handle.cancel_goal_async()
                return
            if not handle.accepted:
                self.plan_failed("planner_rejected")
                return
            self.plan = (*self.plan[:4], handle)
            handle.get_result_async().add_done_callback(lambda future: self.plan_result(future, token))
        except Exception as error:
            if self.plan and token == self.plan[0]:
                self.plan_failed(f"planner_request_error: {error}")

    def plan_result(self, result, token):
        if not self.plan or self.plan[0] != token:
            return
        _, _, alignment, start, _ = self.plan
        try:
            response = result.result()
            if response.status != GoalStatus.STATUS_SUCCEEDED:
                self.plan_failed(f"planner_result_{response.result.error_code}")
                return
            if response.result.path.header.frame_id != "map":
                self.plan_failed("unexpected_path_frame")
                return
            points = [transform(from_pose(pose.pose), inverse(alignment))
                      for pose in response.result.path.poses]
            if len(points) < 2 or distance(start, points[0]) > 0.48:
                self.plan_failed("empty_or_disconnected_path")
                return
            tolerance = 0.14 if self.staged else 0.22
            if (distance(self.plan_goal, points[-1]) > tolerance
                    or abs(wrap(self.plan_goal.yaw-points[-1].yaw)) > 0.13):
                self.plan_failed("path_stops_short_or_at_wrong_heading")
                return
            if self.staged:
                start_insert = points[-1]
                delta = wrap(self.target.yaw-start_insert.yaw)
                steps = math.ceil(distance(start_insert, self.target)/0.12)
                points.extend(Point(start_insert.x+(self.target.x-start_insert.x)*i/steps,
                                    start_insert.y+(self.target.y-start_insert.y)*i/steps,
                                    start_insert.yaw+delta*i/steps, 0)
                              for i in range(1, steps+1))
            legs = split_path(points)
            if not legs:
                self.plan_failed("empty_or_disconnected_path")
                return
            tight = self.kind == "parking"
            if tight and (legs[-1][0] != -1 or distance(legs[-1][1][0], points[-1]) < 0.45):
                self.plan_failed("no_final_reverse_approach")
                return
            if not tight and not self.route_started and legs[0][0] != 1:
                self.plan_failed("initial_motion_must_be_forward")
                return
            if self.kind == "rejoin" and (len(legs) > 2 or legs[-1][0] != self.rejoin_direction):
                self.plan_failed("rejoin_wrong_direction")
                return
            why = audit_path(self.grid, legs, self.footprint, self.settings["parking_clearance_m"],
                             strict=tight)
            if why:
                self.plan_failed(why)
                return
        except Exception as error:
            self.plan_failed(f"invalid_plan: {error}")
            return
        self.plan = None
        if tight:
            self.event("comparison", goal=[self.target.x, self.target.y, self.target.yaw],
                       final_pose_only=True, swept_path="accepted", direction_changes=len(legs)-1,
                       staged_reverse=self.staged)
        self.follow(start, points, legs)

    def follow(self, start, points, legs):
        self.publish_path(points, "odom")
        self.follower = PathFollower(legs)
        self.path_start = start
        if self.kind == "survey":
            self.record_survey_pose(start)
        self.last_motion_pose, self.last_motion = start, self.now()
        self.transition(f"executing_{self.kind}", "Path accepted after full-footprint inspection.")

    def publish_path(self, points, frame, publisher=None):
        path = RosPath()
        path.header.frame_id, path.header.stamp = frame, self.get_clock().now().to_msg()
        path.poses = [pose_message(point, frame, path.header.stamp) for point in points]
        (publisher if publisher is not None else self.path_publisher).publish(path)

    def cancel_plan(self):
        if self.plan and self.plan[4] is not None:
            self.plan[4].cancel_goal_async()
        self.plan = None

    def plan_failed(self, reason):
        attempt = self.plan
        self.cancel_plan()
        target = [self.target.x, self.target.y, self.target.yaw]
        if self.kind == "parking":
            if attempt and not self.staged and reason in (
                    "no_final_reverse_approach", "occupied_sweep", "planner_result_208"):
                staging = transform(Point(1.30, 0, 0, 0), tuple(target))
                insertion = [(-1, [staging, self.target])]
                unsafe = audit_path(self.grid, insertion, self.footprint,
                                    self.settings["parking_clearance_m"], strict=True)
                if not unsafe:
                    self.event("direct_parking_plan_rejected", goal=target, reason=reason)
                    self.request_plan(attempt[3], self.target, self.kind, self.grid, attempt[2],
                                      staging=staging)
                    return
            self.event("comparison", goal=target, final_pose_only=True, swept_path=reason)
            self.failed_parking.append((self.target, self.now()))
        else:
            self.event("survey_target_rejected", goal=target, reason=reason)
            self.rejected.append((self.target, self.now()))
            self.survey_failures += 1
            if self.survey_failures >= 7 and self.now() >= self.recover_until:
                self.retreat_requested = True
        self.next_choice = self.now()+0.25
        self.transition("choosing", reason)

    def choose(self, pose, now, alignment):
        self.grid = self.snapshot(now)
        if self.grid is None:
            return
        if self.entrance_start:
            self.choose_course(pose, now, alignment)
            return
        if self.retreat_requested:
            self.retreat_requested = False
            if self.start_retreat(pose, now):
                return
        recent = [goal for goal, stamp in self.failed_parking if now-stamp < 35.0]
        heading = aligned_aisle(pose.yaw, self.axis_vote)
        margin = self.settings["parking_clearance_m"]
        if self.route_started:
            for target, legs, unsafe in analytical_parking(
                    self.grid, pose, heading, self.footprint, recent, self.settings["planning_radius_m"],
                    margin, survey_trace=self.survey_trace):
                self.event("comparison", goal=[target.x, target.y, target.yaw], final_pose_only=True,
                           swept_path=unsafe or "accepted", source="analytical_reverse_arc")
                if not unsafe:
                    self.kind, self.target = "parking", target
                    self.follow(pose, legs[0][1], legs)
                    return
        candidates = candidate_poses(self.grid, pose, self.footprint, recent,
                                     margin, heading, survey_trace=self.survey_trace) if self.route_started else []
        # A car turning through an intersection can face either map axis.
        # Search the other observed axis when its current-facing side has
        # no slot. The same flank and entry evidence is still required.
        if self.route_started and not candidates and abs(self.axis_vote) >= 1.0:
            candidates = candidate_poses(self.grid, pose, self.footprint, recent,
                                         margin, wrap(heading+math.pi/2), survey_trace=self.survey_trace)
        for candidate in candidates:
            if all(distance(candidate, old) > 0.55 or abs(wrap(candidate.yaw-old.yaw)) > 0.25
                   for old in self.known_slots):
                self.known_slots.append(candidate)
        self.known_slots = self.known_slots[-60:]
        if not candidates:
            previous = sorted((old for old in self.known_slots if distance(old, pose) < 14.0
                               and all(distance(old, bad) > 0.7 for bad in recent)),
                              key=lambda old: distance(pose, old))
            candidates = [old for old in previous if slot_score(
                self.grid, old, self.footprint, margin, self.survey_trace) is not None]
        if candidates:
            self.request_plan(pose, candidates[0], "parking", self.grid, alignment)
            return
        rejected = [goal for goal, stamp in self.rejected if now-stamp < 20]
        goals = survey_goals(self.grid, pose, self.footprint, self.visited[-35:], rejected)
        if goals:
            self.request_plan(pose, goals[0], "survey", self.grid, alignment)
        else:
            self.transition("waiting_for_clear_route", "Continue observing while stationary.")
            self.next_choice = now+2.0

    def initialize_course(self, pose, alignment):
        wheelbase = self.robot["wheelbase_m"]
        home = transform(pose, alignment)
        center = transform(Point(wheelbase/2, 0, 0, 0), (home.x, home.y, home.yaw))
        route = {
            name: [transform(rear_point(point, wheelbase), (center.x, center.y, center.yaw))
                   for point in points]
            for name, points in build_route(self.course_config, wheelbase)
        }
        loop = route["loop"]
        lengths = route_lengths(loop)
        self.course.extend((phase, route[name], None) for phase, name in (
            ("mapping_entry", "entry"), ("mapping_loop", "loop"), ("return_home", "return"),
            ("inspection_entry", "entry")))
        previous = 0
        for station in self.course_config["parking_inspections"]:
            index = goal_index(lengths, 0, station["check_distance_m"])
            anchor = loop[goal_index(lengths, 0, station["aisle_distance_m"])]
            self.course.append(("inspection_transit", loop[previous:index+1], (station, anchor)))
            previous = index
        # A second marked position gives the sensors another viewing angle
        # and the car another feasible start for the same final inspection.
        self.course.append(("inspection_transit", loop[previous:previous+7],
                            (station, anchor)))
        self.course_origin = home

    def course_outcome(self, scenario, decision, *, detail=None, **evidence):
        self.inspection_outcomes[scenario] = decision
        if detail is not None:
            evidence["detail"] = detail
        self.event("inspection", station=scenario, decision=decision, **evidence)
        self.get_logger().info(f"Test scenario {scenario}: {detail or decision.replace('_', ' ')}.")

    def choose_course(self, pose, now, alignment):
        if not self.course:
            self.finish("failed", "inspection_loop_finished_without_a_safe_parking_bay")
            return
        phase, points, inspection = self.course[0]
        if self.inspection_since is not None:
            self.inspect_station(pose, now, alignment, inspection)
            return
        if self.course_blocked:
            self.rejoin_course(pose, now, alignment, points)
            return
        self.kind = "course"
        remaining = points[self.course_start:]
        direction = points[0].direction
        if direction < 0:
            self.course_cruise = self.settings["survey_return_speed_m_s"]
            self.course_speeds = None
        else:
            self.course_cruise = self.settings["inspection_transit_speed_m_s" if self.survey_finished
                                               else "mapping_speed_m_s"]
            self.course_speeds = course_speed_profile(remaining, self.course_cruise, 0.65*self.settings["braking_m_s2"])
        self.follower = PathFollower([(direction, remaining)])
        self.publish_path(remaining, "map", self.course_publisher)
        self.publish_path([], "odom")
        self.last_motion_pose, self.last_motion = pose, now
        detail = {
            "mapping_entry": "Starting autonomous inspection and online mapping in an unmapped environment.",
            "mapping_loop": "Autonomous inspection and online mapping with live obstacle protection.",
            "return_home": "Returning to the entrance to save the first-lap map.",
            "inspection_entry": "Starting the second lap with laser localization.",
            "inspection_transit": "Continuing to the next parking test.",
        }
        self.transition(phase, detail[phase])

    def rejoin_course(self, pose, now, alignment, points):
        lengths = route_lengths(points)
        start = self.course_start
        direction = points[0].direction
        # Preserve the next inspection stop and never jump to a later section.
        for horizon in (4.5, 6.5, 3.0, 8.5, 1.8):
            index = goal_index(lengths, start, horizon)
            if index >= len(points)-3 or index <= start+3:
                continue
            target = transform(points[index], inverse(alignment))
            if any(distance(target, old) < 0.6 and now-stamp < 25 for old, stamp in self.rejected):
                continue
            occupied, observed = self.grid.area(target, self.footprint.bounds(0.12))
            if not occupied and observed >= 0.90:
                self.rejoin_index = index
                self.rejoin_direction = direction
                self.request_plan(pose, target, "rejoin", self.grid, alignment)
                return
        if self.retreat_requested:
            self.retreat_requested = False
            if self.start_retreat(pose, now):
                return
        self.transition("waiting_for_clear_route", "No observed, safe connection to the next course section.")
        self.next_choice = now+1.0

    def drive_course(self, pose, speed, now, alignment):
        phase, points, inspection = self.course[0]
        map_pose = transform(pose, alignment)
        self.record_survey_pose(pose)
        if distance(pose, self.last_motion_pose) > 0.14:
            self.last_motion_pose, self.last_motion = pose, now
            self.route_started = True
            self.traveled.append(pose)
            self.traveled = self.traveled[-75:]
        cruise = self.course_cruise
        if inspection and distance(map_pose, points[-1]) < 4.2:
            cruise = min(cruise, self.settings["inspection_speed_m_s"])
        action, v, w, _ = self.follower.update(map_pose, speed, cruise,
                                                self.settings["braking_m_s2"], False, 0.34, self.course_speeds)
        if action == "deviated" or now-self.last_motion > self.settings["obstacle_wait_s"]:
            self.course_start = min(len(points)-2, self.course_start+self.follower.index)
            self.course_blocked = True
            self.course_rejoins += 1
            self.replan(pose, now, "tracking_error" if action == "deviated" else self.guard)
        elif action == "settled":
            self.follower = None
            self.publish()
            if phase == "return_home":
                self.survey_finished = True
                self.save_map(self.snapshot(now), basename="first_lap_map")
                self.event("lap_one_complete", return_error_m=distance(map_pose, self.course_origin),
                           heading_error_deg=math.degrees(abs(wrap(map_pose.yaw-self.course_origin.yaw))))
                info = self.slam_map.info
                data = np.asarray(self.slam_map.data, dtype=np.int8).reshape(info.height, info.width)
                origin = info.origin.position
                model = ScanLikelihood(data, info.resolution, (origin.x, origin.y, yaw(info.origin.orientation)))
                offset = self.get_parameter("localization_offset").value
                self.localization = CourseLocalization(model, alignment, offset)
                self.event("markov_localization_started", initial_offset=offset,
                           map_size=[info.width, info.height], map_resolution_m=info.resolution)
            if inspection:
                self.inspection_since = now
                self.transition("inspecting", f"Checking the approach for test scenario {inspection[0]['id']}.")
            else:
                self.advance_course()
        else:
            self.publish(v, w)

    def advance_course(self):
        self.course.popleft()
        self.course_start = 0
        self.course_blocked = False
        self.inspection_since = None

    def show_inspection(self, scenario, detail, anchor, outlines=(), returns=()):
        stamp = self.get_clock().now().to_msg()
        markers = []

        def marker(kind, color):
            item = Marker()
            item.header.frame_id, item.header.stamp = "odom", stamp
            item.ns, item.id = f"parking_test_{scenario}", len(markers)
            item.type, item.action = kind, Marker.ADD
            item.pose.orientation.w = 1.0
            item.color.r, item.color.g, item.color.b, item.color.a = (*color, 1.0)
            item.lifetime.sec = 10
            markers.append(item)
            return item

        rejected = detail.startswith("Rejected")
        label = marker(Marker.TEXT_VIEW_FACING, (1.0, 0.83, 0.35) if rejected else (1.0, 1.0, 1.0))
        label.pose.position.x, label.pose.position.y, label.pose.position.z = anchor.x, anchor.y, 1.7
        label.scale.z = 0.76
        label.text = f"Test scenario {scenario} | {detail}"
        for pose, bounds, color in outlines:
            back, front, width = bounds
            corners = [transform(Point(x, y, 0, 0), (pose.x, pose.y, pose.yaw))
                       for x, y in ((back, -width), (front, -width), (front, width),
                                    (back, width), (back, -width))]
            outline = marker(Marker.LINE_STRIP, color)
            outline.scale.x = 0.15
            outline.points = [RosPoint(x=p.x, y=p.y, z=0.24) for p in corners]
        if returns:
            points = marker(Marker.SPHERE_LIST, (1.0, 0.74, 0.14))
            points.scale.x = points.scale.y = points.scale.z = 0.42
            points.points = [RosPoint(x=float(x), y=float(y), z=0.30) for x, y in returns]
        count = len(markers)
        for _ in range(count, self.inspection_marker_counts.get(scenario, 0)):
            marker(Marker.LINE_STRIP, (1.0, 1.0, 1.0)).action = Marker.DELETE
        self.inspection_marker_counts[scenario] = count
        self.inspection_publisher.publish(MarkerArray(markers=markers))
        self.event("inspection_visualized", station=scenario, label=detail,
                   body_outlines=len(outlines), tracked_returns=len(returns))

    def start_inspection_probe(self, pose, entrance):
        legs = parking_observation_path(self.grid, pose, entrance, self.footprint,
                                        self.settings["planning_radius_m"])
        if legs is None:
            self.event("inspection_probe_skipped", station=self.course[0][2][0]["id"],
                       reason="no_observed_clear_reverse_and_exit_toward_mouth")
            return False
        self.kind = "inspection_probe"
        self.probe_return = legs[1][1]
        self.follow(pose, legs[0][1], legs[:1])
        self.event("inspection_probe_started", station=self.course[0][2][0]["id"], length_m=1.3)
        return True

    def drive_inspection_probe(self, pose, speed, now):
        action, v, w, _ = self.follower.update(pose, speed, 0.30, self.settings["braking_m_s2"], True)
        if distance(pose, self.last_motion_pose) > 0.10:
            self.last_motion_pose, self.last_motion = pose, now
        if action == "deviated" or now-self.last_motion > self.settings["obstacle_wait_s"]:
            self.finish("failed", "inspection_observation_maneuver_interrupted")
        elif action == "settled":
            self.publish()
            self.follower = None
            if self.probe_return is None:
                scenario = self.course[0][2][0]["id"]
                self.event("inspection_probe_returned", station=scenario)
                self.inspection_since = now
                self.transition("inspecting", f"Rechecking test scenario {scenario} with new observations.")
            else:
                self.transition("checking_probe_exit", "Checking the forward exit with fresh observations.")
        else:
            self.publish(v, w)

    def resume_probe_exit(self, pose, now):
        points = [pose, *self.probe_return[1:]]
        legs = [(1, points)]
        if audit_path(self.snapshot(now), legs, self.footprint, 0.14, strict=True):
            if now-self.last_motion > self.settings["obstacle_wait_s"]:
                self.finish("failed", "inspection_observation_exit_not_clear")
            return
        self.probe_return = None
        self.follow(pose, points, legs)

    def inspect_station(self, pose, now, alignment, inspection):
        station, map_anchor = inspection
        name = station["id"]
        anchor = transform(map_anchor, inverse(alignment))
        origin = (anchor.x, anchor.y, anchor.yaw)
        relative = inverse(origin)
        if now-self.inspection_since < 2.8:
            return
        margin = self.settings["parking_clearance_m"]
        # Use sustained motion measured at standstill. Two brief updates
        # while approaching can be caused by changing visibility of a car edge.
        crossing = []
        moving_points = []
        for stamp, centers in self.motion_history:
            if stamp < self.inspection_since or now-stamp > 4.5:
                continue
            local = [transform(Point(x, y, 0, 0), relative) for x, y in centers]
            inside = [(x, y) for (x, y), p in zip(centers, local) if abs(p.x) <= 4.0 and -8.2 <= p.y <= -2.0]
            if inside:
                crossing.append(stamp)
                moving_points.extend(inside)
        if len(crossing) >= 5 and crossing[-1]-crossing[0] >= 0.45:
            crossing_zone = transform(Point(0, -5.1, 0, 0), origin)
            self.show_inspection(name, "Rejected | pedestrian crossing at entrance", anchor,
                                 [(crossing_zone, (-4.0, 4.0, 3.1), (1.0, 0.35, 0.12))],
                                 returns=moving_points)
            self.course_outcome(name, "moving_obstacle_in_mouth", tracking_updates=len(crossing))
            self.advance_course()
            return

        def in_window(goal):
            local = transform(goal, relative)
            return abs(local.x) <= 1.65 and -7.65 <= local.y <= -6.25

        # A final-pose-only baseline and straight entry probe share the same
        # footprint samples. An authorized move still requires a feasible,
        # audited Ackermann reverse trajectory in this evidence snapshot.
        final_fits = entry_fits = 0
        blocked_sweeps = []
        occupied_finals = []
        rear_checks = rear_occupied = 0
        for along in np.arange(-1.4, 1.41, 0.35):
            for depth in np.arange(5.8, 8.61, 0.35):
                final = transform(Point(along, -depth, math.pi/2, 0), origin)
                occupied, free = self.grid.area(final, self.footprint.bounds(margin, parked=True))
                # Use the deeper samples within the valid target depth range.
                if name == 4 and 7.2 <= round(depth, 2) <= 7.65:
                    rear_checks += 1
                    rear_occupied += int(occupied)
                rank = abs(along)+abs(depth-6.75)
                if occupied:
                    occupied_finals.append((rank, final))
                if occupied or free < 0.94:
                    continue
                final_fits += 1
                mouth = transform(Point(along, -1.2, math.pi/2, 0), origin)
                rejection = audit_path(self.grid, [(-1, [mouth, final])], self.footprint, margin, strict=True)
                if rejection is None:
                    entry_fits += 1
                else:
                    blocked_sweeps.append((rank, mouth, final, rejection))
        width = measured_entry_width(self.grid, anchor)
        required_width = 2*(self.footprint.half_width+margin)
        centers = np.arange(-1.4, 1.41, 0.35)
        mouth_bounds = (-required_width/2, required_width/2, 1.25)
        mouth_probes = [transform(Point(float(x), -3.15, 0, 0), origin) for x in centers]
        blocked_probes = [probe for probe in mouth_probes if self.grid.area(probe, mouth_bounds)[0]]
        mouth_blocked = len(blocked_probes)
        narrow = (width is not None and width < required_width) or mouth_blocked == len(centers)
        occupied_sweeps = sum(entry[3] == "occupied_sweep" for entry in blocked_sweeps)
        rear_evidence = {"rear_poses_checked": rear_checks, "rear_poses_occupied": rear_occupied} if name == 4 else {}
        self.event("aperture_measurement", station=name, final_poses_fitting=final_fits,
                   straight_sweeps_clear=entry_fits, observed_mouth_width_m=width,
                   required_width_m=required_width, mouth_probes_blocked=mouth_blocked,
                   mouth_probes=len(centers), occupied_final_poses=len(occupied_finals),
                   occupied_entry_sweeps=occupied_sweeps, **rear_evidence)
        if final_fits and not narrow:
            rejected = [goal for goal, stamp in self.failed_parking if now-stamp < 60]
            approved = []
            for target, legs, unsafe in analytical_parking(
                    self.grid, pose, anchor.yaw, self.footprint, rejected,
                    self.settings["planning_radius_m"], margin,
                    survey_trace=self.survey_trace, in_window=in_window, score_bay=surveyed_bay_score):
                self.event("comparison", station=name, goal=[target.x, target.y, target.yaw],
                           final_pose_only=True, swept_path=unsafe or "accepted", source="analytical_reverse_arc")
                if not unsafe:
                    local = transform(target, relative)
                    approved.append((abs(local.x)+abs(local.y+6.75), target, legs))
            if approved:
                _, target, legs = min(approved, key=lambda item: item[0])
                self.active_station = name
                self.kind, self.target = "parking", target
                self.show_inspection(name, "Accepted | verified reverse path", anchor,
                                     [(target, self.footprint.bounds(parked=True), (0.20, 1.0, 0.35))])
                self.follow(pose, legs[0][1], legs)
                return
        decision = ("mouth_too_narrow" if narrow else
                    "no_clear_perpendicular_footprint" if not final_fits else
                    "entry_sweep_blocked_or_unobserved" if not entry_fits else
                    "no_feasible_reverse_connection")
        red = (1.0, 0.26, 0.12)
        outlines = []
        if narrow:
            outlines = [(probe, mouth_bounds, red) for probe in blocked_probes]
            if name == 1:
                final = transform(Point(0, -6.75, math.pi/2, 0), origin)
                outlines.append((final, self.footprint.bounds(parked=True), (0.35, 0.85, 1.0)))
            detail = "Rejected | entrance obstructed"
        elif blocked_sweeps:
            _, mouth, final, rejection = min(blocked_sweeps, key=lambda entry: entry[0])
            back, front, width = self.footprint.bounds(margin)
            outlines = [(mouth, (back-distance(mouth, final), front, width), red),
                        (final, self.footprint.bounds(parked=True), (0.20, 1.0, 0.35))]
            detail = ("Rejected | entry sweep obstructed" if rejection == "occupied_sweep" else
                      "Rejected | entry sweep not fully observed")
            if station.get("observation_probe") and decision == "entry_sweep_blocked_or_unobserved" and not self.probe_used:
                self.probe_used = True
                entrance = transform(Point(0, -3.15, math.pi/2, 0), origin)
                if self.start_inspection_probe(pose, entrance):
                    self.show_inspection(name, "Checking entry from a new position", anchor, outlines)
                    return
        else:
            detail = ("Rejected | no clear final pose" if not final_fits else
                      "Rejected | no feasible reverse connection")
            if occupied_finals:
                _, final = min(occupied_finals, key=lambda entry: entry[0])
                outlines = [(final, self.footprint.bounds(parked=True), red)]
        if name == 4 and not narrow:
            detail = tapered_bay_rejection(width is not None and width >= required_width, rear_checks,
                                          rear_occupied, final_fits, occupied_sweeps)
            if decision == "entry_sweep_blocked_or_unobserved":
                decision = "entry_sweep_obstructed" if occupied_sweeps == final_fits else "entry_sweep_failed"
        self.show_inspection(name, detail, anchor, outlines)
        self.course_outcome(name, decision, detail=detail if name == 4 else None,
                            final_poses_fitting=final_fits, straight_sweeps_clear=entry_fits)
        self.advance_course()

    def start_retreat(self, pose, now):
        legs = retreat_path(self.traveled, pose, radius=3.5)
        # The wide steering rectangle is an outer bound for any wheel angle.
        # On a recent path already driven, still audit the body and continue
        # to check the larger envelope against fresh scans in the motion guard.
        body = Footprint(self.footprint.back, self.footprint.front,
                         self.footprint.parked_half_width, self.footprint.parked_half_width)
        from_history = bool(legs and not audit_path(self.grid, legs, body, 0.02))
        if not from_history and self.route_started and self.survey_failures >= 7:
            legs = observation_maneuver(self.grid, pose, self.footprint,
                                        self.settings["planning_radius_m"], self.visited)
        elif not from_history:
            return False
        if not legs:
            return False
        _, points = legs[0]
        self.kind, self.target = "recovery", points[-1]
        self.follower = PathFollower(legs)
        self.path_start, self.last_motion_pose, self.last_motion = pose, pose, now
        self.recover_until = now+30.0
        self.survey_failures = 0
        self.traveled = []
        detail = ("Retrace a short, observed portion of the previous route." if from_history
                  else "Move slowly through observed clear space before choosing another route.")
        self.transition("executing_recovery", detail)
        return True

    def replan(self, pose, now, reason):
        self.follower = None
        self.publish()
        self.event("execution_interrupted", task=self.kind, reason=reason)
        if self.kind == "parking":
            self.failed_parking.append((self.target, now))
        elif self.kind != "course":
            self.rejected.append((self.target, now))
            self.visited.append(pose)
        if self.kind != "recovery" and now >= self.recover_until:
            self.retreat_requested = True
        self.next_choice = now+0.6
        self.transition("choosing", reason)

    def record_survey_pose(self, pose):
        if not self.survey_trace or distance(pose, self.survey_trace[-1]) >= 0.20:
            self.survey_trace.append(pose)

    def drive(self, pose, speed, now):
        if self.kind == "survey":
            self.record_survey_pose(pose)
        if distance(pose, self.last_motion_pose) > 0.14:
            self.last_motion_pose, self.last_motion = pose, now
            self.traveled.append(pose)
            self.traveled = self.traveled[-75:]
            if self.kind == "survey" and distance(pose, self.path_start) > 0.50:
                self.route_started = True
        if now-self.last_motion > self.settings["obstacle_wait_s"]:
            self.replan(pose, now, self.guard if self.guard != "motion_clear" else "no_progress")
            return
        tight = self.kind == "parking"
        direction = self.follower.legs[0][0]
        if self.kind == "recovery":
            cruise = 0.30
        else:
            cruise = self.settings["reverse_speed_m_s"] if direction < 0 else self.settings[
                "approach_speed_m_s" if tight else "survey_speed_m_s"]
        action, v, w, _ = self.follower.update(pose, speed, cruise,
                                                self.settings["braking_m_s2"], tight)
        if action == "deviated":
            self.replan(pose, now, "tracking_error")
        elif action == "settled":
            self.publish()
            self.follower = None
            if tight:
                self.stopped_since = None
                self.park_anchor = None
                self.transition("verifying_park", "Confirming position, heading and stable standstill.")
            else:
                if self.kind == "rejoin":
                    self.course_start = self.rejoin_index
                    self.course_blocked = False
                self.survey_failures = 0
                self.visited.append(pose)
                self.next_choice = now+0.15
                self.transition("choosing", "Survey waypoint reached.")
        else:
            self.publish(v, w)
            if action == "shift":
                self.last_motion = now
                self.event("direction_change", stopped_v=speed)

    def validate_park(self, pose, speed, now):
        position = distance(pose, self.target)
        heading = math.degrees(abs(wrap(pose.yaw-self.target.yaw)))
        grid = self.snapshot(now)
        margin = self.settings["parking_clearance_m"]
        score_bay = surveyed_bay_score if self.entrance_start else slot_score
        if score_bay(grid, self.target, self.footprint, margin, self.survey_trace) is None:
            self.replan(pose, now, "target_no_longer_qualifies_as_parking_bay")
            return
        joints = dict(zip(self.joints.name, self.joints.position)) if self.joints else {}
        centered = all(abs(joints.get(f"front_{side}_steering_joint", math.inf)) < 0.12
                       for side in ("left", "right"))
        blocked, observed = grid.area(pose, self.footprint.bounds(margin, parked=centered))
        if (blocked or observed < 0.94 or position > self.settings["final_position_m"]
                or heading > self.settings["final_yaw_deg"]):
            self.replan(pose, now, "failed_final_pose_or_clearance")
            return
        if abs(speed) > 0.035 or (self.park_anchor and distance(pose, self.park_anchor) > 0.035):
            self.stopped_since = None
            self.park_anchor = None
            return
        if self.stopped_since is None:
            self.stopped_since = now
            self.park_anchor = pose
        if now-self.stopped_since >= self.settings["standstill_s"]:
            if self.active_station:
                self.course_outcome(self.active_station, "parked")
            self.finish("parked", "Parking bay, perpendicular reverse entry and standstill verified.",
                        estimated_position_error_m=position, estimated_heading_error_deg=heading,
                        observed_free_fraction=observed, target_odom=[self.target.x, self.target.y, self.target.yaw])

    def tick(self):
        now = self.now()
        if self.complete:
            self.publish()
            return
        self.log_sample(now)
        if self.guard == "chassis_contact":
            self.finish("failed", self.guard)
            return
        if self.epoch is not None and now-self.epoch >= self.settings["mission_timeout_s"]:
            self.finish("failed", "mission_timeout")
            return
        if (now <= 0 or self.odom is None or not 0 <= now-stamp_seconds(self.odom.header.stamp) < 0.34
                or self.slam_map is None or now-self.slam_map_time > 8.0
                or not all(topic in self.scan_stamps and 0 <= now-self.scan_stamps[topic] < 0.70
                           for topic in SCAN_TOPICS) or not self.client.server_is_ready()):
            self.publish()
            return
        pose = from_pose(self.odom.pose.pose)
        if self.localization is not None:
            alignment = self.localization.alignment
            if alignment is None or now-self.localized_at > 1.6:
                if self.phase != "localizing":
                    self.phase_before_localizing = self.phase
                    self.transition("localizing", "Holding while the laser localization converges.")
                self.using_particle_pose = False
                self.navigation_alignment = None
                self.last_motion = now
                self.publish()
                return
            if not self.using_particle_pose:
                self.using_particle_pose = True
                self.event("particle_pose_enabled_for_navigation", map_from_odom=alignment)
                if self.phase == "localizing":
                    self.transition(self.phase_before_localizing, "Particle localization acquired.")
        else:
            try:
                alignment = self.map_alignment()
            except (TransformException, RuntimeError):
                self.navigation_alignment = None
                self.publish()
                return
        self.navigation_alignment = alignment
        if self.epoch is None:
            self.epoch = now
            if self.entrance_start:
                self.initialize_course(pose, alignment)
            self.transition("choosing", "Laser, near-ground sensors, SLAM and planner are ready.")
        speed = self.odom.twist.twist.linear.x
        if self.follower:
            if self.kind == "course":
                self.drive_course(pose, speed, now, alignment)
            elif self.kind == "inspection_probe":
                self.drive_inspection_probe(pose, speed, now)
            else:
                self.drive(pose, speed, now)
            return
        self.publish()
        if self.phase == "verifying_park":
            self.validate_park(pose, speed, now)
        elif self.phase == "checking_probe_exit":
            self.resume_probe_exit(pose, now)
        elif self.plan:
            if now-self.plan[1] > 8.0:
                self.plan_failed("planner_timeout")
        elif now >= self.next_choice and abs(speed) <= 0.035:
            self.choose(pose, now, alignment)

    def save_map(self, grid, basename="parking_map"):
        if grid:
            image = np.full(grid.odds.shape, 205, dtype=np.uint8)
            image[grid.free], image[grid.occupied] = 254, 0
            (self.output/f"{basename}.pgm").write_bytes(
                f"P5\n{grid.width} {grid.height}\n255\n".encode()+image[::-1].tobytes())
            (self.output/f"{basename}.yaml").write_text(
                f"image: {basename}.pgm\nresolution: {grid.resolution}\n"
                f"origin: [{grid.origin[0]*grid.resolution}, {grid.origin[1]*grid.resolution}, 0.0]\n"
                "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n", encoding="utf-8")

    def finish(self, state, reason, **metrics):
        self.publish()
        self.cancel_plan()
        self.complete = True
        self.transition(state, reason)
        grid = self.snapshot(self.now())
        self.save_map(grid)
        result = {"status": state, "reason": reason, "sim_s": self.now(),
                  "first_lap_completed": self.survey_finished, "inspections": self.inspection_outcomes,
                  "inspection_schema": "numbered",
                  "course_rejoins": self.course_rejoins, **metrics}
        temporary = self.output/"result.json.tmp"
        temporary.write_text(json.dumps(result, indent=2, allow_nan=False)+"\n", encoding="utf-8")
        temporary.replace(self.output/"result.json")

    def close(self):
        if not self.complete:
            self.finish("interrupted", "launch_stopped_before_completion")
        self.samples_file.close()
        self.events_file.close()


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = ParkingMission()
    try:
        spin_until_stopped(node)
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
