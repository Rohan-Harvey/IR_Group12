#!/usr/bin/env python3
"""Evaluate a completed Gazebo recording against independent scene geometry."""

import argparse
from collections import Counter
import csv
import json
import math
from pathlib import Path

from parking_core import Footprint, inverse
from patrol_route import Point, transform, wrap
from scene_geometry import overlaps, rectangle
from scene_scenarios import obstacle_polygons


def pose(row, x, y, yaw):
    return Point(float(row[x]), float(row[y]), float(row[yaw]), 0)


def origin(point):
    return point.x, point.y, point.yaw


def point_segment_distance(point, a, b):
    dx, dy = b[0]-a[0], b[1]-a[1]
    scale = ((point[0]-a[0])*dx+(point[1]-a[1])*dy)/(dx*dx+dy*dy)
    scale = min(1.0, max(0.0, scale))
    return math.hypot(point[0]-a[0]-scale*dx, point[1]-a[1]-scale*dy)


def polygon_distance(a, b):
    if overlaps(a, b):
        return 0.0
    return min(point_segment_distance(p, x, y) for points, edges in ((a, b), (b, a))
               for p in points for x, y in zip(edges, edges[1:]+edges[:1]))


def steering_width(row, footprint):
    angles = [abs(float(row[key])) for key in ("steering_left", "steering_right") if row[key]]
    return footprint.parked_half_width if len(angles) == 2 and max(angles) <= 0.12 else footprint.half_width


def evaluate(run, project):
    truth = json.loads((project/"config/scene_ground_truth.json").read_text())
    result = json.loads((run/"result.json").read_text())
    events = [json.loads(line) for line in (run/"decisions.jsonl").read_text().splitlines()]
    with (run/"run.csv").open(newline="") as file:
        rows = list(csv.DictReader(file))
    samples = [(row, pose(row, "world_rear_x", "world_rear_y", "world_heading"))
               for row in rows if row["world_rear_x"] and row["world_rear_y"] and row["world_heading"]]
    if not samples:
        raise ValueError("The recording contains no independent Gazebo poses")
    comparisons = Counter(e["swept_path"] for e in events if e["event"] == "comparison")
    report = {"mission_status": result["status"], "samples_with_gazebo_pose": len(samples),
              "baseline_accepted_attempts": sum(comparisons.values()), "sweep_outcomes": dict(comparisons),
              "braking_guard_samples": sum(row["guard"] == "braking_clearance" for row in rows),
              "reported_chassis_contact": any(row["guard"] == "chassis_contact" for row in rows)
              or result["reason"] == "chassis_contact"}
    first_row, initial_world = samples[0]
    initial_odom = pose(first_row, "odom_x", "odom_y", "odom_heading")
    # Evaluation aligns odom to world both at startup and at the accepted
    # parking plan. The first exposes accumulated survey drift and the
    # second measures target error over the parking maneuver itself.
    def world_target(odom, odom_anchor, world_anchor):
        return transform(transform(odom, inverse(origin(odom_anchor))), origin(world_anchor))

    footprint = Footprint.from_robot(truth["robot"])
    center = (footprint.back+footprint.front)/2
    if "target_odom" in result:
        target_odom = Point(*result["target_odom"], 0.0)
        target = world_target(target_odom, initial_odom, initial_world)
        last_row, last = samples[-1]
        report["world_error_from_start_alignment_m"] = math.hypot(last.x-target.x, last.y-target.y)
        report["world_heading_error_from_start_alignment_deg"] = math.degrees(abs(wrap(last.yaw-target.yaw)))
        starts = [e["sim_s"] for e in events if e["event"] == "executing_parking"]
        if starts:
            row, park_start_world = min(samples, key=lambda sample: abs(float(sample[0]["sim_s"])-starts[-1]))
            park_start_odom = pose(row, "odom_x", "odom_y", "odom_heading")
            target = world_target(target_odom, park_start_odom, park_start_world)
        report["world_position_error_m"] = math.hypot(last.x-target.x, last.y-target.y)
        report["world_heading_error_deg"] = math.degrees(abs(wrap(last.yaw-target.yaw)))
        report["final_world_rear_pose"] = origin(last)
        body = (last.x+center*math.cos(last.yaw), last.y+center*math.sin(last.yaw))
        outline = rectangle(body, footprint.front-footprint.back, 2*steering_width(last_row, footprint), last.yaw)
        report["matched_reference_zones"] = []
        for case in truth.get("scenarios", []):
            x0, y0, x1, y1 = case["target_zone_xyxy_m"]
            goal_yaw = math.radians(case["goal_pose_m_deg"][2])
            if (all(x0 <= x <= x1 and y0 <= y <= y1 for x, y in outline)
                    and abs(wrap(last.yaw-goal_yaw)) <= math.radians(10)):
                report["matched_reference_zones"].append(case["id"])
    cars = [bay for bay in truth["bays"] if bay["occupied"]]
    obstacles = obstacle_polygons(cars)
    minimum, limiting_object = math.inf, None
    for row, world in samples:
        body = (world.x+center*math.cos(world.yaw), world.y+center*math.sin(world.yaw))
        # Use the straight-ahead envelope for standstill, and conservative
        # wheel envelope at larger measured steering angles.
        width = steering_width(row, footprint)
        shape = rectangle(body, footprint.front-footprint.back, 2*width, world.yaw)
        for name, outline in obstacles:
            clearance = polygon_distance(shape, outline)
            if clearance < minimum:
                minimum, limiting_object = clearance, name
    report["sampled_static_clearance_m"] = minimum if math.isfinite(minimum) else None
    report["limiting_static_object"] = limiting_object
    if "first_lap_completed" in result:
        lap = next((event for event in events if event["event"] == "lap_one_complete"), None)
        inspections = [event for event in events if event["event"] == "inspection"]
        stations = list(dict.fromkeys(event["station"] for event in inspections))
        if lap:
            _, returned = min(samples, key=lambda sample: abs(float(sample[0]["sim_s"])-lap["sim_s"]))
            home_error = math.hypot(returned.x-initial_world.x, returned.y-initial_world.y)
            home_heading_error = math.degrees(abs(wrap(returned.yaw-initial_world.yaw)))
        else:
            home_error = home_heading_error = None
        parking = [event["sim_s"] for event in events if event["event"] == "executing_parking"]
        report["demonstration"] = {
            "first_lap_completed": result["first_lap_completed"],
            "home_return_error_m": home_error,
            "home_return_heading_error_deg": home_heading_error,
            "first_lap_end_sim_s": lap["sim_s"] if lap else None,
            "inspection_order": stations,
            "inspections": result["inspections"],
            "red_motion_updates": max((event.get("tracking_updates", 0) for event in inspections
                                       if event["station"] == "red"), default=0),
            "all_parking_attempts_after_first_lap": bool(lap and parking
                                                         and all(t > lap["sim_s"] for t in parking)),
        }
        demonstration = report["demonstration"]
        outcomes = demonstration["inspections"]
        checks = {
            "returned_to_start": bool(result["first_lap_completed"] and home_error is not None
                                      and home_error <= 0.45 and home_heading_error <= 10),
            "saved_first_lap_map": all((run/f"first_lap_map.{ext}").is_file() for ext in ("pgm", "yaml")),
            "ordered_inspections": stations == ["white", "blue", "red", "yellow", "green"],
            "white_bypassed": outcomes.get("white") == "bypass",
            "blue_mouth_blocked": outcomes.get("blue") == "mouth_too_narrow",
            "red_motion_observed": outcomes.get("red") == "moving_obstacle_in_mouth",
            "yellow_rejected": outcomes.get("yellow") in {
                "no_clear_perpendicular_footprint", "entry_sweep_blocked_or_unobserved",
                "no_feasible_reverse_connection"},
            "parked_only_after_survey": demonstration["all_parking_attempts_after_first_lap"],
            "parked_inside_green": (result["status"] == "parked" and outcomes.get("green") == "parked"
                                    and report.get("matched_reference_zones") == ["S1"]),
            "recorded_clearance_without_contact": (not report["reported_chassis_contact"]
                                                   and minimum > 0 and math.isfinite(minimum)),
        }
        demonstration["checks"] = checks
        demonstration["passed"] = all(checks.values())
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path)
    parser.add_argument("--check-demonstration", action="store_true",
                        help="Require a complete, measured entrance demonstration")
    args = parser.parse_args()
    report = evaluate(args.run_directory, Path(__file__).resolve().parents[1])
    print(json.dumps(report, indent=2))
    if args.check_demonstration and not report.get("demonstration", {}).get("passed"):
        parser.exit(1, "Entrance demonstration did not pass every recorded check.\n")


if __name__ == "__main__":
    main()
