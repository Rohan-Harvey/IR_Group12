#!/usr/bin/env python3
"""Check XML structure and geometric scene invariants without ROS or Gazebo.

This does not replace libsdformat parsing or a Gazebo runtime smoke test.
"""

import argparse
import itertools
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from scene_geometry import overlaps, rectangle


ROOT = Path(__file__).resolve().parents[1]


def check(condition, message):
    if not condition:
        raise ValueError(message)


def inspect_xml(path, truth):
    root = ET.parse(path).getroot()
    check(root.tag == "sdf" and root.get("version") == "1.9", "Wrong SDF root/version")
    world = root.find("world")
    check(world is not None and world.get("name") == "parking_lot", "Missing world")
    check(not root.findall(".//include") and not root.findall(".//uri"), "External assets found")
    names = [m.get("name") for m in world.findall("model")]
    check(len(names) == len(set(names)), "Duplicate model names")
    plugins = {p.get("name") for p in world.findall("plugin")}
    check({f"gz::sim::systems::{x}" for x in ["Physics", "SceneBroadcaster", "UserCommands", "Sensors"]}
          <= plugins, "Missing required world systems")
    check("parking_markings" not in names, "Painted markings remain")
    check(not any(v.get("name", "").startswith(("arrow_", "A_line_", "B_line_", "C_line_", "D_line_"))
                  for v in world.findall(".//visual")), "Bay lines or road arrows remain")
    cars = [m for m in world.findall("model") if m.get("name", "").startswith("car_")]
    check(len(cars) == truth["parked_car_count"], "Car count differs from ground truth")
    occupied = {b["car"]["model_name"]: b["car"] for b in truth["bays"] if b["occupied"]}
    for model in world.findall("model"):
        if model.get("name") != "parking_robot":
            check(model.findtext("static") == "true", f"Dynamic scenery: {model.get('name')}")
        for link in model.findall("link"):
            for kind in ["visual", "collision"]:
                entries = link.findall(kind)
                check(len({x.get("name") for x in entries}) == len(entries),
                      f"Duplicate {kind} names in {model.get('name')}")
    for car in cars:
        expected = occupied[car.get("name")]
        pose = [float(x) for x in car.findtext("pose").split()]
        check(all(abs(pose[i] - expected["position_m"][i]) < 2e-6 for i in range(3))
              and abs(pose[5] - expected["yaw_rad"]) < 2e-6,
              f"Pose differs from manifest: {car.get('name')}")
        check(len(car.findall(".//collision")) >= 10, f"Missing collision parts: {car.get('name')}")
        for kind in ["visual", "collision"]:
            for item in car.findall(f".//{kind}"):
                geometry = item.find("geometry")
                check(geometry is not None, f"Missing geometry: {car.get('name')}")
    for element in root.iter():
        if element.tag in ["size", "radius", "length", "pose"] and element.text:
            values = [float(x) for x in element.text.split()]
            check(all(math.isfinite(x) for x in values), "Non-finite geometry")
            if element.tag != "pose":
                check(all(x > 0 for x in values), "Non-positive geometry")
    return world


def validate_robot(root, world, truth):
    robot = world.find("model[@name='parking_robot']")
    if truth["robot"] is None:
        check(robot is None, "Robot should have been disabled")
        return
    check(robot is not None and robot.findtext("static") == "false", "Missing dynamic example robot")
    drive = robot.find("plugin[@name='gz::sim::systems::AckermannSteering']")
    check(drive is not None, "Missing four-wheel drive system")
    driven = [v.text for v in drive.findall("left_joint") + drive.findall("right_joint")]
    check(len(driven) == len(set(driven)) == 4, "All four wheel joints must be powered")
    joints = {j.get("name"): j for j in robot.findall("joint")}
    urdf = ET.parse(root / "urdf" / "parking_robot.urdf").getroot()
    urdf_joints = {j.get("name") for j in urdf.findall("joint")}
    links = {link.get("name"): link for link in robot.findall("link")}
    for name in driven:
        check(name in joints and name in urdf_joints, "SDF/URDF wheel joint mismatch")
        check(joints[name].get("type") == "revolute", "Wheel joint must rotate")
        check(joints[name].findtext("axis/xyz") == "0 1 0", "Wheel axis must be transverse")
        child = joints[name].findtext("child")
        radius = float(links[child].findtext("collision/geometry/cylinder/radius"))
        check(abs(radius - float(drive.findtext("wheel_radius"))) < 1e-6, "Drive radius differs from geometry")
    for side in ("left","right"):
        name=drive.findtext(side+"_steering_joint")
        check(name in joints and name in urdf_joints,"Missing matching front steering joint")
        check(joints[name].findtext("axis/xyz")=="0 0 1","Steering axis must be vertical")
    check(abs(float(drive.findtext("wheel_base"))-2.75*truth["scale"])<1e-6,"Unexpected wheelbase")
    check(truth["robot"]["footprint_size_m"]==[4.62*truth["scale"],2.12*truth["scale"]],"Robot must match sedan footprint")
    for link in links.values():
        check(float(link.findtext("inertial/mass")) > 0, "Missing positive link mass")
        for diagonal in ("ixx", "iyy", "izz"):
            check(float(link.findtext("inertial/inertia/" + diagonal)) > 0, "Invalid link inertia")
    sensors = {sensor.get("type") for sensor in robot.findall(".//sensor")}
    check({"gpu_lidar", "camera", "imu", "contact"} <= sensors, "Missing robot sensors")
    lidar = robot.find(".//sensor[@type='gpu_lidar']")
    check(int(lidar.findtext("lidar/scan/horizontal/samples")) == 720, "Unexpected lidar sample count")
    check(lidar.findtext("gz_frame_id") == "lidar_link", "Lidar TF frame mismatch")
    check(robot.find(".//sensor[@type='camera']").findtext("gz_frame_id") == "camera_optical_frame", "Camera TF mismatch")
    expected_links = {"base_footprint", "base_link", "lidar_link", "camera_link", "camera_optical_frame", "imu_link"}
    check(expected_links <= {link.get("name") for link in urdf.findall("link")}, "Missing TF frames")


def validate_people(root, world, truth, car_polygons):
    actors = world.findall("actor")
    check(len(actors) == truth["pedestrian_count"], "Wrong pedestrian count")
    namespace = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
    checked = set()
    for actor in actors:
        check(actor.findtext("script/loop") == "true" and actor.findtext("script/auto_start") == "true",
              "Pedestrian must start and loop")
        points = actor.findall("script/trajectory/waypoint")
        times = [float(p.findtext("time")) for p in points]
        check(len(points) >= 4 and all(b > a for a,b in zip(times,times[1:])), "Invalid walking trajectory")
        poses = [[float(x) for x in p.findtext("pose").split()] for p in points]
        check(poses[0] == poses[-1], "Walking loop must return to its original pose")
        initial = [float(x) for x in actor.findtext("pose").split()]
        # Sample every straight segment including turns against parked cars.
        for a,b in zip(poses,poses[1:]):
            for i in range(21):
                xy = [initial[k] + a[k] + (b[k]-a[k])*i/20 for k in range(2)]
                person = rectangle(xy, 1.0*truth["scale"], 1.0*truth["scale"], 0)
                check(all(not overlaps(person,p) for _,p in car_polygons), "Pedestrian route crosses a parked car")
        uri = actor.findtext("skin/filename")
        check(uri.startswith("model://walking_person/"), "Pedestrian mesh should be bundled locally")
        path = root / "models" / uri.removeprefix("model://")
        check(path.is_file(), f"Missing local person model: {path}")
        check(actor.findtext("animation/filename") == uri, "Skin and skeleton should share a rig")
        if uri in checked:
            continue
        checked.add(uri)
        mesh = ET.parse(path).getroot()
        ids = [e.get("id") for e in mesh.iter() if e.get("id")]
        check(len(ids) == len(set(ids)), "Duplicate COLLADA IDs")
        for e in mesh.iter():
            for attr in ("source", "url", "target"):
                ref = e.get(attr, "")
                if ref.startswith("#"):
                    check(ref[1:] in ids, "Dangling COLLADA asset reference")
        for array in mesh.findall(".//c:float_array", namespace) + mesh.findall(".//c:Name_array", namespace):
            check(len(array.text.split()) == int(array.get("count")), "COLLADA array count mismatch")
        for matrix in mesh.findall(".//c:matrix", namespace):
            numbers = [float(v) for v in matrix.text.split()]
            check(len(numbers) == 16 and numbers[12:] == [0,0,0,1], "Invalid COLLADA affine matrix")
        weights = mesh.find(".//c:vertex_weights", namespace)
        check(weights is not None and int(weights.get("count")) == len(weights.find("c:vcount",namespace).text.split()),
              "Missing skin weights")
        animations = mesh.findall(".//c:library_animations/c:animation",namespace)
        check(len(animations) >= 8, "Missing articulated walking animation")
        root_motion = mesh.find(".//c:source[@id='Root_transforms']/c:float_array",namespace)
        values = [float(v) for v in root_motion.text.split()]
        check(values[-13] > values[3], "Walk animation needs forward root displacement")


def validate(root, strict_bays=False):
    truth = json.loads((root / "config" / "scene_ground_truth.json").read_text())
    primary = inspect_xml(root / "worlds" / "parking_lot.sdf", truth)
    unmarked = inspect_xml(root / "worlds" / "parking_lot_unmarked.sdf", truth)
    # Strip formatting whitespace before comparing the XML trees.
    for node in itertools.chain(primary.iter(), unmarked.iter()):
        if node.text and not node.text.strip():
            node.text = None
        node.tail = None
    check(ET.tostring(primary) == ET.tostring(unmarked), "Unmarked world filename aliases differ")
    check(truth["painted_markings"] is False, "Ground truth should describe an unmarked world")
    check(len(truth["bays"]) == truth["bay_count"], "Wrong bay count")
    check(sum(b["occupied"] for b in truth["bays"]) == truth["parked_car_count"], "Wrong occupied count")
    polygons = []
    padded_polygons = []
    scale = truth["scale"]
    clearance = truth["minimum_car_clearance_m"]
    departures = []
    for bay in truth["bays"]:
        if not bay["occupied"]:
            continue
        car = bay["car"]
        polygon = rectangle(car["position_m"], car["envelope_m"][0],
                            car["envelope_m"][1], car["yaw_rad"])
        polygons.append((bay["id"], polygon))
        padded_polygons.append((bay["id"], rectangle(car["position_m"],
                               car["envelope_m"][0] + clearance,
                               car["envelope_m"][1] + clearance, car["yaw_rad"])))
        if any(abs(x - bay["center_m"][0]) > bay["bay_size_m"][0] / 2
               or abs(y - bay["center_m"][1]) > bay["bay_size_m"][1] / 2 for x, y in polygon):
            departures.append(bay["id"])
        # The conservative envelope includes mirrors. Check curbs and median.
        check(all(0.60 * scale < abs(y) < 17.395 * scale for _, y in polygon),
              f"Car {bay['id']} crosses a curb/median boundary")
    for (a_name, a), (b_name, b) in itertools.combinations(polygons, 2):
        check(not overlaps(a, b), f"Overlapping parked-car envelopes: {a_name}, {b_name}")
    for (a_name, a), (b_name, b) in itertools.combinations(padded_polygons, 2):
        check(not overlaps(a, b), f"Insufficient car clearance: {a_name}, {b_name}")
    validate_robot(root, primary, truth)
    validate_people(root, primary, truth, polygons)
    spawn = truth["suggested_robot_spawn"]["position_m"]
    # Verify a 4.8 x 2.2 m car-sized footprint fits at the suggested start.
    robot = rectangle(spawn, 4.8 * scale, 2.2 * scale, 0)
    check(all(not overlaps(robot, p) for _, p in polygons), "Suggested robot start overlaps a car")
    check(all(-23.99 * scale < x < 23.99 * scale and -12.19 * scale < y < -5.81 * scale
              for x, y in robot), "Suggested robot start is outside the clear approach")
    if strict_bays:
        check(not departures, f"Cars outside their own bays: {departures}")
    print(f"PASS: 2 identical unmarked XML world aliases; {truth['parked_car_count']} static cars; "
          "no painted lines/arrows; no overlapping car envelopes; minimum spacing and clear start verified.")
    print(f"PASS: robot definition, driven wheel joints, sensors, TF; {truth['pedestrian_count']} pedestrian paths and bundled rigs.")
    if departures:
        print(f"Irregular gaps: {len(departures)} cars extend beyond their unpainted reference slot.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", nargs="?", type=Path, default=ROOT)
    parser.add_argument("--strict-bays", action="store_true")
    args = parser.parse_args()
    try:
        validate(args.root, args.strict_bays)
    except (ValueError, KeyError, ET.ParseError) as exc:
        parser.exit(1, f"FAIL: {exc}\n")


if __name__ == "__main__":
    main()
