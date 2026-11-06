#!/usr/bin/env python3
"""Generate an unmarked Gazebo lot, a sensor robot and animated pedestrians.

The JSON contains design parameters. scene_ground_truth.json is for evaluation,
not an input to a perception, mapping or parking-space detection algorithm.
"""

import argparse
import copy
import json
import math
from pathlib import Path
import random
import xml.etree.ElementTree as ET

from scene_geometry import overlaps, rectangle
from scene_robot import add_robot, write_urdf
from scene_people import ROUTES, add_people, write_people_assets
import scene_scenarios as scenarios


ROOT = Path(__file__).resolve().parents[1]
PALETTE = [
    ("red", "0.66 0.08 0.07 1"),
    ("blue", "0.06 0.22 0.55 1"),
    ("silver", "0.63 0.68 0.72 1"),
    ("white", "0.88 0.90 0.88 1"),
    ("charcoal", "0.09 0.12 0.15 1"),
    ("ochre", "0.82 0.48 0.09 1"),
    ("green", "0.12 0.36 0.25 1"),
    ("teal", "0.05 0.40 0.47 1"),
]
VEHICLES = {
    "compact": (3.90, 1.72, 1.48),
    "sedan": (4.50, 1.84, 1.52),
    "suv": (4.70, 1.96, 1.80),
    "van": (4.80, 1.90, 1.96),
}
GLASS = "0.055 0.15 0.20 1"
DARK = "0.035 0.045 0.05 1"
CONCRETE = "0.66 0.66 0.62 1"


def add(parent, tag, value=None, **attrs):
    node = ET.SubElement(parent, tag, attrs)
    if value is not None:
        node.text = str(value)
    return node


def numbers(values):
    return " ".join(f"{x:.7f}" for x in values)


class Builder:
    def __init__(self, scale):
        self.scale = scale
        self.root = ET.Element("sdf", version="1.9")
        self.world = add(self.root, "world", name="parking_lot")
        add(self.world, "gravity", "0 0 -9.81")
        physics = add(self.world, "physics", name="1ms", type="ignored")
        add(physics, "max_step_size", "0.001")
        add(physics, "real_time_factor", "1.0")
        for filename, name in [
            ("physics", "Physics"),
            ("user-commands", "UserCommands"),
            ("scene-broadcaster", "SceneBroadcaster"),
            ("sensors", "Sensors"),
            ("imu", "Imu"),
            ("contact", "Contact"),
        ]:
            plugin = add(self.world, "plugin", filename=f"gz-sim-{filename}-system",
                         name=f"gz::sim::systems::{name}")
            if name == "Sensors":
                add(plugin, "render_engine", "ogre2")
        scene = add(self.world, "scene")
        add(scene, "ambient", "0.60 0.62 0.66 1")
        add(scene, "background", "0.76 0.84 0.91 1")
        add(scene, "shadows", "true")
        light = add(self.world, "light", name="sun", type="directional")
        add(light, "pose", self.pose((0, 0, 30)))
        add(light, "cast_shadows", "true")
        add(light, "diffuse", "0.85 0.83 0.77 1")
        add(light, "specular", "0.25 0.25 0.25 1")
        add(light, "direction", "-0.4 0.2 -0.9")
        attenuation = add(light, "attenuation")
        add(attenuation, "range", 1000 * scale)
        add(attenuation, "constant", 0.9)
        add(attenuation, "linear", 0.01)
        add(attenuation, "quadratic", 0.001)
        self.gui()

    def pose(self, position, rotation=(0, 0, 0)):
        return numbers([v * self.scale for v in position] + list(rotation))

    def gui(self):
        gui = add(self.world, "gui", fullscreen="false")
        view = add(gui, "plugin", filename="MinimalScene", name="3D View")
        settings = add(view, "gz-gui")
        add(settings, "title", "3D View")
        add(settings, "property", "false", key="showTitleBar", type="bool")
        add(settings, "property", "docked", key="state", type="string")
        add(view, "engine", "ogre2")
        add(view, "scene", "scene")
        add(view, "ambient_light", "0.60 0.62 0.66")
        add(view, "background_color", "0.76 0.84 0.91")
        add(view, "camera_pose", self.pose((-42, -40, 48), (0, 0.692, 0.761)))
        clip = add(view, "camera_clip")
        add(clip, "near", 0.1 * self.scale)
        add(clip, "far", 500 * self.scale)
        for filename, name in [
            ("GzSceneManager", "Scene Manager"),
            ("InteractiveViewControl", "Interactive view control"),
            ("CameraTracking", "Camera Tracking"),
            ("EntityContextMenuPlugin", "Entity context menu"),
            ("SelectEntities", "Select entities"),
            ("TransformControl", "Transform control"),
        ]:
            plugin = add(gui, "plugin", filename=filename, name=name)
            properties = add(plugin, "gz-gui")
            add(properties, "property", "false", key="showTitleBar", type="bool")
            add(properties, "property", "floating", key="state", type="string")
        for filename, width, height, side in [
            ("WorldControl", 140, 72, "left"),
            ("WorldStats", 300, 110, "right"),
        ]:
            plugin = add(gui, "plugin", filename=filename, name=filename)
            properties = add(plugin, "gz-gui")
            add(properties, "property", "false", key="showTitleBar", type="bool")
            add(properties, "property", "false", key="resizable", type="bool")
            add(properties, "property", width, key="width", type="double")
            add(properties, "property", height, key="height", type="double")
            add(properties, "property", "floating", key="state", type="string")
            anchors = add(properties, "anchors", target="3D View")
            add(anchors, "line", own=side, target=side)
            add(anchors, "line", own="bottom", target="bottom")
            if filename == "WorldControl":
                add(plugin, "play_pause", "true")
                add(plugin, "step", "true")
                add(plugin, "start_paused", "false")
                add(plugin, "service", "/world/parking_lot/control")
                add(plugin, "stats_topic", "/world/parking_lot/stats")
            else:
                for tag in ["sim_time", "real_time", "real_time_factor", "iterations"]:
                    add(plugin, tag, "true")
                add(plugin, "topic", "/world/parking_lot/stats")
        add(gui, "plugin", filename="EntityTree", name="Entity tree")

    def model(self, name, position=(0, 0, 0), yaw=0):
        model = add(self.world, "model", name=name)
        add(model, "static", "true")
        add(model, "pose", self.pose(position, (0, 0, yaw)))
        return add(model, "link", name="body")

    def shape(self, link, name, geometry, position, color, collision=True,
              rotation=(0, 0, 0), shadows=True, emissive=None):
        visual = add(link, "visual", name=name)
        add(visual, "pose", self.pose(position, rotation))
        visual.append(copy.deepcopy(geometry))
        add(visual, "cast_shadows", str(shadows).lower())
        material = add(visual, "material")
        add(material, "ambient", color)
        add(material, "diffuse", color)
        add(material, "specular", "0.18 0.18 0.18 1")
        if emissive:
            add(material, "emissive", emissive)
        if collision:
            body = add(link, "collision", name=name + "_collision")
            add(body, "pose", self.pose(position, rotation))
            body.append(copy.deepcopy(geometry))

    def box(self, link, name, size, position, color, **kwargs):
        geometry = ET.Element("geometry")
        add(add(geometry, "box"), "size", numbers([x * self.scale for x in size]))
        self.shape(link, name, geometry, position, color, **kwargs)

    def cylinder(self, link, name, radius, length, position, color, **kwargs):
        geometry = ET.Element("geometry")
        cylinder = add(geometry, "cylinder")
        add(cylinder, "radius", radius * self.scale)
        add(cylinder, "length", length * self.scale)
        self.shape(link, name, geometry, position, color, **kwargs)

    def sphere(self, link, name, radius, position, color, **kwargs):
        geometry = ET.Element("geometry")
        add(add(geometry, "sphere"), "radius", radius * self.scale)
        self.shape(link, name, geometry, position, color, **kwargs)


def make_bays(config):
    rng = random.Random(config["seed"])
    bays = []
    for row in config["rows"]:
        for column in range(config["columns"]):
            bay_id = f"{row['id']}{column + 1:02d}"
            x = (column - (config["columns"] - 1) / 2) * config["bay_width_m"]
            # Draw for every bay, including empty ones, so changing occupancy does
            # not change the shapes, colors or poses of all subsequent vehicles.
            vehicle_type = rng.choice(["compact", "sedan", "sedan", "suv", "van"])
            color_name, color = rng.choice(PALETTE)
            jitter_x = rng.uniform(-0.055, 0.055)
            jitter_y = rng.uniform(-0.045, 0.045)
            yaw = row["yaw_deg"] + rng.uniform(-1.4, 1.4)
            yaw += config.get("angle_offsets_deg", {}).get(bay_id, 0.0)
            length, width, height = VEHICLES[vehicle_type]
            bays.append({
                "id": bay_id, "row": row["id"], "occupied": bay_id not in config["empty_bays"],
                "center_m": [x, row["y_m"]], "bay_size_m": [config["bay_width_m"], config["bay_depth_m"]],
                "car": {"model_name": f"car_{bay_id}", "type": vehicle_type,
                        "color_name": color_name, "rgba": color,
                        "position_m": [x + jitter_x, row["y_m"] + jitter_y, 0.0],
                        "yaw_rad": math.radians(yaw), "body_size_m": [length, width, height],
                        "envelope_m": [length + 0.12, width + 0.28, height + 0.015]},
            })
    randomize_car_poses(config, bays)
    return bays


def randomize_car_poses(config, bays):
    """Rejection-sample car poses while keeping a collision-free scene throughout.

    Start with parallel tilted cars, then change individual cars against all
    other current footprints. Small position changes keep rotated cars off the
    curbs. The empty reference slots are unpainted and may have irregular edges.
    """
    maximum = config.get("random_angle_range_deg", 35.0)
    clearance = config.get("minimum_car_clearance_m", 0.08)
    if maximum == 0:
        return
    occupied = [bay for bay in bays if bay["occupied"]]
    rng = random.Random(f"{config['seed']}:random-angles")
    row_yaws = {row["id"]: row["yaw_deg"] for row in config["rows"]}
    original = {bay["id"]: copy.deepcopy(bay["car"]) for bay in occupied}

    def footprint(car):
        return rectangle(car["position_m"], car["envelope_m"][0] + clearance,
                         car["envelope_m"][1] + clearance, car["yaw_rad"])

    def candidate(bay, offset, jitter=False):
        car = copy.deepcopy(original[bay["id"]])
        angle = row_yaws[bay["row"]] + offset
        angle += config.get("angle_offsets_deg", {}).get(bay["id"], 0.0)
        car["yaw_rad"] = math.radians(angle)
        if jitter:
            car["position_m"][0] = bay["center_m"][0] + rng.uniform(-0.13, 0.13)
            car["position_m"][1] = bay["center_m"][1] + rng.uniform(-0.18, 0.18)
        half_y = (abs(math.sin(car["yaw_rad"])) * (car["envelope_m"][0] + clearance)
                  + abs(math.cos(car["yaw_rad"])) * (car["envelope_m"][1] + clearance)) / 2
        side = math.copysign(1, bay["center_m"][1])
        # Keep the complete padded envelope between median and outer sidewalk.
        distance = abs(car["position_m"][1])
        distance = max(0.605 + half_y, min(distance, 17.390 - half_y))
        car["position_m"][1] = side * distance
        return car

    warm_offsets = {row["id"]: rng.choice([-1, 1]) * min(12.0, maximum)
                    for row in config["rows"]}
    for bay in occupied:
        bay["car"] = candidate(bay, warm_offsets[bay["row"]])
    polygons = {bay["id"]: footprint(bay["car"]) for bay in occupied}
    if any(overlaps(polygons[a["id"]], polygons[b["id"]])
           for i, a in enumerate(occupied) for b in occupied[i + 1:]):
        # A custom manual offset can invalidate the warm start. Fall back to
        # the original scene and report impossible inputs instead of overlaps.
        for bay in occupied:
            bay["car"] = original[bay["id"]]
        polygons = {bay["id"]: footprint(bay["car"]) for bay in occupied}
        if any(overlaps(polygons[a["id"]], polygons[b["id"]])
               for i, a in enumerate(occupied) for b in occupied[i + 1:]):
            raise ValueError("Manual angle offsets overlap cars; reduce the offsets or empty adjacent slots")
    # Later passes allow neighboring cars to settle into more varied angles.
    for _ in range(3):
        order = list(occupied)
        rng.shuffle(order)
        for bay in order:
            for _attempt in range(100):
                angle = rng.uniform(-maximum, maximum)
                if abs(angle) < min(5.0, maximum / 2):
                    continue
                car = candidate(bay, angle, jitter=True)
                polygon = footprint(car)
                if all(other_id == bay["id"] or not overlaps(polygon, other)
                       for other_id, other in polygons.items()):
                    bay["car"] = car
                    polygons[bay["id"]] = polygon
                    break


def add_car(builder, bay):
    car = bay["car"]
    link = builder.model(car["model_name"], car["position_m"], car["yaw_rad"])
    length, width, height = car["body_size_m"]
    color = car["rgba"]
    body_top = height * 0.61
    builder.box(link, "body", (length, width, body_top - 0.20),
                (0, 0, (body_top + 0.20) / 2), color)
    builder.box(link, "hood", (0.28 * length, 0.95 * width, 0.07),
                (0.35 * length, 0, body_top + 0.02), color, collision=False)
    cabin_height = height - body_top + 0.04
    cabin_z = (height + body_top - 0.04) / 2
    builder.box(link, "cabin", (0.52 * length, 0.82 * width, cabin_height),
                (-0.06 * length, 0, cabin_z), color)
    builder.box(link, "roof", (0.54 * length, 0.86 * width, 0.07),
                (-0.06 * length, 0, height - 0.02), color, collision=False)
    builder.box(link, "windshield", (0.035, 0.77 * width, 0.72 * cabin_height),
                (0.205 * length, 0, cabin_z + 0.015), GLASS,
                collision=False, rotation=(0, -0.38, 0))
    builder.box(link, "rear_window", (0.035, 0.77 * width, 0.68 * cabin_height),
                (-0.325 * length, 0, cabin_z + 0.015), GLASS,
                collision=False, rotation=(0, 0.38, 0))
    for side in [-1, 1]:
        suffix = "left" if side == 1 else "right"
        builder.box(link, f"side_windows_{suffix}", (0.45 * length, 0.025, 0.65 * cabin_height),
                    (-0.06 * length, side * (0.41 * width + 0.014), cabin_z + 0.025),
                    GLASS, collision=False)
        builder.box(link, f"pillar_{suffix}", (0.085, 0.035, 0.70 * cabin_height),
                    (-0.045 * length, side * (0.41 * width + 0.019), cabin_z + 0.02),
                    color, collision=False)
        builder.box(link, f"mirror_{suffix}", (0.22, 0.14, 0.14),
                    (0.16 * length, side * (width / 2 + 0.07), body_top + 0.10), color)
        for front in [-1, 1]:
            wheel_name = f"wheel_{suffix}_{'front' if front == 1 else 'rear'}"
            wheel_position = (front * 0.31 * length, side * (width / 2 - 0.055), 0.32)
            builder.cylinder(link, wheel_name, 0.32, 0.20, wheel_position, DARK,
                             rotation=(math.pi / 2, 0, 0))
            builder.cylinder(link, wheel_name + "_rim", 0.18, 0.012,
                             (wheel_position[0], side * (width / 2 + 0.052), 0.32),
                             "0.52 0.56 0.60 1", collision=False,
                             rotation=(math.pi / 2, 0, 0))
        for front, light_color in [(1, "0.95 0.90 0.66 1"), (-1, "0.65 0.025 0.02 1")]:
            builder.box(link, f"light_{suffix}_{front}", (0.035, width * 0.21, 0.16),
                        (front * (length / 2 + 0.02), side * width * 0.30, body_top - 0.12),
                        light_color, collision=False)
    for front in [-1, 1]:
        builder.box(link, f"bumper_{front}", (0.09, width * 0.91, 0.16),
                    (front * (length / 2 + 0.015), 0, 0.36), DARK)


def add_site(builder):
    link = builder.model("ground_and_asphalt")
    builder.box(link, "ground", (76, 60, 0.40), (-3, 0, -0.26), "0.28 0.42 0.24 1")
    builder.box(link, "asphalt", (48, 40, 0.10), (0, 0, -0.05), "0.19 0.21 0.23 1")
    builder.box(link, "entrance_road", (12, 6.4, 0.10), (-30, -9, -0.05), "0.19 0.21 0.23 1")
    for sign in [-1, 1]:
        link = builder.model(f"sidewalk_{sign}")
        builder.box(link, "paving", (43.0, 1.6, 0.12), (0, sign * 18.3, 0.06), CONCRETE)
        builder.box(link, "curb", (43.0, 0.15, 0.20), (0, sign * 17.47, 0.10), "0.79 0.78 0.71 1")
    link = builder.model("central_island")
    builder.box(link, "curb", (33.6, 1.2, 0.18), (0, 0, 0.09), CONCRETE)
    builder.box(link, "grass", (33.1, 0.84, 0.05), (0, 0, 0.19), "0.25 0.43 0.19 1")
    for x in [-14, -8, -2, 4, 10, 15]:
        builder.sphere(link, f"shrub_{x}", 0.38, (x, 0, 0.48), "0.15 0.32 0.15 1")
    link = builder.model("perimeter_walls")
    for name, size, position in [
        ("north", (48.6, 0.30, 0.50), (0, 20.15, 0.25)),
        ("south", (48.6, 0.30, 0.50), (0, -20.15, 0.25)),
        ("east", (0.30, 40, 0.50), (24.15, 0, 0.25)),
        ("west_north", (0.30, 25.8, 0.50), (-24.15, 7.1, 0.25)),
        ("west_south", (0.30, 7.8, 0.50), (-24.15, -16.1, 0.25)),
    ]:
        builder.box(link, name, size, position, "0.55 0.54 0.49 1")
    for index, x in enumerate([-12, 0, 12]):
        link = builder.model(f"lamp_{index}", (x, 0, 0))
        builder.cylinder(link, "base", 0.22, 0.30, (0, 0, 0.27), DARK)
        builder.cylinder(link, "pole", 0.075, 5.6, (0, 0, 3.0), DARK)
        builder.box(link, "arm", (0.10, 2.8, 0.10), (0, 0, 5.78), DARK)
        for side in [-1, 1]:
            builder.box(link, f"head_{side}", (0.48, 0.80, 0.12), (0, side * 1.2, 5.75), DARK)
            builder.box(link, f"lens_{side}", (0.38, 0.65, 0.018), (0, side * 1.2, 5.68),
                        "0.88 0.84 0.66 1", collision=False, emissive="0.40 0.35 0.20 1")
    for index, (x, y) in enumerate([(-20, 23), (-9, 23), (5, 23), (20, 23),
                                   (-20, -23), (-7, -23), (8, -23), (21, -23)]):
        link = builder.model(f"tree_{index}", (x, y, 0))
        builder.cylinder(link, "trunk", 0.19, 2.7, (0, 0, 1.35), "0.28 0.19 0.11 1")
        builder.sphere(link, "canopy", 1.65, (0, 0, 3.35), "0.13 0.32 0.16 1")
        builder.sphere(link, "crown", 1.12, (0.30, 0, 4.4), "0.19 0.39 0.18 1")


def ground_truth(config, bays,routes=None):
    scale = config["scale"]
    routes=ROUTES if routes is None else routes
    truth = {
        "purpose": "Simulator ground truth for evaluation ONLY; not a perception or navigation input.",
        "world_name": "parking_lot", "units": "metres and radians", "scale": scale,
        "seed": config["seed"], "bay_count": len(bays),
        "parked_car_count": sum(bay["occupied"] for bay in bays),
        "asphalt_size_m": [48 * scale, 40 * scale],
        "nominal_aisle_width_m": 6.4 * scale,
        "painted_markings": False,
        "random_angle_range_deg": config.get("random_angle_range_deg", 35.0),
        "minimum_car_clearance_m": config.get("minimum_car_clearance_m", 0.08) * scale,
        "robot": ({"name": "parking_robot", "drive": "four powered wheels, front Ackermann steering",
                   "footprint_size_m": [4.62 * scale, 2.12 * scale],
                   "wheelbase_m":2.75*scale,"steering_limit_rad":0.55,
                   "sensors": ["360 degree GPU LiDAR", "RGB camera", "IMU", "front contact bumper"],
                   "wheel_radius_m": 0.32 * scale} if config.get("include_robot", True) else None),
        "pedestrian_count": len(routes) if config.get("include_pedestrians", True) else 0,
        "pedestrians": [dict(route, start=[v * scale for v in route["start"]],
                             end=[v * scale for v in route["end"]], speed=route["speed"] * scale,
                             ground_z_m=route.get("ground_z_m",0) * scale)
                        for route in routes] if config.get("include_pedestrians", True) else [],
        "suggested_robot_spawn": {
            "position_m": [config["suggested_robot_spawn"][axis + "_m"] * scale for axis in "xyz"],
            "yaw_rad": math.radians(config["suggested_robot_spawn"]["yaw_deg"]),
        },
        "bays": copy.deepcopy(bays),
    }
    for bay in truth["bays"]:
        for key in ["center_m", "bay_size_m"]:
            bay[key] = [round(x * scale, 7) for x in bay[key]]
        if bay["occupied"]:
            for key in ["position_m", "body_size_m", "envelope_m"]:
                bay["car"][key] = [round(x * scale, 7) for x in bay["car"][key]]
        else:
            bay.pop("car")
    return truth


def validate_config(config):
    if config.get("layout_mode","scenarios") not in ["scenarios","random"]:
        raise ValueError("layout_mode must be scenarios or random")
    if not math.isfinite(config["scale"]) or config["scale"] <= 0:
        raise ValueError("scale must be a finite positive number")
    angle_range = config.get("random_angle_range_deg", 35.0)
    if not math.isfinite(angle_range) or not 0 <= angle_range <= 60:
        raise ValueError("random_angle_range_deg must be between 0 and 60")
    clearance = config.get("minimum_car_clearance_m", 0.08)
    if not math.isfinite(clearance) or not 0 <= clearance <= 0.15:
        raise ValueError("minimum_car_clearance_m must be between 0 and 0.15")
    # Site geometry is designed around this layout. Scale is the supported way
    # to resize it; occupancy, seed and vehicle angles can be changed freely.
    expected = [("A", 14.8, 90), ("B", 3.2, -90), ("C", -3.2, 90), ("D", -14.8, -90)]
    rows = [(r["id"], r["y_m"], r["yaw_deg"]) for r in config["rows"]]
    if (config["columns"] != 12 or config["bay_width_m"] != 2.8
            or config["bay_depth_m"] != 5.2 or rows != expected):
        raise ValueError("Keep the supplied row/bay geometry; use scale to resize the whole lot")
    ids = {f"{row['id']}{i:02d}" for row in config["rows"] for i in range(1, 13)}
    unknown = (set(config["empty_bays"]) | set(config.get("angle_offsets_deg", {}))) - ids
    if unknown:
        raise ValueError(f"Unknown bay identifiers: {sorted(unknown)}")
    if len(set(config["empty_bays"])) != len(config["empty_bays"]):
        raise ValueError("empty_bays contains duplicates")
    if any(not math.isfinite(v) for v in config.get("angle_offsets_deg", {}).values()):
        raise ValueError("angle offsets must be finite")


def generate(config, output_dir):
    validate_config(config)
    controlled=config.get("layout_mode","scenarios")=="scenarios"
    bays=scenarios.make_layout(config) if controlled else make_bays(config)
    routes=scenarios.people_routes() if controlled else ROUTES
    output_dir = Path(output_dir)
    (output_dir / "worlds").mkdir(parents=True, exist_ok=True)
    (output_dir / "config").mkdir(parents=True, exist_ok=True)
    builder = Builder(config["scale"])
    add_site(builder)
    if controlled:
        scenarios.add_layout_obstacles(builder)
    for bay in bays:
        if bay["occupied"]:
            add_car(builder, bay)
    if config.get("include_robot", True):
        add_robot(builder, config)
    add_people(builder, config.get("include_pedestrians", True),routes)
    write_people_assets(output_dir)
    write_urdf(output_dir / "urdf" / "parking_robot.urdf", config["scale"])
    ET.indent(builder.root, space="  ")
    # Keep both existing entry points; both now open the same unmarked world.
    for filename in ["parking_lot.sdf", "parking_lot_unmarked.sdf"]:
        ET.ElementTree(builder.root).write(output_dir / "worlds" / filename,
                                          encoding="utf-8", xml_declaration=True)
    truth=ground_truth(config,bays,routes)
    truth["layout_mode"]=config.get("layout_mode","scenarios")
    if controlled:
        manifest=scenarios.make_manifest(config,bays)
        truth["scenarios"]=manifest["cases"]
        (output_dir/"config"/"parking_scenarios.json").write_text(json.dumps(manifest,indent=2)+"\n")
        scenarios.write_reference_paths(output_dir,config)
        for case in scenarios.definitions():
            case_root=copy.deepcopy(builder.root)
            robot=case_root.find("world/model[@name='parking_robot']")
            if robot is not None:
                x,y,yaw=case["spawn"]
                robot.find("pose").text=builder.pose((x,y,0.03),(0,0,math.radians(yaw)))
            ET.ElementTree(case_root).write(output_dir/"worlds"/f"case_{case['id'].lower()}.sdf",
                                            encoding="utf-8",xml_declaration=True)
    (output_dir / "config" / "scene_ground_truth.json").write_text(
        json.dumps(truth, indent=2) + "\n", encoding="utf-8")
    (output_dir / "config" / "parking_lot.json").write_text(
        json.dumps(config, indent=2) + "\n", encoding="utf-8")
    print(f"Created unmarked {config.get('layout_mode','scenarios')} world: {sum(b['occupied'] for b in bays)} cars, "
          f"scale={config['scale']}; "
          f"robot={config.get('include_robot',True)}, pedestrians={config.get('include_pedestrians',True)}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "config" / "parking_lot.json")
    parser.add_argument("--output-dir", type=Path, default=ROOT)
    parser.add_argument("--seed", type=int)
    parser.add_argument("--scale", type=float)
    parser.add_argument("--layout",choices=["scenarios","random"],help="Controlled test cases or the previous random layout")
    parser.add_argument("--no-pedestrians", action="store_true", help="Disable walking actors for static mapping trials")
    parser.add_argument("--no-robot", action="store_true", help="Leave out the example robot")
    parser.add_argument("--max-angle", type=float,
                        help="Random rotation range in degrees either side of each row heading (default 35)")
    parser.add_argument("--empty", help="Replace empty bays with a comma-separated list; '' fills all bays")
    parser.add_argument("--angle", action="append", default=[], metavar="BAY:DEGREES",
                        help="Extra rotation, e.g. --angle B06:8; repeat for multiple cars")
    args = parser.parse_args()
    config = json.loads(args.config.read_text(encoding="utf-8"))
    if args.seed is not None:
        config["seed"] = args.seed
    if args.scale is not None:
        config["scale"] = args.scale
    if args.layout is not None:
        config["layout_mode"]=args.layout
    if args.no_pedestrians:
        config["include_pedestrians"] = False
    if args.no_robot:
        config["include_robot"] = False
    if args.max_angle is not None:
        config["random_angle_range_deg"] = args.max_angle
    if args.empty is not None:
        config["empty_bays"] = [x.strip().upper() for x in args.empty.split(",") if x.strip()]
    try:
        for entry in args.angle:
            bay_id, angle = entry.split(":", 1)
            config.setdefault("angle_offsets_deg", {})[bay_id.upper()] = float(angle)
        generate(config, args.output_dir)
    except (ValueError, KeyError, TypeError) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    main()
