"""Car-sized four-wheel-drive robot with front Ackermann steering and sensors."""

import math
import xml.etree.ElementTree as ET

FOOTPRINT_LENGTH=4.62
FOOTPRINT_WIDTH=2.12
WHEELBASE=2.75
TRACK=1.58
WHEEL_RADIUS=0.32


def sub(parent, tag, value=None, **attrs):
    element = ET.SubElement(parent, tag, attrs)
    if value is not None:
        element.text = str(value)
    return element


def inertia(link, mass, diagonal):
    inertial = sub(link, "inertial")
    sub(inertial, "mass", mass)
    tensor = sub(inertial, "inertia")
    for name, value in zip(("ixx", "iyy", "izz"), diagonal):
        sub(tensor, name, value)
    for name in ("ixy", "ixz", "iyz"):
        sub(tensor, name, 0)


def add_robot(builder, config):
    s = builder.scale
    spawn = config["suggested_robot_spawn"]
    model = sub(builder.world, "model", name="parking_robot", canonical_link="base_link")
    sub(model, "static", "false")
    sub(model, "self_collide", "false")
    sub(model, "pose", builder.pose((spawn["x_m"], spawn["y_m"], 0.03),
                                   (0, 0, math.radians(spawn["yaw_deg"]))))
    base = sub(model, "link", name="base_link")
    sub(base, "pose", builder.pose((0, 0, 0.36)))
    inertia(base, 1200 * s**3, [v * s**5 for v in (344.81,2031.25,2363.56)])
    builder.box(base, "chassis", (4.50, 1.84, 0.25), (0, 0, 0), "0.08 0.31 0.68 1")
    builder.box(base, "deck", (3.20, 1.70, 0.10), (-0.05, 0, 0.17), "0.82 0.85 0.88 1")
    builder.box(base, "front_bumper", (0.09, 1.75, 0.12), (2.265, 0, -0.04), "0.04 0.05 0.06 1")
    builder.box(base, "rear_bumper", (0.09, 1.75, 0.12), (-2.265, 0, -0.04), "0.04 0.05 0.06 1")
    builder.cylinder(base, "sensor_mast", 0.035, 0.21, (0, 0, 0.30), "0.15 0.17 0.19 1")
    for side in (1, -1):
        builder.box(base, f"headlight_{side}", (0.025, 0.20, 0.06), (2.26, 0.65 * side, 0.04),
                    "0.97 0.94 0.70 1", collision=False, emissive="0.3 0.3 0.2 1")
        builder.box(base,f"mirror_{side}",(0.22,0.14,0.14),(1.4,0.99*side,0.18),"0.08 0.31 0.68 1")
    wheel_names = []
    steering_names=[]
    for end, x in (("front", WHEELBASE/2), ("rear", -WHEELBASE/2)):
        for side, y in (("left", TRACK/2), ("right", -TRACK/2)):
            name = f"{end}_{side}_wheel"
            wheel_names.append(name)
            parent="base_link"
            if end=="front":
                parent=f"front_{side}_knuckle"
                knuckle=sub(model,"link",name=parent)
                sub(knuckle,"pose",builder.pose((x,y,WHEEL_RADIUS)))
                inertia(knuckle,2*s**3,[0.005*s**5]*3)
                steering_name=f"front_{side}_steering_joint"
                steering_names.append(steering_name)
                steering=sub(model,"joint",name=steering_name,type="revolute")
                sub(steering,"parent","base_link");sub(steering,"child",parent)
                axis=sub(steering,"axis")
                sub(axis,"xyz","0 0 1",expressed_in="__model__")
                limit=sub(axis,"limit")
                for tag,value in (("lower",-0.65),("upper",0.65),("effort",1500*s**4),("velocity",1.5)):
                    sub(limit,tag,value)
            link = sub(model, "link", name=name)
            sub(link, "pose", builder.pose((x, y, WHEEL_RADIUS), (-math.pi / 2, 0, 0)))
            mass, radius, width = 20.0, WHEEL_RADIUS, 0.18
            transverse = mass * (3 * radius**2 + width**2) / 12
            axial = mass * radius**2 / 2
            inertia(link, mass * s**3, [transverse * s**5, transverse * s**5, axial * s**5])
            builder.cylinder(link, "tire", radius, width, (0, 0, 0), "0.035 0.04 0.045 1")
            builder.cylinder(link, "hub", 0.18, width + 0.008, (0, 0, 0),
                             "0.62 0.66 0.70 1", collision=False)
            surface = sub(link.find("collision"), "surface")
            ode = sub(sub(surface, "friction"), "ode")
            for tag, value in (("mu", 0.9), ("mu2", 0.9), ("slip1", 0.01), ("slip2", 0.05)):
                sub(ode, tag, value)
            joint = sub(model, "joint", name=name + "_joint", type="revolute")
            sub(joint, "parent", parent)
            sub(joint, "child", name)
            axis = sub(joint, "axis")
            sub(axis, "xyz", "0 1 0", expressed_in=parent if end=="front" else "__model__")
            limit = sub(axis, "limit")
            sub(limit, "lower", "-1e16")
            sub(limit, "upper", "1e16")
            sub(limit, "effort", 800 * s**4)
            sub(limit, "velocity", 20)
            sub(sub(axis, "dynamics"), "damping", 0.02 * s**5)

    def sensor_link(name, position):
        link = sub(model, "link", name=name)
        sub(link, "pose", builder.pose(position))
        inertia(link, 0.05 * s**3, [0.0001 * s**5] * 3)
        joint = sub(model, "joint", name=name + "_fixed", type="fixed")
        sub(joint, "parent", "base_link")
        sub(joint, "child", name)
        return link

    lidar_link = sensor_link("lidar_link", (0, 0, 0.76))
    builder.cylinder(lidar_link, "lidar_housing", 0.09, 0.07, (0, 0, 0), "0.12 0.14 0.16 1")
    builder.cylinder(lidar_link, "lidar_band", 0.094, 0.012, (0, 0, 0), "0.16 0.75 0.73 1", collision=False)
    lidar = sub(lidar_link, "sensor", name="lidar", type="gpu_lidar")
    sub(lidar, "topic", "/robot/scan")
    sub(lidar, "gz_frame_id", "lidar_link")
    sub(lidar, "always_on", "true")
    sub(lidar, "update_rate", 10)
    sub(lidar, "visualize", "true")
    ray = sub(lidar, "lidar")
    horizontal = sub(sub(ray, "scan"), "horizontal")
    for tag, value in (("samples", 720), ("resolution", 1), ("min_angle", -math.pi), ("max_angle", math.pi)):
        sub(horizontal, tag, value)
    ranges = sub(ray, "range")
    for tag, value in (("min", 0.12 * s), ("max", 30 * s), ("resolution", 0.01 * s)):
        sub(ranges, tag, value)
    noise = sub(ray, "noise", type="gaussian")
    sub(noise, "mean", 0)
    sub(noise, "stddev", 0.008 * s)
    camera_link = sensor_link("camera_link", (2.25, 0, 0.56))
    builder.box(camera_link, "camera_body", (0.08, 0.12, 0.08), (0, 0, 0), "0.07 0.08 0.10 1")
    builder.cylinder(camera_link, "camera_lens", 0.025, 0.02, (0.05, 0, 0),
                     "0.08 0.30 0.37 1", rotation=(0, math.pi / 2, 0), collision=False)
    camera = sub(camera_link, "sensor", name="front_camera", type="camera")
    sub(camera, "pose", builder.pose((0.07, 0, 0)))
    sub(camera, "topic", "/robot/camera/image")
    sub(camera, "gz_frame_id", "camera_optical_frame")
    sub(camera, "always_on", "true")
    sub(camera, "update_rate", 15)
    sub(camera, "visualize", "false")
    optics = sub(camera, "camera")
    sub(optics, "horizontal_fov", 1.0472)
    sub(optics, "camera_info_topic", "/robot/camera/camera_info")
    image = sub(optics, "image")
    for tag, value in (("width", 640), ("height", 480), ("format", "R8G8B8")):
        sub(image, tag, value)
    clip = sub(optics, "clip")
    sub(clip, "near", 0.05 * s)
    sub(clip, "far", 50 * s)
    imu_link = sensor_link("imu_link", (0, 0, 0.38))
    imu = sub(imu_link, "sensor", name="imu", type="imu")
    sub(imu, "topic", "/robot/imu")
    sub(imu, "gz_frame_id", "imu_link")
    sub(imu, "always_on", "true")
    sub(imu, "update_rate", 100)
    sub(imu, "imu")
    bumper = sub(base, "sensor", name="bumper_contact", type="contact")
    sub(bumper, "topic", "/robot/contacts")
    sub(bumper, "always_on", "true")
    sub(bumper, "update_rate", 30)
    sub(sub(bumper, "contact"), "collision", "front_bumper_collision")
    drive = sub(model, "plugin", filename="gz-sim-ackermann-steering-system", name="gz::sim::systems::AckermannSteering")
    for side in ("left", "right"):
        for end in ("front", "rear"):
            sub(drive, side + "_joint", f"{end}_{side}_wheel_joint")
    sub(drive,"left_steering_joint","front_left_steering_joint")
    sub(drive,"right_steering_joint","front_right_steering_joint")
    for tag, value in (("wheel_separation", TRACK*s), ("wheel_radius", WHEEL_RADIUS*s),
                       ("wheel_base",WHEELBASE*s),("kingpin_width",TRACK*s),
                       ("steering_limit",0.55),("steer_p_gain",5.0),
                       ("topic", "/robot/cmd_vel"), ("odom_topic", "/robot/odom"),
                       ("tf_topic", "/robot/tf"), ("frame_id", "odom"),
                       ("child_frame_id", "base_footprint"), ("odom_publish_frequency", 30),
                       ("max_velocity",0.8*s),("min_velocity",-0.8*s),
                       ("max_acceleration",0.5*s),("min_acceleration",-0.5*s)):
        sub(drive, tag, value)
    joints = sub(model, "plugin", filename="gz-sim-joint-state-publisher-system",
                 name="gz::sim::systems::JointStatePublisher")
    sub(joints, "topic", "/robot/joint_states")
    for name in wheel_names:
        sub(joints, "joint_name", name + "_joint")
    for name in steering_names:
        sub(joints,"joint_name",name)


def write_urdf(path, scale):
    """Mirror the SDF frames and visuals; Gazebo spawns the embedded SDF robot."""
    s = scale
    root = ET.Element("robot", name="parking_robot")
    sub(root, "link", name="base_footprint")

    def link(name, shape=None, size=None, color="0.08 0.31 0.68 1", origin=(0, 0, 0)):
        item = sub(root, "link", name=name)
        if shape:
            visual = sub(item, "visual")
            sub(visual, "origin", xyz=" ".join(str(v * s) for v in origin), rpy="0 0 0")
            geometry = sub(visual, "geometry")
            if shape == "box":
                sub(geometry, "box", size=" ".join(str(v * s) for v in size))
            else:
                sub(geometry, "cylinder", radius=str(size[0] * s), length=str(size[1] * s))
            sub(sub(visual, "material", name=name + "_material"), "color", rgba=color)
        return item

    def joint(name, parent, child, xyz, rpy=(0, 0, 0), moving=False,steering=False):
        item = sub(root, "joint", name=name, type="revolute" if steering else "continuous" if moving else "fixed")
        sub(item, "parent", link=parent)
        sub(item, "child", link=child)
        sub(item, "origin", xyz=" ".join(str(v * s) for v in xyz), rpy=" ".join(map(str, rpy)))
        if moving or steering:
            sub(item, "axis", xyz="0 0 1")
        if steering:
            sub(item,"limit",lower="-0.65",upper="0.65",effort="1500",velocity="1.5")

    link("base_link", "box", (4.50, 1.84, 0.25))
    joint("base_fixed", "base_footprint", "base_link", (0, 0, 0.36))
    for end, x in (("front", WHEELBASE/2), ("rear", -WHEELBASE/2)):
        for side, y in (("left", TRACK/2), ("right", -TRACK/2)):
            name = f"{end}_{side}_wheel"
            link(name, "cylinder", (WHEEL_RADIUS, 0.18), "0.035 0.04 0.045 1")
            parent="base_link";position=(x,y,WHEEL_RADIUS-0.36)
            if end=="front":
                parent=f"front_{side}_knuckle"
                link(parent)
                joint(f"front_{side}_steering_joint","base_link",parent,position,steering=True)
                position=(0,0,0)
            joint(name + "_joint",parent,name,position,(-math.pi/2,0,0),True)
    for name, xyz, shape, size in (("lidar_link", (0, 0, 0.40), "cylinder", (0.09, 0.07)),
                                   ("camera_link", (2.25, 0, 0.20), "box", (0.08, 0.12, 0.08)),
                                   ("imu_link", (0, 0, 0.02), None, None)):
        link(name, shape, size, "0.12 0.14 0.16 1")
        joint(name + "_fixed", "base_link", name, xyz)
    link("camera_optical_frame")
    joint("camera_optical_fixed", "camera_link", "camera_optical_frame", (0.07, 0, 0),
          (-math.pi / 2, 0, -math.pi / 2))
    ET.indent(root, space="  ")
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)
