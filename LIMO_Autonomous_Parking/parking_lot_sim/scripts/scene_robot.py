"""Official AgileX LIMO meshes with a project adaptation for Gazebo Harmonic.

The original LIMO proportions are retained. Its straight-ahead wheel envelope
is uniformly enlarged to 2.12 m wide. This is not an official Jazzy release.
"""

import math
import xml.etree.ElementTree as ET

from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parents[1]
ROBOT_SCALE = 2.12 / (0.168 + 2 * 0.04366261 * 1.2)
MESH_SCALE = 1.2 * ROBOT_SCALE
FOOTPRINT_LENGTH = 0.3215476 * MESH_SCALE
FOOTPRINT_WIDTH = 2.12
WHEELBASE = 0.24 * ROBOT_SCALE
TRACK = 0.168 * ROBOT_SCALE
WHEEL_CENTER = TRACK / 2 + 0.006331305 * MESH_SCALE
WHEEL_RADIUS = 0.04814471 * MESH_SCALE
WHEEL_WIDTH = 0.07466261 * MESH_SCALE
BASE_HEIGHT = WHEEL_RADIUS + 0.10 * ROBOT_SCALE
BODY_CENTER_X = (-0.1630476 + 0.1585) * MESH_SCALE / 2
BODY_CENTER_Z = (0.02437367 + 0.2535344) * MESH_SCALE / 2 - 0.15 * ROBOT_SCALE
BODY_SIZE = (FOOTPRINT_LENGTH, 0.19 * MESH_SCALE, 0.22916073 * MESH_SCALE)
STEERING_LIMIT = 0.45
JOINT_STEERING_LIMIT = 0.523598767
# Environment visuals retain SDF's default all-bits visibility. Reserve a
# separate bit for this robot so its GPU LiDAR excludes its own body/wheels,
# while GUI and RGB cameras (all-bits masks) still render the official meshes.
ROBOT_VISIBILITY_FLAGS = 2
LIDAR_VISIBILITY_MASK = 1
# Conservative envelope containing steered tires as well as the chassis.
STEERED_WIDTH = 2 * (WHEEL_CENTER + math.hypot(WHEEL_RADIUS, WHEEL_WIDTH / 2))


def robot_metadata(scale=1):
    return {"name": "parking_robot", "platform": "AgileX LIMO", "source": "agilexrobotics/limo_ros2, humble, limo_car",
            "adaptation": "Official meshes; uniformly enlarged model and project Harmonic interfaces",
            "drive": "four powered wheels, front Ackermann steering",
            "footprint_size_m": [FOOTPRINT_LENGTH*scale, FOOTPRINT_WIDTH*scale],
            "footprint_center_offset_m": [BODY_CENTER_X*scale, 0.0],
            "steered_envelope_width_m": STEERED_WIDTH*scale,
            "wheelbase_m": WHEELBASE*scale, "track_m": TRACK*scale,
            "navigation_reference_frame": "rear_axle",
            "rear_axle_offset_m": [-WHEELBASE*scale/2, 0.0, 0.0],
            "wheel_radius_m": WHEEL_RADIUS*scale, "steering_limit_rad": STEERING_LIMIT,
            "uniform_scale_from_official_xacro": ROBOT_SCALE,
            "sensors": ["360 degree GPU LiDAR", "Front and rear near-ground LiDAR",
                        "RGB camera", "IMU", "chassis contacts"]}


def write_limo_assets(output_dir):
    source = ROOT / "models" / "limo"
    destination = Path(output_dir) / "models" / "limo"
    if source.resolve() != destination.resolve():
        shutil.copytree(source, destination, dirs_exist_ok=True)


def sdf_mesh(link, name, asset, scale, position=(0,0,0), rotation=(0,0,0)):
    visual = sub(link, "visual", name=name)
    sub(visual, "visibility_flags", ROBOT_VISIBILITY_FLAGS)
    sub(visual, "pose", " ".join(map(str, [v*scale for v in position]+list(rotation))))
    mesh = sub(sub(visual, "geometry"), "mesh")
    sub(mesh, "uri", f"model://limo/meshes/{asset}")
    sub(mesh, "scale", " ".join([str(MESH_SCALE*scale)]*3))


def urdf_mesh(link, asset, scale, position=(0,0,0), rotation=(0,0,0)):
    visual = sub(link, "visual")
    sub(visual, "origin", xyz=" ".join(str(v*scale) for v in position), rpy=" ".join(map(str,rotation)))
    sub(sub(visual, "geometry"), "mesh", filename=f"package://parking_lot_sim/models/limo/meshes/{asset}",
        scale=" ".join([str(MESH_SCALE*scale)]*3))


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
    sub(base, "pose", builder.pose((0, 0, BASE_HEIGHT)))
    base_mass = 4.34 * ROBOT_SCALE**3
    bx, by, bz = BODY_SIZE
    inertia(base, base_mass*s**3, [base_mass*(by*by+bz*bz)/12*s**5,
                                  base_mass*(bx*bx+bz*bz)/12*s**5,
                                  base_mass*(bx*bx+by*by)/12*s**5])
    sub(base.find("inertial"), "pose", builder.pose((BODY_CENTER_X,0,BODY_CENTER_Z)))
    navigation_frame = sub(model, "frame", name="rear_axle", attached_to="base_link")
    sub(navigation_frame, "pose", builder.pose((-WHEELBASE/2,0,0)), relative_to="__model__")
    sdf_mesh(base, "limo_body", "limo_base.dae", s, (0,0,-0.15*ROBOT_SCALE), (0,0,math.pi/2))
    builder.box(base,"chassis",BODY_SIZE,(BODY_CENTER_X,0,BODY_CENTER_Z),"0.1 0.1 0.1 1")
    base.remove(base.findall("visual")[-1])
    wheel_names = []
    steering_names=[]
    for end, x in (("front", WHEELBASE/2), ("rear", -WHEELBASE/2)):
        for side, y in (("left", WHEEL_CENTER), ("right", -WHEEL_CENTER)):
            name = f"{end}_{side}_wheel"
            wheel_names.append(name)
            parent="base_link"
            if end=="front":
                parent=f"front_{side}_knuckle"
                knuckle=sub(model,"link",name=parent)
                sub(knuckle,"pose",builder.pose((x,TRACK/2 if side=="left" else -TRACK/2,WHEEL_RADIUS)))
                inertia(knuckle,0.25*ROBOT_SCALE**3*s**3,[v*ROBOT_SCALE**5*s**5 for v in (0.00525,0.00035,0.00525)])
                steering_name=f"front_{side}_steering_joint"
                steering_names.append(steering_name)
                steering=sub(model,"joint",name=steering_name,type="revolute")
                sub(steering,"parent","base_link");sub(steering,"child",parent)
                axis=sub(steering,"axis")
                sub(axis,"xyz","0 0 1",expressed_in="__model__")
                limit=sub(axis,"limit")
                for tag,value in (("lower",-JOINT_STEERING_LIMIT),("upper",JOINT_STEERING_LIMIT),("effort",1500*s**4),("velocity",1.5)):
                    sub(limit,tag,value)
            link = sub(model, "link", name=name)
            sub(link, "pose", builder.pose((x, y, WHEEL_RADIUS), (-math.pi / 2, 0, 0)))
            mass, radius, width = 0.5*ROBOT_SCALE**3, WHEEL_RADIUS, WHEEL_WIDTH
            transverse = mass * (3 * radius**2 + width**2) / 12
            axial = mass * radius**2 / 2
            inertia(link, mass * s**3, [transverse * s**5, transverse * s**5, axial * s**5])
            builder.cylinder(link, "tire", radius, width, (0, 0, 0), "0.035 0.04 0.045 1")
            link.remove(link.findall("visual")[-1])
            sign = 1 if side == "left" else -1
            sdf_mesh(link,"limo_wheel","limo_wheel.dae",s,
                     (0,0,-sign*0.006331305*MESH_SCALE),
                     (math.pi/2 if side=="left" else -math.pi/2,0,0))
            surface = sub(link.find("collision"), "surface")
            ode = sub(sub(surface, "friction"), "ode")
            # The previous nonzero force-dependent slip, combined with the
            # car-sized model mass, made wheel odometry overstate translation.
            for tag, value in (("mu", 0.9), ("mu2", 0.9), ("slip1", 0.0), ("slip2", 0.0)):
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

    lidar_link = sensor_link("lidar_link", (0.1236*ROBOT_SCALE, 0, BASE_HEIGHT-0.01*ROBOT_SCALE))
    lidar = sub(lidar_link, "sensor", name="lidar", type="gpu_lidar")
    sub(lidar, "topic", "/robot/scan")
    sub(lidar, "gz_frame_id", "lidar_link")
    sub(lidar, "always_on", "true")
    sub(lidar, "update_rate", 10)
    sub(lidar, "visualize", "true")
    ray = sub(lidar, "lidar")
    sub(ray, "visibility_mask", LIDAR_VISIBILITY_MASK)
    horizontal = sub(sub(ray, "scan"), "horizontal")
    for tag, value in (("samples", 720), ("resolution", 1), ("min_angle", -math.pi), ("max_angle", math.pi)):
        sub(horizontal, tag, value)
    ranges = sub(ray, "range")
    for tag, value in (("min", 0.12 * s), ("max", 30 * s), ("resolution", 0.01 * s)):
        sub(ranges, tag, value)
    noise = sub(ray, "noise", type="gaussian")
    sub(noise, "mean", 0)
    sub(noise, "stddev", 0.008 * s)

    add_parking_sensors(model, s)
    camera_link = sensor_link("camera_link", (0.1*ROBOT_SCALE, 0, BASE_HEIGHT+0.065*ROBOT_SCALE))
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
    imu_link = sensor_link("imu_link", (0, 0, BASE_HEIGHT-0.1*ROBOT_SCALE))
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
    sub(sub(bumper, "contact"), "collision", "chassis_collision")
    drive = sub(model, "plugin", filename="gz-sim-ackermann-steering-system", name="gz::sim::systems::AckermannSteering")
    for side in ("left", "right"):
        # AckermannSteering uses the first wheel on each side for odometry.
        # Integrate rear wheels at the rear-axle reference frame.
        for end in ("rear", "front"):
            sub(drive, side + "_joint", f"{end}_{side}_wheel_joint")
    sub(drive,"left_steering_joint","front_left_steering_joint")
    sub(drive,"right_steering_joint","front_right_steering_joint")
    for tag, value in (("wheel_separation", 2*WHEEL_CENTER*s), ("wheel_radius", WHEEL_RADIUS*s),
                       ("wheel_base",WHEELBASE*s),("kingpin_width",TRACK*s),
                       ("steering_limit",STEERING_LIMIT),("steer_p_gain",5.0),
                       ("topic", "/robot/cmd_vel"), ("odom_topic", "/robot/odom"),
                       ("tf_topic", "/robot/tf"), ("frame_id", "odom"),
                       ("child_frame_id", "rear_axle"), ("odom_publish_frequency", 30),
                       ("max_velocity",1.75*s),("min_velocity",-0.8*s),
                       ("max_acceleration",0.5*s),("min_acceleration",-0.5*s)):
        sub(drive, tag, value)
    joints = sub(model, "plugin", filename="gz-sim-joint-state-publisher-system",
                 name="gz::sim::systems::JointStatePublisher")
    sub(joints, "topic", "/robot/joint_states")
    for name in wheel_names:
        sub(joints, "joint_name", name + "_joint")
    for name in steering_names:
        sub(joints,"joint_name",name)


def add_parking_sensors(model, scale):
    """Near-ground range sensing and an independent simulator evaluation topic."""
    base = model.find("link[@name='base_link']")
    for side, sign in (("front", 1), ("rear", -1)):
        sensor = sub(base, "sensor", name=f"low_{side}", type="gpu_lidar")
        x = (BODY_CENTER_X+sign*(FOOTPRINT_LENGTH/2+0.035))*scale
        z = (0.12-BASE_HEIGHT)*scale
        sub(sensor, "pose", f"{x} 0 {z} 0 0 {0 if sign == 1 else math.pi}")
        sub(sensor, "topic", f"/robot/scan_low_{side}")
        sub(sensor, "gz_frame_id", f"low_{side}_link")
        sub(sensor, "always_on", "true")
        sub(sensor, "update_rate", 12)
        lidar = sub(sensor, "lidar")
        sub(lidar, "visibility_mask", LIDAR_VISIBILITY_MASK)
        horizontal = sub(sub(lidar, "scan"), "horizontal")
        for name, value in (("samples", 181), ("resolution", 1),
                            ("min_angle", -math.pi*0.59), ("max_angle", math.pi*0.59)):
            sub(horizontal, name, value)
        range_element = sub(lidar, "range")
        for name, value in (("min", 0.05*scale), ("max", 6.0*scale), ("resolution", 0.01*scale)):
            sub(range_element, name, value)
    truth = sub(model, "plugin", filename="gz-sim-odometry-publisher-system",
                name="gz::sim::systems::OdometryPublisher")
    sub(truth, "odom_topic", "/robot/evaluation_pose")
    sub(truth, "odom_frame", "world")
    sub(truth, "robot_base_frame", "truth_rear_axle")
    sub(truth, "xyz_offset", f"{-WHEELBASE*scale/2} 0 0")
    sub(truth, "odom_publish_frequency", 30)


def write_urdf(path, scale):
    """Mirror the SDF frames and visuals; Gazebo spawns the embedded SDF robot."""
    s = scale
    root = ET.Element("robot", name="parking_robot")
    sub(root, "link", name="rear_axle")
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
            sub(item,"limit",lower=str(-JOINT_STEERING_LIMIT),upper=str(JOINT_STEERING_LIMIT),effort="1500",velocity="1.5")

    base = link("base_link")
    urdf_mesh(base,"limo_base.dae",s,(0,0,-0.15*ROBOT_SCALE),(0,0,math.pi/2))
    joint("base_fixed", "base_footprint", "base_link", (0, 0, BASE_HEIGHT))
    joint("rear_axle_fixed", "rear_axle", "base_footprint", (WHEELBASE/2, 0, 0))
    for end, x in (("front", WHEELBASE/2), ("rear", -WHEELBASE/2)):
        for side, y in (("left", WHEEL_CENTER), ("right", -WHEEL_CENTER)):
            name = f"{end}_{side}_wheel"
            wheel_link = link(name)
            sign = 1 if side=="left" else -1
            urdf_mesh(wheel_link,"limo_wheel.dae",s,(0,0,-sign*0.006331305*MESH_SCALE),
                      (math.pi/2 if side=="left" else -math.pi/2,0,0))
            parent="base_link";position=(x,y,WHEEL_RADIUS-BASE_HEIGHT)
            if end=="front":
                parent=f"front_{side}_knuckle"
                link(parent)
                hinge_y = TRACK/2 if side=="left" else -TRACK/2
                joint(f"front_{side}_steering_joint","base_link",parent,(x,hinge_y,WHEEL_RADIUS-BASE_HEIGHT),steering=True)
                position=(0,y-hinge_y,0)
            joint(name + "_joint",parent,name,position,(-math.pi/2,0,0),True)
    for name, xyz in (("lidar_link",(0.1236*ROBOT_SCALE,0,-0.01*ROBOT_SCALE)),
                      ("camera_link",(0.1*ROBOT_SCALE,0,0.065*ROBOT_SCALE)),
                      ("imu_link",(0,0,-0.1*ROBOT_SCALE))):
        link(name)
        joint(name+"_fixed","base_link",name,xyz)
    for side, sign in (("front", 1), ("rear", -1)):
        name = f"low_{side}_link"
        link(name)
        joint(name+"_fixed", "base_link", name,
              (BODY_CENTER_X+sign*(FOOTPRINT_LENGTH/2+0.035), 0, 0.12-BASE_HEIGHT),
              (0, 0, 0 if sign == 1 else math.pi))
    link("camera_optical_frame")
    joint("camera_optical_fixed", "camera_link", "camera_optical_frame", (0.07, 0, 0),
          (-math.pi / 2, 0, -math.pi / 2))
    ET.indent(root, space="  ")
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)
