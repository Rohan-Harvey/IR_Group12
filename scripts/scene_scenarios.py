"""Six controlled parking cases. All labels and goals are evaluation metadata.

The robot must infer spaces from sensors. This module never feeds targets to
the controller. The combined world has one immediately clear accepted case;
S2 is a wait/reassess case, not a permanently unusable geometric space.
"""

import copy
import csv
import math
from pathlib import Path
import random

from scene_geometry import overlaps, rectangle


CAR_LENGTH = 4.62
CAR_WIDTH = 2.12
WHEELBASE = 2.75
REVERSE_RADIUS = 5.50
SAFETY_MARGIN = 0.15
ANGLE_TOLERANCE_DEG = 10.0
SEDAN_BODY = [4.50, 1.84, 1.52]


def definitions():
    return [
        {"id":"S1", "name":"Inward-angled cars", "kind":"tapered_gap",
         "goal":[-14.0,14.6,-90.0], "zone":[-18.4,11.5,-9.6,17.395],
         "expected":"REJECT", "reason":"Wide entrance, but inward-pointing cars obstruct the perpendicular final footprint.",
         "spawn":[-7.125,10.475,0.0]},
        {"id":"S2", "name":"Usable space, pedestrian crossing", "kind":"pedestrian_block",
         "goal":[0.0,14.6,-90.0], "zone":[-3.6,11.5,3.6,17.395],
         "expected":"WAIT_WHILE_BLOCKED", "reason":"Static goal and reverse path fit; a walking person initially occupies the swept path. Reassess after they clear it.",
         "spawn":[6.875,10.475,0.0]},
        {"id":"S3", "name":"Too narrow", "kind":"narrow_gap",
         "goal":[-14.0,-14.6,90.0], "zone":[-14.85,-17.395,-13.15,-11.5],
         "expected":"REJECT", "reason":"1.70 m opening is narrower than the 2.12 m vehicle, even before safety margins.",
         "opening_width_m":1.70, "spawn":[-7.125,-10.475,0.0]},
        {"id":"S4", "name":"Only a steep parking angle fits", "kind":"orientation_failure",
         "goal":[0.0,-13.0,90.0], "diagonal_pose":[0.0,-13.0,55.0],
         "zone":[-4.4,-17.395,4.4,-9.0], "expected":"REJECT",
         "reason":"The diagonal pose is collision-free but 35 degrees from perpendicular; the permitted error is at most 10 degrees.",
         "spawn":[-7.0,-7.4,0.0]},
        {"id":"S5", "name":"No room for reverse entry", "kind":"approach_blocked",
         "goal":[14.0,-14.6,90.0], "zone":[12.3,-17.395,15.7,-11.5],
         "expected":"REJECT", "reason":"Final 3.40 m wide pocket fits, but opposing cars leave a 1.75 m approach throat; the vehicle cannot enter it.",
         "opening_width_m":3.40, "approach_throat_m":1.75, "spawn":[14.0,-7.5,0.0]},
        {"id":"S6", "name":"Large clear perpendicular reverse park", "kind":"clear_reverse",
         "goal":[14.0,14.6,-90.0], "zone":[10.0,11.5,18.0,17.395],
         "expected":"ACCEPT", "reason":"8.00 m clear opening, a collision-free reverse arc, and a final pose perpendicular to the curb.",
         "opening_width_m":8.00, "spawn":[20.875,10.475,0.0]},
    ]


def make_layout(config):
    cars=[]
    palette=[("silver","0.63 0.68 0.72 1"),("blue","0.06 0.22 0.55 1"),
             ("red","0.66 0.08 0.07 1"),("white","0.88 0.90 0.88 1"),
             ("green","0.12 0.36 0.25 1"),("ochre","0.82 0.48 0.09 1")]
    rng=random.Random(config["seed"])
    def car(name,x,y,yaw,color=None):
        color_name,rgba=color or palette[len(cars)%len(palette)]
        cars.append({"id":name,"row":"scenario", "occupied":True,
                     "center_m":[x,y],"bay_size_m":[2.8,5.2],
                     "car":{"model_name":"car_"+name,"type":"sedan","color_name":color_name,
                            "rgba":rgba,"position_m":[x,y,0.0],"yaw_rad":math.radians(yaw),
                            "body_size_m":list(SEDAN_BODY),"envelope_m":[CAR_LENGTH,CAR_WIDTH,1.535]}})
    car("S1_left",-16.6,14.4,60)
    car("S1_right",-11.4,14.4,120)
    for sign in [-1,1]:
        car(f"S2_{sign}",sign*(7.2/2+CAR_WIDTH/2),14.6,90)
        car(f"S3_{sign}",-14+sign*(1.7/2+CAR_WIDTH/2),-14.6,90)
        car(f"S5_side_{sign}",14+sign*(3.4/2+CAR_WIDTH/2),-14.6,90)
        car(f"S5_approach_{sign}",14+sign*(1.75/2+CAR_LENGTH/2),-10.7,0)
        car(f"S6_{sign}",14+sign*(8.0/2+CAR_WIDTH/2),14.6,90)
    angle=math.radians(55)
    separation=2.7/2+CAR_WIDTH/2
    for sign in [-1,1]:
        car(f"S4_{sign}",sign*math.sin(angle)*separation,
            -13-sign*math.cos(angle)*separation,55)
    for i,x in enumerate([-20.3,-7.4]):
        car(f"north_filler_{i}",x,14.6,90)
    for i,x in enumerate([-20.4,-18.15,-9.2,-6.5,5.6,8.1,20.2]):
        car(f"south_filler_{i}",x,-14.6,-90)
    for row,y,heading in [("B",3.2,-90),("C",-3.2,90)]:
        for i in range(12):
            car(f"{row}{i+1:02d}",(i-5.5)*2.8,y,heading+rng.uniform(-4,4))
    return cars


def add_layout_obstacles(builder):
    # Prevent the leftover strip between S2 and S6 becoming another tiny bay.
    link=builder.model("curbside_planter")
    builder.box(link,"concrete",(0.7,1.8,0.55),(6.8,16.0,0.275),"0.54 0.55 0.50 1")
    builder.box(link,"soil",(0.56,1.64,0.08),(6.8,16.0,0.56),"0.22 0.18 0.12 1")
    builder.sphere(link,"shrub",0.34,(6.8,16.0,0.83),"0.18 0.38 0.20 1")
    wall=builder.model("S5_back_wall")
    builder.box(wall,"wall",(11.0,0.20,0.90),(14.0,-17.7,0.45),"0.60 0.61 0.56 1")


def people_routes():
    return [
        {"name":"pedestrian_1","start":[-18,18.3],"end":[-7,18.3],"ground_z_m":0.12,"speed":1.2,"delay":0,"shirt":"blue"},
        {"name":"pedestrian_2","start":[-5,18.3],"end":[6,18.3],"ground_z_m":0.12,"speed":1.2,"delay":1,"shirt":"orange"},
        {"name":"pedestrian_3","start":[8,18.3],"end":[19,18.3],"ground_z_m":0.12,"speed":1.1,"delay":2,"shirt":"green"},
        {"name":"pedestrian_4","start":[-17,-18.3],"end":[-3,-18.3],"ground_z_m":0.12,"speed":1.2,"delay":0,"shirt":"orange"},
        {"name":"pedestrian_5","start":[1,-18.3],"end":[17,-18.3],"ground_z_m":0.12,"speed":1.3,"delay":2,"shirt":"blue"},
        {"name":"pedestrian_S2_blocker","start":[2.5,11.2],"end":[-3.5,11.2],"speed":1.1,"delay":0,"shirt":"green"},
    ]


def obstacle_polygons(cars):
    obstacles=[(b["car"]["model_name"],rectangle(b["car"]["position_m"],
                  b["car"]["envelope_m"][0],b["car"]["envelope_m"][1],b["car"]["yaw_rad"])) for b in cars]
    for name,center,length,width in [
        ("north_curb",(0,17.47),43.0,0.15),("south_curb",(0,-17.47),43.0,0.15),
        ("central_island",(0,0),33.6,1.2),("east_wall",(24.15,0),0.3,40),
        ("planter",(6.8,16.0),0.7,1.8),("S5_back_wall",(14,-17.7),11.0,0.20)]:
        obstacles.append((name,rectangle(center,length,width,0)))
    return obstacles


def pose_collisions(pose,obstacles,margin=SAFETY_MARGIN):
    shape=rectangle(pose[:2],CAR_LENGTH+2*margin,CAR_WIDTH+2*margin,math.radians(pose[2]))
    return [name for name,polygon in obstacles if overlaps(shape,polygon)]


def reverse_arc(goal,samples=361):
    """Kinematic reverse-left/right quarter circle, referenced at the rear axle.

    Poses in the output are the vehicle/model center; drive steering limits are
    checked separately. This is a reference path for independent evaluation.
    """
    gx,gy,heading=goal
    side=1 if gy>0 else -1
    speed=0.4
    distance_total=REVERSE_RADIUS*math.pi/2
    acceleration=0.4
    ramp_time=speed/acceleration
    ramp_distance=speed*speed/(2*acceleration)
    duration=2*ramp_time+(distance_total-2*ramp_distance)/speed
    path=[]
    for i in range(samples):
        fraction=i/(samples-1)
        distance=distance_total*fraction
        if distance<ramp_distance:
            time=math.sqrt(2*distance/acceleration)
            velocity=-acceleration*time
        elif distance>distance_total-ramp_distance:
            remaining=math.sqrt(2*(distance_total-distance)/acceleration)
            time=duration-remaining
            velocity=-acceleration*remaining
        else:
            time=ramp_time+(distance-ramp_distance)/speed
            velocity=-speed
        angle=fraction*math.pi/2
        yaw=-side*angle
        rear_x=gx+REVERSE_RADIUS-REVERSE_RADIUS*math.sin(angle)
        rear_y=gy+side*WHEELBASE/2-side*REVERSE_RADIUS*math.cos(angle)
        path.append({"time_s":time,
                     "x_m":rear_x+WHEELBASE/2*math.cos(yaw),
                     "y_m":rear_y+WHEELBASE/2*math.sin(yaw),
                     "yaw_deg":math.degrees(yaw),"speed_m_s":velocity,
                     "rear_axle_curvature_1_m":side/REVERSE_RADIUS})
    return path


def make_manifest(config,cars):
    s=config["scale"]
    obstacles=obstacle_polygons(cars)
    cases=definitions()
    for case in cases:
        case["goal_pose_m_deg"]=case.pop("goal")
        case["spawn_pose_m_deg"]=case.pop("spawn")
        case["target_zone_xyxy_m"]=case.pop("zone")
        case["static_goal_collisions"]=pose_collisions(case["goal_pose_m_deg"],obstacles)
        case["static_goal_fits"]=not case["static_goal_collisions"]
        if "diagonal_pose" in case:
            case["diagonal_pose_m_deg"]=case.pop("diagonal_pose")
            case["diagonal_pose_fits"]=not pose_collisions(case["diagonal_pose_m_deg"],obstacles)
            case["diagonal_heading_error_deg"]=35.0
        if case["id"] in ["S2","S6"]:
            path=reverse_arc(case["goal_pose_m_deg"])
            # Additional padding covers movement between the 0.25-degree samples.
            collisions=[(i,pose_collisions([p["x_m"],p["y_m"],p["yaw_deg"]],obstacles,SAFETY_MARGIN+0.025))
                        for i,p in enumerate(path)]
            case["reference_path_static_clear"]=not any(names for _,names in collisions)
            case["reference_path_collision_samples"]=[i for i,names in collisions if names]
            case["reference_path_csv"]=f"reference_paths/{case['id']}_reverse.csv"
        if case["id"]=="S2":
            person=rectangle((2.5,11.2),0.8,0.8,0)
            path=reverse_arc(case["goal_pose_m_deg"])
            case["pedestrian_intersects_reference_path_at_start"]=config.get('include_pedestrians',True) and any(
                overlaps(rectangle((p["x_m"],p["y_m"]),CAR_LENGTH+0.3,CAR_WIDTH+0.3,math.radians(p["yaw_deg"])),person)
                for p in path)
            case["blocking_pedestrian"]="pedestrian_S2_blocker"
            if not config.get('include_pedestrians',True):
                case['expected']='ACCEPT'
                case['reason']='Static goal and reverse path fit; pedestrians are disabled in this configuration.'
        # All exported scene coordinates scale with the world.
        for key in ("goal_pose_m_deg","spawn_pose_m_deg","diagonal_pose_m_deg"):
            if key in case:
                case[key]=[case[key][0]*s,case[key][1]*s,case[key][2]]
        case["target_zone_xyxy_m"]=[v*s for v in case["target_zone_xyxy_m"]]
        for key in ("opening_width_m","approach_throat_m"):
            if key in case:
                case[key]*=s
    return {"purpose":"Evaluation/reference data ONLY. Do not feed case labels or goals to navigation.",
            "scale":s,"layout":"six_controlled_cases","immediately_accepted_case":"S6",
            "immediately_accepted_cases":[c['id'] for c in cases if c['expected']=='ACCEPT'],
            "dynamic_caveat":"S2 becomes eligible after its pedestrian clears the actual planned swept path.",
            "robot_footprint_m":[CAR_LENGTH*s,CAR_WIDTH*s],"wheelbase_m":WHEELBASE*s,
            "safety_margin_m":SAFETY_MARGIN*s,"maximum_heading_error_deg":ANGLE_TOLERANCE_DEG,
            "reverse_reference_radius_m":REVERSE_RADIUS*s,"cases":cases}


def write_reference_paths(root,config):
    root=Path(root)
    directory=root/"config"/"reference_paths"
    directory.mkdir(parents=True,exist_ok=True)
    scale=config["scale"]
    for case in definitions():
        if case["id"] not in ["S2","S6"]:
            continue
        rows=reverse_arc(case["goal"])
        for p in rows:
            for field in ("x_m","y_m","speed_m_s"):
                p[field]*=scale
            p["rear_axle_curvature_1_m"]/=scale
        with (directory/f"{case['id']}_reverse.csv").open("w",newline="") as handle:
            writer=csv.DictWriter(handle,fieldnames=list(rows[0]))
            writer.writeheader();writer.writerows(rows)
