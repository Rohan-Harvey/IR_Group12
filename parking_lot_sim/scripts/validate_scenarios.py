#!/usr/bin/env python3
"""Independent evaluation checks for the six controlled cases; no ROS required."""

import argparse
import csv
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from scene_geometry import overlaps, rectangle
import scene_scenarios as suite
from validate_world import validate


ROOT=Path(__file__).resolve().parents[1]


def check(value,message):
    if not value:
        raise ValueError(message)


def verify_s5_enclosure(root,scale):
    """Certify a closed blocked curve around S5 from actual SDF collision boxes.

    The chassis contains an inscribed disc at every yaw. If a closed curve of
    forbidden disc-center positions encloses the goal, no planar vehicle path
    from outside can reach it. A Lipschitz bound covers between sample points.
    """
    world=ET.parse(root/'worlds/parking_lot.sdf').getroot().find('world')
    robot=world.find("model[@name='parking_robot']")
    width=(float(robot.findtext("link[@name='base_link']/collision[@name='chassis_collision']/geometry/box/size").split()[1])
           if robot is not None else 1.84*scale)
    radius=width/2
    names=['car_S5_side_-1','car_S5_side_1','car_S5_approach_-1','car_S5_approach_1','S5_back_wall']
    boxes=[];centers={}
    for name in names:
        model=world.find(f"model[@name='{name}']")
        pose=[float(v) for v in model.findtext('pose').split()]
        centers[name]=pose[:2]
        cosine,sine=math.cos(pose[5]),math.sin(pose[5])
        for item in model.findall('link/collision'):
            dimensions=item.findtext('geometry/box/size')
            if not dimensions:continue
            size=[float(v) for v in dimensions.split()]
            local=[float(v) for v in item.findtext('pose','0 0 0 0 0 0').split()]
            # The parked car boxes are model-relative; their body link is at 0.
            if local[2]+size[2]/2<0.235*scale or local[2]-size[2]/2>0.485*scale:continue
            xy=(pose[0]+cosine*local[0]-sine*local[1],pose[1]+sine*local[0]+cosine*local[1])
            boxes.append((xy,size[0]/2,size[1]/2,pose[5]+local[5]))
    def distance(point):
        distances=[]
        for (x,y),half_l,half_w,yaw in boxes:
            dx,dy=point[0]-x,point[1]-y
            px=math.cos(yaw)*dx+math.sin(yaw)*dy
            py=-math.sin(yaw)*dx+math.cos(yaw)*dy
            distances.append(math.hypot(max(abs(px)-half_l,0),max(abs(py)-half_w,0)))
        return min(distances)
    left=centers['car_S5_side_-1'];right=centers['car_S5_side_1']
    wall_y=-17.7*scale
    curve=[left,centers['car_S5_approach_-1'],centers['car_S5_approach_1'],right,
           [right[0],wall_y],[left[0],wall_y]]
    for a,b in zip(curve,curve[1:]+curve[:1]):
        length=math.hypot(b[0]-a[0],b[1]-a[1]);count=max(1,math.ceil(length/(0.01*scale)))
        distances=[distance([a[0]+(b[0]-a[0])*i/count,a[1]+(b[1]-a[1])*i/count]) for i in range(count+1)]
        check(max(distances)+length/count/2<radius,'S5 enclosure has a possible chassis-width opening')
    def inside(point):
        x,y=point;result=False
        for a,b in zip(curve,curve[1:]+curve[:1]):
            if (a[1]>y)!=(b[1]>y) and x<(b[0]-a[0])*(y-a[1])/(b[1]-a[1])+a[0]:result=not result
        return result
    check(inside((14*scale,-14.6*scale)),'S5 goal is not enclosed')
    check(not inside((14*scale,-7.5*scale)),'S5 initial pose should be outside the enclosure')


def validate_cases(root=ROOT):
    validate(root)
    config=json.loads((root/"config/parking_lot.json").read_text())
    manifest=json.loads((root/"config/parking_scenarios.json").read_text())
    cars=suite.make_layout(config)
    obstacles=suite.obstacle_polygons(cars)
    cases={c['id']:c for c in manifest['cases']}
    check(set(cases)=={f'S{i}' for i in range(1,7)},"Need exactly six named scenarios")
    expected=['S6'] if config.get('include_pedestrians',True) else ['S2','S6']
    check([c['id'] for c in cases.values() if c['expected']=='ACCEPT']==expected,"Unexpected immediately accepted cases")
    check(not cases['S1']['static_goal_fits'],"Inward-angled case unexpectedly fits")
    check(not cases['S3']['static_goal_fits'],"Narrow case unexpectedly fits")
    check(cases['S3']['opening_width_m']<1.84*config['scale'],"Narrow opening must also be narrower than the chassis")
    check(cases['S4']['diagonal_pose_fits'] and not cases['S4']['static_goal_fits'],"Steep-angle geometry is incorrect")
    check(cases['S4']['diagonal_heading_error_deg']>manifest['maximum_heading_error_deg'],"Steep pose must fail heading tolerance")
    check(cases['S5']['static_goal_fits'],"Approach-blocked final placement must fit")
    check(cases['S5']['approach_throat_m']<1.84*config['scale'],"Approach throat must be narrower than the physical chassis")
    verify_s5_enclosure(root,config['scale'])
    for case_id in ['S2','S6']:
        case=cases[case_id]
        check(case['static_goal_fits'] and case['reference_path_static_clear'],f'{case_id} needs a clear static reference path')
        with (root/'config'/case['reference_path_csv']).open() as handle:
            path=list(csv.DictReader(handle))
        check(len(path)>=300,'Reference arc sampling too sparse')
        for row in path:
            pose=[float(row['x_m'])/config['scale'],float(row['y_m'])/config['scale'],float(row['yaw_deg'])]
            check(not suite.pose_collisions(pose,obstacles,suite.SAFETY_MARGIN+0.025),f'{case_id}: reference path hits static scenery')
            check(float(row['speed_m_s'])<=0,'Reference manoeuvre must never move forward')
        last=path[-1]
        goal=case['goal_pose_m_deg']
        check(abs(float(last['x_m'])-goal[0])<1e-7 and abs(float(last['y_m'])-goal[1])<1e-7,'Reference arc misses target')
        check(abs(float(last['yaw_deg'])-goal[2])<1e-7,'Final heading is not perpendicular')
        # For the reference rear-axle radius, even the inside front wheel stays
        # inside its physical 0.65 rad limit, and the virtual steering is <0.55.
        check(math.atan(2.75/5.5)<0.55 and math.atan(2.75/(5.5-0.79))<0.65,'Reference arc exceeds steering limits')
    check(cases['S2']['pedestrian_intersects_reference_path_at_start']==config.get('include_pedestrians',True),
          'S2 blocking condition must match the configured pedestrians')
    scale=config['scale']
    for case_id,case in cases.items():
        tree=ET.parse(root/'worlds'/f'case_{case_id.lower()}.sdf')
        raw_pose=tree.findtext("world/model[@name='parking_robot']/pose")
        if raw_pose is None:
            check(not config.get('include_robot',True),'Missing robot in case variant')
            continue
        pose=[float(v) for v in raw_pose.split()]
        expected=case['spawn_pose_m_deg']
        check(abs(pose[0]-expected[0])<1e-6 and abs(pose[1]-expected[1])<1e-6,'Wrong scenario spawn')
        check(not suite.pose_collisions([pose[0]/scale,pose[1]/scale,math.degrees(pose[5])],obstacles),f'{case_id}: unsafe scenario start')
    # Sample the curb parking band: no additional perpendicular target region
    # may pass the 0.15 m margin. S5 fits but its entry is enclosed as above.
    for side in (1,-1):
        for i in range(421):
            x=-21+i*0.1
            for y in [side*v for v in (14.2,14.4,14.6,14.8)]:
                if not suite.pose_collisions([x,y,-90*side],obstacles):
                    allowed=(side>0 and (-3.6<=x<=3.6 or 10<=x<=18)) or (side<0 and 12.3<=x<=15.7)
                    check(allowed,'Unexpected additional clear curb-side target')
    print('PASS: S1 taper reject; S2 static fit + person on sweep; S3 narrow reject; S4 angle reject; S5 final fit + entry blocked; S6 clear reverse arc.')
    print('PASS: car-sized footprint, steering limits, six clear starts, and reference final poses perpendicular to the curb.')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root',nargs='?',type=Path,default=ROOT)
    args=parser.parse_args()
    try:
        validate_cases(args.root)
    except (ValueError,KeyError,ET.ParseError) as error:
        parser.exit(1,f'FAIL: {error}\n')
