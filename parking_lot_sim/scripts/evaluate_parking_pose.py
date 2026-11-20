#!/usr/bin/env python3
"""Score a recorded WORLD pose offline; never use this reference data for navigation.

Example: python3 scripts/evaluate_parking_pose.py S6 --x 14 --y 14.6 --yaw-deg -90
"""

import argparse
import json
import math
from pathlib import Path
import scene_scenarios as suite
from scene_geometry import overlaps, rectangle

ROOT=Path(__file__).resolve().parents[1]


def evaluate(root,case_id,x,y,yaw,pedestrian_clear=False):
    truth=json.loads((root/'config/scene_ground_truth.json').read_text())
    robot=truth["robot"]
    if robot and robot.get("platform")=="AgileX LIMO":
        case = next(item for item in truth["scenarios"] if item["id"] == case_id)
        length, width = robot["footprint_size_m"]
        footprint = rectangle((x, y), length, width, math.radians(yaw))
        x0, y0, x1, y1 = case["target_zone_xyxy_m"]
        fits_zone = all(x0 <= px <= x1 and y0 <= py <= y1 for px, py in footprint)
        heading_error = abs((yaw-case["goal_pose_m_deg"][2]+180)%360-180)
        static = suite.obstacle_polygons([bay for bay in truth["bays"] if bay["occupied"]])
        contacts = [name for name, obstacle in static if overlaps(footprint, obstacle)]
        return {"case": case_id, "inside_reference_zone": fits_zone,
                "heading_error_deg": heading_error, "static_footprint_overlaps": contacts,
                "accepted_final_pose": fits_zone and heading_error <= 10 and not contacts,
                "source": "current LIMO geometry; x and y locate the body rectangle center",
                "sweep_outcome": "evaluate the recorded motion with evaluate_run.py"}
    config=json.loads((root/'config/parking_lot.json').read_text())
    manifest=json.loads((root/'config/parking_scenarios.json').read_text())
    case=next(c for c in manifest['cases'] if c['id']==case_id)
    scale=config['scale']
    goal=case['goal_pose_m_deg']
    yaw_error=abs((yaw-goal[2]+180)%360-180)
    errors=[]
    if yaw_error>manifest['maximum_heading_error_deg']:
        errors.append('heading exceeds 10 degrees from the required perpendicular pose')
    if math.hypot(x-goal[0],y-goal[1])>0.50*scale:
        errors.append('position is more than 0.50 m from the goal center')
    if suite.pose_collisions([x/scale,y/scale,yaw],suite.obstacle_polygons(suite.make_layout(config))):
        errors.append('vehicle footprint or safety margin intersects scenery')
    if case_id=='S5':
        errors.append('final placement fits, but the reverse-entry throat is blocked')
    if case_id=='S2' and config.get('include_pedestrians',True) and not pedestrian_clear:
        errors.append('pedestrian has not been confirmed clear of the actual planned swept path')
    return {'case':case_id,'accepted_final_pose_and_case_conditions':not errors,
            'heading_error_deg':yaw_error,'reasons':errors,
            'scope':'Offline pose/case check. Does not certify tracking, motion safety, time or contact history.'}


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('case',choices=[f'S{i}' for i in range(1,7)])
    parser.add_argument('--root',type=Path,default=ROOT)
    parser.add_argument('--x',type=float,required=True);parser.add_argument('--y',type=float,required=True)
    parser.add_argument('--yaw-deg',type=float,required=True)
    parser.add_argument('--pedestrian-clear',action='store_true',help='Use only after verifying the person is clear of the planned swept path')
    args=parser.parse_args()
    if not all(math.isfinite(v) for v in (args.x,args.y,args.yaw_deg)):
        parser.error('Pose values must be finite')
    print(json.dumps(evaluate(args.root,args.case,args.x,args.y,args.yaw_deg,args.pedestrian_clear),indent=2))
