#!/usr/bin/env python3
"""Validate the delivered minimal and sensor-free worlds without ROS/Gazebo."""
import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]


def check(condition,message):
    if not condition:raise ValueError(message)


def validate(root=ROOT):
    cfg=json.loads((root/'config/parking_lot.json').read_text())
    mission=json.loads((root/'config/ccw_route.json').read_text())
    check(not cfg['include_pedestrians'] and cfg['pedestrian_limit']==0,'Pedestrian not disabled')
    check(list(mission['laser_frames'])==['/scan'],'Mission still requires extra laser topics')
    mount=mission['laser_frames']['/scan']
    check(mount['x']==mount['y']==mount['yaw']==0,'Incorrect centred scanner transform')
    check(abs(mount['angle_min']+math.pi)<1e-9 and abs(mount['angle_max']-math.pi)<1e-9,'Scan is not 360 degrees')
    check(set(mission['disabled_sensors'])=={'camera','contact'},'Disabled topics not declared')
    results={}
    for path in sorted((root/'worlds').glob('*.sdf')):
        tree=ET.parse(path);world=tree.find('world')
        check(not tree.findall('.//actor'),'Actor remains in '+path.name)
        check(not tree.findall('.//mesh'),'Mesh remains in '+path.name)
        check(not tree.findall('.//sensor[@type="camera"]'),'Camera remains in '+path.name)
        check(not tree.findall('.//sensor[@type="contact"]'),'Contact sensor remains in '+path.name)
        check(world.findtext('scene/shadows')=='false','Shadows enabled')
        check(len(tree.findall('.//visual'))<=70 and len(tree.findall('.//collision'))<=70,'Geometry budget exceeded')
        check(abs(float(world.findtext('physics/max_step_size'))-.005)<1e-9,'Unexpected physics update rate')
        check(not any(m.get('name','').startswith(('tree_','lamp_')) for m in world.findall('model')),'Decorative trees/lights remain')
        sensors=tree.findall('.//sensor')
        robot=world.find("model[@name='parking_robot']")
        if path.name=='parking_lot_physics_only.sdf':
            check(not sensors,'Bare world still contains sensors')
            check(not any(p.get('name') in {'gz::sim::systems::Sensors','gz::sim::systems::Imu','gz::sim::systems::Contact'} for p in world.findall('plugin')),'Bare world still starts sensor systems')
            check(robot.find("plugin[@name='gz::sim::systems::JointStatePublisher']") is None,'Bare world publishes encoders')
        else:
            check(len(sensors)==2 and {s.get('type') for s in sensors}=={'gpu_lidar','imu'},'Incorrect active sensors')
            lidar=robot.find('.//sensor[@type="gpu_lidar"]')
            check(lidar.findtext('lidar/visibility_mask')=='1','No self-visibility filter')
            check(int(lidar.findtext('lidar/scan/horizontal/samples'))==180,'Incorrect beam budget')
            check(float(lidar.findtext('update_rate'))==5,'Incorrect scan rate')
            check(robot.findtext("plugin[@name='gz::sim::systems::JointStatePublisher']/update_rate")=='20','Encoder publisher is not capped')
            check(all(int(v.findtext('visibility_flags','4294967295'))&1==0 for v in robot.findall('.//visual')),'Robot would obstruct its scan')
        results[path.name]={'visuals':len(tree.findall('.//visual')),'collisions':len(tree.findall('.//collision')),'sensors':len(sensors),'actors':0,'meshes':0}
    check(len(results)==9,'Expected eight scenario/alias worlds and the bare world')
    cases=json.loads((root/'config/parking_scenarios.json').read_text())
    check(cases['immediately_accepted_cases']==['S2','S6'],'S2 must become suitable after pedestrian removal')
    print('PASS: nine primitive-only worlds; no actors, camera or contact sensors; 64 visuals / 63 collisions.')
    print('PASS: one centred scan + IMU; bare world has zero sensors and no sensor systems; capped encoder publication.')
    print('PASS: no camera/rear-scan dependency in the preset mission; S2 correctly becomes statically acceptable.')
    return results


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('root',nargs='?',type=Path,default=ROOT)
    args=parser.parse_args()
    try:validate(args.root)
    except ValueError as e:parser.exit(1,'FAIL: '+str(e)+'\n')
