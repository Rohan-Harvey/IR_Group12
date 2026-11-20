#!/usr/bin/env python3
from pathlib import Path
import json, sys, xml.etree.ElementTree as ET
import yaml
ROOT = Path(__file__).resolve().parents[1]

def validate(root=ROOT):
    for path in [*root.glob('scripts/*.py'), *root.glob('parking_system/*.py'), *root.glob('launch/*.py'), *root.glob('tests/*.py')]:
        compile(path.read_text(), str(path), 'exec')
    cfg = json.loads((root / 'config/autonomy.json').read_text())
    assert cfg['laps'] == 2
    truth = json.loads((root / 'config/scene_ground_truth.json').read_text())
    dimensions = json.loads((root / 'config/parking_scenarios.json').read_text())['parked_vehicle_dimensions_m']
    for bay in truth['bays']:
        assert all((abs(a - b) < 1e-06 for a, b in zip(bay['car']['envelope_m'], dimensions)))
    for path in (root / 'worlds').glob('*.sdf'):
        tree = ET.parse(path)
        world = tree.find('world')
        assert world is not None
        names = [node.get('name') for node in world.findall('model')]
        assert len(names) == len(set(names))
        for uri in tree.findall('.//uri'):
            assert uri.text.startswith('model://')
            assert (root / 'models' / uri.text.removeprefix('model://')).is_file(), uri.text
        for actor in world.findall('actor'):
            assert actor.findtext('script/loop') == 'true'
            assert actor.findtext('script/auto_start') == 'true'
            for filename in actor.findall('.//filename'):
                assert (root / 'models' / filename.text.removeprefix('model://')).is_file()
        bare = path.name == 'parking_lot_physics_only.sdf'
        assert len(world.findall('actor')) == (0 if bare else 1)
        assert len(tree.findall('.//sensor')) == (0 if bare else 2)
        assert not tree.findall('.//sensor[@type="camera"]') and (not tree.findall('.//sensor[@type="contact"]'))
        assert world.findtext('scene/shadows') == 'false'
        if not bare:
            laser = tree.find('.//sensor[@type="gpu_lidar"]')
            assert laser.findtext('update_rate') == '5'
            assert laser.findtext('lidar/scan/horizontal/samples') == '180'
            assert laser.findtext('lidar/visibility_mask') == '1'
            assert abs(float(laser.findtext('lidar/range/max')) - cfg['mapping']['range_max']) < 1e-06
            assert abs(float(laser.findtext('lidar/range/min')) - 0.15) < 1e-06
            assert abs(float(laser.findtext('lidar/noise/stddev')) - cfg['noise']['range_std_m']) < 1e-06
        assert len(world.findall("model[@name='parking_robot']/.//mesh")) == 5
    urdf = ET.parse(root / 'urdf/parking_robot.urdf')
    for mesh in urdf.findall('.//mesh'):
        assert (root / mesh.get('filename').removeprefix('package://parking_lot_sim/')).is_file()
    regular = yaml.safe_load((root / 'config/robot_bridge.yaml').read_text())
    evaluation = yaml.safe_load((root / 'config/verification_bridge.yaml').read_text())
    assert {x['ros_topic_name'] for x in regular} == {'/cmd_vel', '/scan', '/imu', '/joint_states'}
    assert evaluation[0]['ros_topic_name'] == '/evaluation/robot_pose'
    assert '/evaluation/robot_pose' not in (root / 'scripts/drive_project.py').read_text()
    for name in ['python3-numpy', 'python3-scipy', 'tf2_ros', 'std_msgs', 'nav_msgs', 'sensor_msgs']:
        assert name in [x.text for x in ET.parse(root / 'package.xml').findall('exec_depend')]
    print('PASS: source syntax, LIMO scale, local assets, three worlds, sensor budgets, bridge contracts, dependency declarations and evaluation separation.')
    print('ROS/Gazebo/colcon/CMake live build and physical execution are not available in this workspace.')
if __name__ == '__main__':
    validate()
