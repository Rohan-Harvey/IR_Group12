#!/usr/bin/env python3
import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
ROOT = Path(__file__).resolve().parents[1]
NS = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}

def check(condition, message):
    if not condition:
        raise ValueError(message)

def mesh_stats(path, expected):
    tree = ET.parse(path)
    ids = {node.get('id'): node for node in tree.iter() if node.get('id')}
    for array in tree.findall('.//c:float_array', NS):
        values = [float(v) for v in array.text.split()]
        check(len(values) == int(array.get('count')) and all(map(math.isfinite, values)), 'Invalid mesh array: ' + path.name)
    points = []
    triangles = 0
    for mesh in tree.findall('.//c:mesh', NS):
        for vertices in mesh.findall('c:vertices', NS):
            source = ids[vertices.find('c:input', NS).get('source')[1:]]
            values = [float(v) for v in source.find('c:float_array', NS).text.split()]
            points.extend(zip(values[::3], values[1::3], values[2::3]))
        for item in mesh.findall('c:triangles', NS):
            count = int(item.get('count'))
            inputs = item.findall('c:input', NS)
            stride = max((int(i.get('offset', '0')) for i in inputs)) + 1
            indices = [int(v) for v in item.find('c:p', NS).text.split()]
            check(len(indices) == count * 3 * stride, 'Invalid mesh triangle count')
            for source_input in inputs:
                source = ids[source_input.get('source')[1:]]
                if source.tag.endswith('vertices'):
                    source = ids[source.find('c:input', NS).get('source')[1:]]
                limit = int(source.find('c:technique_common/c:accessor', NS).get('count'))
                check(all((0 <= v < limit for v in indices[int(source_input.get('offset', '0'))::stride])), 'Mesh index out of bounds')
            triangles += count
    bounds = [[min((p[i] for p in points)) for i in range(3)], [max((p[i] for p in points)) for i in range(3)]]
    check(all((abs(a - b) < 1e-07 for arow, brow in zip(bounds, expected['bounds_m']) for a, b in zip(arow, brow))), 'LIMO mesh dimensions changed')
    check(triangles == expected['triangles_after'], 'Mesh report differs from bundled file')
    check(len(tree.findall('.//c:material', NS)) == expected['materials'], 'Mesh material budget exceeded')
    for node in tree.iter():
        for key in ('url', 'source', 'target'):
            value = node.get(key, '')
            if value.startswith('#'):
                check(value[1:] in ids, 'Broken mesh reference: ' + value)
    return triangles

def validate(root=ROOT):
    cfg = json.loads((root / 'config/parking_lot.json').read_text())
    mission = json.loads((root / 'config/ccw_route.json').read_text())
    report = json.loads((root / 'config/limo_meshes.json').read_text())
    check(cfg['include_pedestrians'] and cfg['pedestrian_limit'] == 1, 'Need one dynamic pedestrian')
    check(mission['lap_count'] == 2, 'Need two complete laps')
    check(list(mission['laser_frames']) == ['/scan'], 'Unexpected extra laser dependency')
    check(set(mission['disabled_sensors']) == {'camera', 'contact'}, 'Disabled sensors differ')
    counts = {name: mesh_stats(root / 'models/limo/meshes' / name, data) for name, data in report.items()}
    total = counts['limo_base.dae'] + 4 * counts['limo_wheel.dae']
    check(total <= 110000, 'LIMO visual triangle budget exceeded')
    results = {}
    for path in sorted((root / 'worlds').glob('*.sdf')):
        tree = ET.parse(path)
        world = tree.find('world')
        bare = path.name == 'parking_lot_physics_only.sdf'
        actors = world.findall('actor')
        check(len(actors) == (0 if bare else 1), 'Incorrect pedestrian count: ' + path.name)
        if actors:
            check(actors[0].findtext('script/loop') == 'true' and actors[0].findtext('script/auto_start') == 'true', 'Pedestrian animation disabled')
            for filename in actors[0].findall('.//filename'):
                check((root / 'models' / filename.text.removeprefix('model://')).is_file(), 'Missing pedestrian asset')
        meshes = tree.findall('.//mesh')
        check(len(meshes) == 5, 'Expected a LIMO body and four wheels')
        for mesh in meshes:
            check((root / 'models' / mesh.findtext('uri').removeprefix('model://')).is_file(), 'Missing LIMO mesh')
        check(world.findtext('scene/shadows') == 'false', 'Shadows enabled')
        check(len(tree.findall('.//visual')) <= 70 and len(tree.findall('.//collision')) <= 70, 'Scenery budget exceeded')
        check(abs(float(world.findtext('physics/max_step_size')) - 0.005) < 1e-09, 'Physics rate changed')
        check(not any((m.get('name', '').startswith(('tree_', 'lamp_')) for m in world.findall('model'))), 'Decorative scenery remains')
        sensors = tree.findall('.//sensor')
        robot = world.find("model[@name='parking_robot']")
        if bare:
            check(not sensors, 'Bare world has sensors')
            check(not any((p.get('name') in {'gz::sim::systems::Sensors', 'gz::sim::systems::Imu', 'gz::sim::systems::Contact'} for p in world.findall('plugin'))), 'Bare world starts sensor systems')
            check(robot.find("plugin[@name='gz::sim::systems::JointStatePublisher']") is None, 'Bare world publishes encoders')
        else:
            check(len(sensors) == 2 and {s.get('type') for s in sensors} == {'gpu_lidar', 'imu'}, 'Incorrect active sensors')
            lidar = robot.find('.//sensor[@type="gpu_lidar"]')
            check(lidar.findtext('lidar/visibility_mask') == '1', 'No self-visibility filter')
            check(int(lidar.findtext('lidar/scan/horizontal/samples')) == 180, 'Incorrect beam budget')
            check(float(lidar.findtext('update_rate')) == 5, 'Incorrect scan rate')
            check(robot.findtext("plugin[@name='gz::sim::systems::JointStatePublisher']/update_rate") == '20', 'Encoder rate is not capped')
            check(all((int(v.findtext('visibility_flags', '4294967295')) & 1 == 0 for v in robot.findall('.//visual'))), 'Robot would obstruct its scan')
        results[path.name] = {'sensors': len(sensors), 'actors': len(actors), 'mesh_instances': len(meshes)}
    check(len(results) == 9, 'Expected eight scenario/alias worlds and one bare world')
    cases = json.loads((root / 'config/parking_scenarios.json').read_text())
    check(cases['immediately_accepted_cases'] == ['S6'], 'S6 should be the only initially accepted case')
    print(f'PASS: detailed LIMO visuals, {total:,} triangles; one looping pedestrian; two CCW laps; S6 last.')
    print('PASS: nine worlds; one 5 Hz 360-degree laser + IMU; capped encoders; no camera/contact sensors or shadows.')
    print('PASS: bundled LIMO dimensions, triangle indices, materials and local asset references; sensor-free manual fallback.')
    return results
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('root', nargs='?', type=Path, default=ROOT)
    args = parser.parse_args()
    try:
        validate(args.root)
    except ValueError as error:
        parser.exit(1, 'FAIL: ' + str(error) + '\n')
