#!/usr/bin/env python3
from pathlib import Path
import sys, json, math
from datetime import datetime, timezone
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from parking_system.geometry import rectangle, intersects, wrap

def main(args=None):
    import rclpy
    from rclpy.node import Node
    from tf2_msgs.msg import TFMessage
    from std_msgs.msg import String
    from ament_index_python.packages import get_package_share_directory
    rclpy.init(args=args)
    node = Node('parking_independent_evaluator')
    node.declare_parameter('report_dir', str(Path.cwd() / 'parking_results'))
    share = Path(get_package_share_directory('parking_lot_sim'))
    sys.path.insert(0, str(share / 'scripts'))
    import scene_scenarios
    cfg = json.loads((share / 'config/parking_lot.json').read_text())
    runtime = json.loads((share / 'config/autonomy.json').read_text())
    geometry = scene_scenarios.obstacle_polygons(scene_scenarios.make_layout(cfg))
    g = runtime['robot']
    records = []
    state = {'goal': None, 'state': 'STARTING', 'saved': False}
    pub = node.create_publisher(String, '/parking/evaluation', 10)

    def save():
        if state['saved']:
            return
        directory = Path(node.get_parameter('report_dir').value).expanduser()
        directory.mkdir(parents=True, exist_ok=True)
        result = {'kind': 'Independent simulator-pose geometry evaluation; physical contact messages are not measured', 'runtime_truth_feedback': False, 'records': records, 'final_state': state['state']}
        if records and state['goal']:
            pose = records[-1]['pose']
            goal = state['goal']
            result.update(position_error_m=math.dist(pose[:2], goal[:2]), heading_error_deg=math.degrees(abs(wrap(pose[2] - goal[2]))), geometric_overlap_samples=sum((bool(p['overlaps']) for p in records)))
            result['live_world_pose_verified'] = state['state'] == 'COMPLETE' and result['position_error_m'] <= 0.3 and (result['heading_error_deg'] <= 5) and (result['geometric_overlap_samples'] == 0)
        path = directory / ('evaluation_' + datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S_%f') + '.json')
        path.write_text(json.dumps(result, indent=2) + '\n')
        state['saved'] = True

    def on_state(msg):
        data = json.loads(msg.data)
        state['goal'] = data.get('goal_pose')
        state['state'] = data['state']
        if data['state'] == 'COMPLETE':
            save()

    def on_pose(msg):
        for t in msg.transforms:
            if t.child_frame_id.split('::')[-1].split('/')[-1] != 'parking_robot':
                continue
            p = t.transform.translation
            q = t.transform.rotation
            h = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            pose = [p.x, p.y, h]
            footprint = rectangle(pose, g['length'], g['width'])
            hits = [name for name, poly in geometry if intersects(footprint, poly)]
            record = {'time_s': t.header.stamp.sec + t.header.stamp.nanosec * 1e-09, 'pose': pose, 'overlaps': hits}
            records.append(record)
            pub.publish(String(data=json.dumps(record)))
    subs = [node.create_subscription(TFMessage, '/evaluation/robot_pose', on_pose, 10), node.create_subscription(String, '/parking/state', on_state, 10)]
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        save()
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()
