#!/usr/bin/env python3
import json
import math
from pathlib import Path
import signal
import time
from datetime import datetime, timezone
from ccw_navigation import MissionFollower, StreamWatchdog, WheelImuOdometry, wrap

def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan, Imu, JointState
    from tf2_msgs.msg import TFMessage
    from ament_index_python.packages import get_package_share_directory
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = Node('limo_ccw_demo', parameter_overrides=[Parameter('use_sim_time', value=True)])
    source_root = Path(__file__).resolve().parents[1]
    try:
        share = Path(get_package_share_directory('parking_lot_sim'))
    except Exception:
        share = source_root
    node.declare_parameter('route_file', str(share / 'config/ccw_route.json'))
    node.declare_parameter('report_dir', str(Path.cwd() / 'limo_ccw_results'))
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)
    report = {'status': 'starting', 'controller': 'prescribed CCW demonstration', 'simulator_truth_used_for_control': False, 'live_world_pose_verified': False, 'error': None, 'events': [], 'sensor_counts': {}, 'wheel_rotation_rad': {}, 'trajectory': []}
    subscriptions = []
    state = {'body': None, 'anchor': None, 'speed': 0, 'clouds': {}, 'truth': None, 'imu_valid': False, 'joints': {}}
    exit_code = 1
    follower = None
    mission = None
    old_term = signal.signal(signal.SIGTERM, lambda signum, frame: (_ for _ in ()).throw(KeyboardInterrupt()))

    def count(name):
        report['sensor_counts'][name] = report['sensor_counts'].get(name, 0) + 1

    def stamp(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-09
    try:
        mission = json.loads(Path(node.get_parameter('route_file').value).read_text())
        g = mission['geometry']
        spawn = mission['start_pose_m_rad']
        goal = mission['goal_pose_m_rad']
        follower = MissionFollower(mission)
        estimator = WheelImuOdometry(spawn, g['wheelbase'], g['wheel_radius'])
        watchdog = StreamWatchdog(['odom', 'imu', 'joint_states', *mission['laser_frames']], timeout=30)
        report['pose_estimator'] = 'rear-wheel encoders plus IMU yaw, anchored at the demonstration spawn'
        report['geometry'] = g
        report['goal_pose_m_rad'] = goal

        def on_odom(msg):
            p = msg.pose.pose
            q = p.orientation
            yaw = yaw_of(q)
            valid = all((math.isfinite(v) for v in [p.position.x, p.position.y, yaw, msg.twist.twist.linear.x]))
            watchdog.feed('odom', stamp(msg), time.monotonic(), valid)
            count('odom')
            if not valid:
                return
            if state['anchor'] is None:
                state['anchor'] = (p.position.x, p.position.y, yaw)
            ax, ay, ah = state['anchor']
            angle = spawn[2] - ah
            c, s = (math.cos(angle), math.sin(angle))
            dx, dy = (p.position.x - ax, p.position.y - ay)
            state['raw_odom_pose'] = (spawn[0] + c * dx - s * dy, spawn[1] + s * dx + c * dy, yaw + angle)
            state['speed'] = msg.twist.twist.linear.x

        def on_scan(msg, topic):
            mount = mission['laser_frames'][topic]
            points = []
            leaf = lambda frame: frame.split('::')[-1].split('/')[-1]
            valid = leaf(msg.header.frame_id) == leaf(mount['frame']) and len(msg.ranges) >= 30 and (msg.angle_increment > 0) and (msg.range_max > msg.range_min >= 0 and abs(msg.angle_min - mount['angle_min']) < 0.02 and (abs(msg.angle_max - mount['angle_max']) < 0.02))
            for i, r in enumerate(msg.ranges):
                if r == math.inf:
                    continue
                if not math.isfinite(r) or r < 0 or r > msg.range_max:
                    valid = False
                    continue
                angle = mount['yaw'] + msg.angle_min + i * msg.angle_increment
                points.append((mount['x'] + r * math.cos(angle), mount['y'] + r * math.sin(angle)))
            state['clouds'][topic] = points
            watchdog.feed(topic, stamp(msg), time.monotonic(), valid)
            count(topic)

        def on_imu(msg):
            q = msg.orientation
            values = [q.x, q.y, q.z, q.w, msg.angular_velocity.z]
            state['imu_valid'] = all((math.isfinite(v) for v in values)) and 0.5 < sum((v * v for v in values[:4])) < 1.5
            watchdog.feed('imu', stamp(msg), time.monotonic(), state['imu_valid'])
            if state['imu_valid']:
                estimator.imu(yaw_of(q), msg.angular_velocity.z, stamp(msg))
            count('imu')

        def on_joints(msg):
            count('joint_states')
            samples = dict(zip(msg.name, msg.position))
            names = ['rear_left_wheel_joint', 'rear_right_wheel_joint']
            valid = all((n in samples and math.isfinite(samples[n]) for n in names))
            watchdog.feed('joint_states', stamp(msg), time.monotonic(), valid)
            if valid:
                pose = estimator.wheels(samples[names[0]], samples[names[1]], stamp(msg))
                if pose is not None:
                    state['body'] = pose
            for name, position in zip(msg.name, msg.position):
                if not math.isfinite(position):
                    continue
                if name.endswith('_wheel_joint'):
                    if name in state['joints']:
                        total = report['wheel_rotation_rad'].get(name, 0)
                        report['wheel_rotation_rad'][name] = total + abs(wrap(position - state['joints'][name]))
                    state['joints'][name] = position
                elif name.endswith('_steering_joint'):
                    bounds = report.setdefault('steering_ranges_rad', {}).setdefault(name, [position, position])
                    bounds[0] = min(bounds[0], position)
                    bounds[1] = max(bounds[1], position)

        def on_truth(msg):
            for transform in msg.transforms:
                if transform.child_frame_id.split('::')[-1].split('/')[-1] != 'parking_robot':
                    continue
                p = transform.transform.translation
                q = transform.transform.rotation
                state['truth'] = {'pose': [p.x, p.y, yaw_of(q)], 'stamp': stamp(transform)}
                count('evaluation_pose')
        subscriptions.append(node.create_subscription(Odometry, '/odom', on_odom, qos_profile_sensor_data))
        for topic in mission['laser_frames']:
            subscriptions.append(node.create_subscription(LaserScan, topic, lambda m, t=topic: on_scan(m, t), qos_profile_sensor_data))
        subscriptions.extend([node.create_subscription(Imu, '/imu', on_imu, qos_profile_sensor_data), node.create_subscription(JointState, '/joint_states', on_joints, qos_profile_sensor_data), node.create_subscription(TFMessage, '/verification/robot_pose', on_truth, qos_profile_sensor_data)])
        node.get_logger().info('Entrance -> two CCW laps (S2 -> S1 -> S3 -> S4 -> S5 -> S6), then reverse park')
        report['status'] = 'running'
        last_sim = None
        clock_wall = time.monotonic()
        last_sample = -math.inf
        last_event = 0
        last_log = 0
        last_control = 0
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if now - last_control < 0.05:
                continue
            last_control = now
            sim = node.get_clock().now().nanoseconds * 1e-09
            if last_sim is None or sim > last_sim + 1e-08:
                clock_wall = now
            last_sim = sim
            if node.count_publishers('/cmd_vel') > 1:
                raise ValueError('Another /cmd_vel publisher is active; run one controller')
            ready = watchdog.ready(sim, now)
            if not ready or state['body'] is None or now - clock_wall > 2:
                publisher.publish(Twist())
                if now - last_log > 5:
                    node.get_logger().info('Waiting for fresh clock, odometry, IMU, encoders and the 360-degree laser')
                    last_log = now
                continue
            cloud = [p for points in state['clouds'].values() for p in points]
            if not cloud:
                raise ValueError('The scan has no environment returns; check sensor visibility')
            v, w = follower.step(state['body'], cloud, sim, state['speed'])
            command = Twist()
            command.linear.x = v
            command.angular.z = w
            publisher.publish(command)
            if sim - last_sample >= 0.25:
                report['trajectory'].append({'time_s': sim, 'pose_m_rad': list(state['body']), 'command': [v, w], 'phase': follower.phase})
                last_sample = sim
            for event, event_time in follower.events[last_event:]:
                node.get_logger().info(event)
            last_event = len(follower.events)
            if follower.completed:
                stop_time = sim
                stop_wall = time.monotonic()
                stop_last = sim
                stop_progress = stop_wall
                while rclpy.ok() and node.get_clock().now().nanoseconds * 1e-09 - stop_time < 1.0:
                    publisher.publish(Twist())
                    rclpy.spin_once(node, timeout_sec=0.05)
                    current_sim = node.get_clock().now().nanoseconds * 1e-09
                    if current_sim > stop_last:
                        stop_last = current_sim
                        stop_progress = time.monotonic()
                    if time.monotonic() - stop_progress > 2 or time.monotonic() - stop_wall > 180:
                        break
                report['final_odometry_pose_m_rad'] = list(state['body'])
                report['final_plugin_odometry_pose_m_rad'] = state.get('raw_odom_pose')
                report['status'] = 'completed_odometry_only'
                exit_code = 0
                truth = state['truth']
                now_sim = node.get_clock().now().nanoseconds * 1e-09
                if truth and -0.1 <= now_sim - truth['stamp'] < 1:
                    p = truth['pose']
                    position_error = math.dist(p[:2], goal[:2])
                    heading_error = abs(wrap(p[2] - goal[2]))
                    report['final_simulator_pose_m_rad'] = p
                    report['world_position_error_m'] = position_error
                    report['world_heading_error_deg'] = math.degrees(heading_error)
                    report['live_world_pose_verified'] = position_error <= 0.35 * g['scale'] and heading_error <= math.radians(10)
                    if report['live_world_pose_verified']:
                        report['status'] = 'completed_world_pose_verified'
                    else:
                        report['status'] = 'failed_world_pose_verification'
                        exit_code = 2
                node.get_logger().info(report['status'])
                break
    except KeyboardInterrupt:
        report['status'] = 'interrupted'
        exit_code = 130
    except Exception as error:
        report['status'] = 'aborted'
        report['error'] = str(error)
        node.get_logger().error(str(error))
        exit_code = 1
    finally:
        for _ in range(10):
            if rclpy.ok():
                publisher.publish(Twist())
                rclpy.spin_once(node, timeout_sec=0.05)
        report['events'] = follower.events if follower else []
        report['completed_laps'] = follower.completed_laps if follower else 0
        report['imu_valid'] = state['imu_valid']
        wheel_names = [f'{end}_{side}_wheel_joint' for end in ['front', 'rear'] for side in ['left', 'right']]
        report['all_four_wheels_moved'] = all((report['wheel_rotation_rad'].get(name, 0) > 0.5 for name in wheel_names))
        report['disabled_sensors'] = mission.get('disabled_sensors', [])
        report['camera_valid'] = None
        report['contacts_detected'] = None
        report['sensor_health_passed'] = state['imu_valid'] and report['all_four_wheels_moved'] and all((report['sensor_counts'].get(t, 0) > 0 for t in mission['laser_frames']))
        report['scope'] = 'Prescribed-route run. Odom-only completion does not certify actual parking. Simulator poses are evaluation-only. One dynamic walking actor; camera/contact sensors remain disabled for performance. Actor contacts are not physical.'
        try:
            directory = Path(node.get_parameter('report_dir').value).expanduser()
            directory.mkdir(parents=True, exist_ok=True)
            path = directory / ('ccw_' + datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S_%f') + '.json')
            path.write_text(json.dumps(report, indent=2) + '\n')
            node.get_logger().info('Run report: ' + str(path))
        finally:
            signal.signal(signal.SIGTERM, old_term)
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    return exit_code
if __name__ == '__main__':
    raise SystemExit(main())
