#!/usr/bin/env python3
from pathlib import Path
import sys, json, math, time, signal
from datetime import datetime, timezone
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
from parking_system.geometry import compose, wrap
from parking_system.mapping import Scan, PoseBuffer, EncoderImuOdometry, LocalSlam, MotionTracker
from parking_system.control import Mission, StreamWatchdog
from parking_system import __version__

def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-09

def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
    from sensor_msgs.msg import LaserScan, Imu, JointState
    from nav_msgs.msg import OccupancyGrid, Path as RosPath, Odometry
    from std_msgs.msg import String
    from tf2_ros import TransformBroadcaster
    from ament_index_python.packages import get_package_share_directory
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = Node('limo_autonomous_parking', parameter_overrides=[Parameter('use_sim_time', value=True)])
    share = Path(get_package_share_directory('parking_lot_sim'))
    node.declare_parameter('config_file', str(share / 'config/autonomy.json'))
    node.declare_parameter('report_dir', str(Path.cwd() / 'parking_results'))
    cfg = json.loads(Path(node.get_parameter('config_file').value).read_text())
    g = cfg['robot']
    slam = LocalSlam(cfg['mapping'])
    dynamic = MotionTracker(cfg['safety']['dynamic_memory_s'])
    mission = Mission(cfg, slam, dynamic)
    odom = EncoderImuOdometry(cfg['start_pose'], g['wheelbase'], g['wheel_radius'])
    buffer = PoseBuffer()
    watchdog = StreamWatchdog(['imu', 'encoders', 'scan'], timeout=30)
    command = node.create_publisher(Twist, '/cmd_vel', 10)
    state_pub = node.create_publisher(String, '/parking/state', 10)
    decision_pub = node.create_publisher(String, '/parking/decisions', 10)
    pose_pub = node.create_publisher(PoseStamped, '/parking/estimated_pose', 10)
    odom_pub = node.create_publisher(Odometry, '/parking/encoder_odom', 10)
    transient = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    map_pub = node.create_publisher(OccupancyGrid, '/map', transient)
    path_pub = node.create_publisher(RosPath, '/parking/path', transient)
    tf = TransformBroadcaster(node)
    state = {'raw': None, 'pending': None, 'cloud': [], 'scan_stamp': None, 'last_wheels': None, 'speed': 0.0, 'failure': None, 'last_report': -math.inf, 'last_map': -math.inf, 'event_count': 0, 'decision_count': 0, 'stopped': False, 'last_control_sim': None, 'startup_until': None, 'first_imu_yaw': None, 'laser_range': None, 'last_status_log': -math.inf, 'last_stop_reason': None, 'last_status': None, 'last_status_publish': -math.inf, 'last_published_mode': None, 'last_logged_mode': None}
    report_root = Path(node.get_parameter('report_dir').value).expanduser()
    report_root.mkdir(parents=True, exist_ok=True)
    started_wall = time.monotonic()
    subscriptions = []

    def on_imu(msg):
        valid = np.isfinite([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w, msg.angular_velocity.z]).all()
        valid = bool(valid and 0.5 < sum((v * v for v in [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])) < 1.5)
        watchdog.feed('imu', stamp(msg), time.monotonic(), valid)
        if valid:
            try:
                measured_yaw = yaw(msg.orientation)
                if state['first_imu_yaw'] is None:
                    state['first_imu_yaw'] = measured_yaw
                odom.imu(measured_yaw, msg.angular_velocity.z, stamp(msg))
            except ValueError as e:
                state['failure'] = str(e)

    def on_joints(msg):
        values = dict(zip(msg.name, msg.position))
        names = ['rear_left_wheel_joint', 'rear_right_wheel_joint']
        valid = all((n in values and math.isfinite(values[n]) for n in names))
        t = stamp(msg)
        watchdog.feed('encoders', t, time.monotonic(), valid)
        if not valid:
            return
        try:
            pose = odom.wheels(values[names[0]], values[names[1]], t)
            if pose is not None:
                if state['last_wheels'] is not None:
                    old, old_t = state['last_wheels']
                    distance = g['wheel_radius'] * (wrap(values[names[0]] - old[0]) + wrap(values[names[1]] - old[1])) / 2
                    state['speed'] = distance / max(t - old_t, 0.001)
                state['last_wheels'] = ([values[names[0]], values[names[1]]], t)
                state['raw'] = pose
                buffer.add(t, pose)
        except ValueError as e:
            state['failure'] = str(e)

    def on_scan(msg):
        state['laser_range'] = float(msg.range_max)
        if abs(msg.range_max - cfg['mapping']['range_max']) > 0.01:
            state['failure'] = f"Laser range {msg.range_max:g} m differs from configured {cfg['mapping']['range_max']:g} m; regenerate the world and rebuild parking_lot_sim."
        scan = Scan(stamp(msg), msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment, np.asarray(msg.ranges, dtype=float), msg.range_min, msg.range_max)
        valid = scan.valid() and msg.header.frame_id.split('::')[-1].split('/')[-1] == 'lidar_link' and (abs(msg.angle_min + math.pi) < 0.02) and (abs(msg.angle_max - math.pi) < 0.02)
        watchdog.feed('scan', scan.stamp, time.monotonic(), valid)
        if valid:
            state['pending'] = scan
    subscriptions.extend([node.create_subscription(Imu, '/imu', on_imu, qos_profile_sensor_data), node.create_subscription(JointState, '/joint_states', on_joints, qos_profile_sensor_data), node.create_subscription(LaserScan, '/scan', on_scan, qos_profile_sensor_data)])
    report = {'kind': 'ROS/Gazebo runtime estimated-state report', 'simulator_truth_used_for_control': False, 'status': 'STARTING', 'trajectory': []}

    def pose_message(p, t):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = t
        msg.pose.position.x = float(p[0])
        msg.pose.position.y = float(p[1])
        msg.pose.orientation.z = math.sin(p[2] / 2)
        msg.pose.orientation.w = math.cos(p[2] / 2)
        return msg

    def publish_map(t):
        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.header.stamp = t
        msg.info.resolution = float(slam.grid.resolution)
        msg.info.width = int(slam.grid.width)
        msg.info.height = int(slam.grid.height)
        msg.info.origin.position.x = float(slam.grid.origin[0])
        msg.info.origin.position.y = float(slam.grid.origin[1])
        msg.info.origin.orientation.w = 1.0
        msg.data = slam.grid.message_data().tolist()
        map_pub.publish(msg)

    def publish_tf(raw, estimated, t):
        h = raw[2]
        c, s = (math.cos(h), math.sin(h))
        inverse = [-c * raw[0] - s * raw[1], s * raw[0] - c * raw[1], -h]
        correction = compose(estimated, inverse)
        transforms = []
        for parent, child, p in [('map', 'odom', correction), ('odom', 'base_footprint', raw)]:
            msg = TransformStamped()
            msg.header.frame_id = parent
            msg.child_frame_id = child
            msg.header.stamp = t
            msg.transform.translation.x = float(p[0])
            msg.transform.translation.y = float(p[1])
            msg.transform.rotation.z = math.sin(p[2] / 2)
            msg.transform.rotation.w = math.cos(p[2] / 2)
            transforms.append(msg)
        tf.sendTransform(transforms)

    def publish_status(now, reason=None, estimated=None, mode=None):
        detail = {'version': __version__, 'state': mode or mission.state, 'laps': mission.laps, 'goal_pose': mission.goal, 'time_s': round(now, 3), 'stop_reason': reason or mission.last_stop_reason, 'path_index': mission.tracker.index, 'blocking_point_body_m': mission.guard.blocking_point, 'first_imu_yaw_deg': None if state['first_imu_yaw'] is None else round(math.degrees(state['first_imu_yaw']), 3), 'laser_range_m': state['laser_range'], 'scan_age_s': None if state['scan_stamp'] is None else round(now - state['scan_stamp'], 3)}
        if estimated is not None:
            detail['estimated_pose'] = np.asarray(estimated).tolist()
            detail['cross_track_m'] = round(mission.tracker.deviation(estimated), 3)
        if state['raw'] is not None:
            detail['encoder_pose'] = state['raw'].tolist()
        state['last_status'] = detail
        wall = time.monotonic()
        if wall - state['last_status_publish'] > 0.25 or detail['stop_reason'] != state['last_stop_reason'] or detail['state'] != state['last_published_mode']:
            state_pub.publish(String(data=json.dumps(detail)))
            state['last_status_publish'] = wall
            state['last_published_mode'] = detail['state']
        if wall - state['last_status_log'] > 4 or detail['stop_reason'] != state['last_stop_reason'] or detail['state'] != state['last_logged_mode']:
            node.get_logger().info('status ' + json.dumps(detail))
            try:
                temporary = report_root / 'latest_status.tmp'
                temporary.write_text(json.dumps(detail, indent=2) + '\n')
                temporary.replace(report_root / 'latest_status.json')
            except OSError as error:
                node.get_logger().warning('Could not save live status: ' + str(error))
            state['last_status_log'] = wall
            state['last_stop_reason'] = detail['stop_reason']
            state['last_logged_mode'] = detail['state']
    exit_code = 1
    old = signal.signal(signal.SIGTERM, lambda signum, frame: (_ for _ in ()).throw(KeyboardInterrupt()))
    try:
        node.get_logger().info(f"Parking runtime {__version__}; config={node.get_parameter('config_file').value}; initial-spawn IMU reference retained")
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            wall = time.monotonic()
            clock = node.get_clock().now()
            now = clock.nanoseconds * 1e-09
            if state['failure']:
                raise ValueError(state['failure'])
            streams_ready = watchdog.ready(now, wall)
            if state['raw'] is None or not streams_ready:
                command.publish(Twist())
                publish_status(now, 'WAITING_FOR_FRESH_SENSORS', mode='WAITING_FOR_SENSORS')
                continue
            if state['startup_until'] is None:
                state['startup_until'] = now + cfg['safety'].get('startup_settle_s', 2.0)
            if now < state['startup_until']:
                command.publish(Twist())
                publish_status(now, 'STARTUP_SETTLING', mode='STARTUP')
                continue
            if state['pending'] is not None:
                scan = state['pending']
                paired = buffer.at(scan.stamp) if buffer.samples and scan.stamp <= buffer.samples[-1][0] else None
                if paired is not None:
                    slam.process(scan, paired, dynamic)
                    state['cloud'] = scan.points()
                    state['scan_stamp'] = scan.stamp
                    state['pending'] = None
                elif buffer.samples and scan.stamp < buffer.samples[0][0]:
                    state['pending'] = None
            if slam.pose is None:
                command.publish(Twist())
                publish_status(now, 'WAITING_FOR_TIME_ALIGNED_SCAN', mode='WAITING_FOR_SENSORS')
                if wall - started_wall > 90:
                    raise ValueError('No time-aligned wheel/IMU/laser estimate after 90 seconds')
                continue
            if state['scan_stamp'] is None or now - state['scan_stamp'] > cfg['safety']['scan_timeout']:
                command.publish(Twist())
                publish_status(now, 'WAITING_FOR_TIME_ALIGNED_SCAN', mode='WAITING_FOR_SENSORS')
                continue
            if state['last_control_sim'] is not None and now - state['last_control_sim'] < 0.045:
                continue
            state['last_control_sim'] = now
            estimated = slam.current(state['raw'])
            v, w = mission.step(estimated, state['cloud'], now, state['speed'])
            msg = Twist()
            msg.linear.x = float(v)
            msg.angular.z = float(w)
            command.publish(msg)
            pose_pub.publish(pose_message(estimated, clock.to_msg()))
            publish_tf(state['raw'], estimated, clock.to_msg())
            od = Odometry()
            od.header.frame_id = 'odom'
            od.child_frame_id = 'base_footprint'
            od.header.stamp = clock.to_msg()
            od.pose.pose = pose_message(state['raw'], clock.to_msg()).pose
            od.twist.twist.linear.x = float(state['speed'])
            odom_pub.publish(od)
            if now - state['last_map'] >= 1:
                publish_map(clock.to_msg())
                state['last_map'] = now
            for event in mission.events[state['event_count']:]:
                node.get_logger().info(json.dumps(event))
            state['event_count'] = len(mission.events)
            for decision in mission.decisions[state['decision_count']:]:
                decision_pub.publish(String(data=json.dumps(decision)))
            state['decision_count'] = len(mission.decisions)
            publish_status(now, estimated=estimated)
            if mission.plan is not None and (not state['stopped']):
                p = RosPath()
                p.header.frame_id = 'map'
                p.header.stamp = clock.to_msg()
                p.poses = [pose_message(q, clock.to_msg()) for q in mission.plan[::3]]
                path_pub.publish(p)
                state['stopped'] = True
            if now - state['last_report'] >= 0.25:
                report['trajectory'].append({'time_s': now, 'estimate': estimated.tolist(), 'encoder_odom': state['raw'].tolist(), 'command': [v, w], 'state': mission.state})
                state['last_report'] = now
            if mission.completed:
                report['status'] = 'COMPLETED_ESTIMATED_POSE'
                exit_code = 0
                break
    except KeyboardInterrupt:
        report['status'] = 'INTERRUPTED'
        exit_code = 130
    except Exception as error:
        report['status'] = 'ABORTED'
        report['error'] = str(error)
        node.get_logger().error(str(error))
        publish_status(node.get_clock().now().nanoseconds * 1e-09, str(error), mode='ABORTED')
        exit_code = 1
    finally:
        for _ in range(10):
            if rclpy.ok():
                command.publish(Twist())
                rclpy.spin_once(node, timeout_sec=0.02)
        report.update(runtime_version=__version__, last_status=state['last_status'], first_imu_yaw_rad=state['first_imu_yaw'], events=mission.events, decisions=mission.decisions, completed_laps=mission.laps, goal_pose=mission.goal, map_revision=slam.grid.revision, dynamic_samples=len(dynamic.zones), scan_processing_ms_mean=float(np.mean(slam.processing_ms)) if slam.processing_ms else None)
        directory = Path(node.get_parameter('report_dir').value).expanduser() / datetime.now(timezone.utc).strftime('run_%Y%m%d_%H%M%S_%f')
        directory.mkdir(parents=True, exist_ok=True)
        if mission.assessment_grid is not None:
            mission.assessment_grid.save(directory / 'assessment_map.npz')
        (directory / 'runtime.json').write_text(json.dumps(report, indent=2) + '\n')
        slam.grid.save(directory / 'estimated_map.npz')
        node.get_logger().info('Saved run: ' + str(directory))
        signal.signal(signal.SIGTERM, old)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code
if __name__ == '__main__':
    raise SystemExit(main())
