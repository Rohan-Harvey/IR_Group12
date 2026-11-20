"""ROS conversions shared by the project parking processes."""

import math
import signal

from geometry_msgs.msg import PoseStamped, Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.time import Time
from tf2_ros import TransformException

from patrol_route import Point


SCAN_TOPICS = ("/scan", "/scan_low_front", "/scan_low_rear")


def stamp_seconds(stamp):
    return stamp.sec+stamp.nanosec*1e-9


def yaw(quaternion):
    return math.atan2(2*(quaternion.w*quaternion.z+quaternion.x*quaternion.y),
                      1-2*(quaternion.y**2+quaternion.z**2))


def xyz_yaw(transform):
    return (transform.translation.x, transform.translation.y, yaw(transform.rotation))


def from_pose(pose):
    return Point(pose.position.x, pose.position.y, yaw(pose.orientation), 0.0)


def pose_message(pose, frame, stamp):
    message = PoseStamped()
    message.header.frame_id = frame
    message.header.stamp = stamp
    message.pose.position.x, message.pose.position.y = pose.x, pose.y
    message.pose.orientation.z = math.sin(pose.yaw/2)
    message.pose.orientation.w = math.cos(pose.yaw/2)
    return message


def twist(v=0.0, w=0.0):
    output = Twist()
    output.linear.x = v
    output.angular.z = w
    return output


def scan_pose(buffer, scan):
    try:
        transform = buffer.lookup_transform("odom", scan.header.frame_id,
                                            Time.from_msg(scan.header.stamp))
    except TransformException:
        return None
    return Point(*xyz_yaw(transform.transform), 0.0)


def spin_until_stopped(node):
    """Handle shutdown between executor callbacks and let the caller flush evidence."""
    stopping = False

    def stop(_signum, _frame):
        nonlocal stopping
        stopping = True

    signals = (signal.SIGINT, signal.SIGTERM)
    previous = [signal.signal(number, stop) for number in signals]
    executor = SingleThreadedExecutor(context=node.context)
    executor.add_node(node)
    try:
        while node.context.ok() and not stopping:
            executor.spin_once(timeout_sec=0.15)
    finally:
        executor.remove_node(node)
        executor.shutdown()
        for number, handler in zip(signals, previous):
            signal.signal(number, handler)
