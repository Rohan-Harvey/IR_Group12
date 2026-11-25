#!/usr/bin/env python3
"""W/A/S/D controls with a short command timeout; run in a real terminal."""

import select
import sys
import termios
import time
import tty


def velocity_for_key(key, speed=0.4, turn=0.07):
    return {"w": (speed, 0.0), "s": (-speed, 0.0),
            "a": (speed, turn), "d": (speed, -turn),
            "z": (-speed,-turn),"c":(-speed,turn),
            " ": (0.0, 0.0), "x": (0.0, 0.0)}.get(key)


def main():
    import rclpy
    from geometry_msgs.msg import Twist
    if not sys.stdin.isatty():
        raise SystemExit("Open a terminal to run keyboard control.")
    rclpy.init()
    node = rclpy.create_node("parking_robot_keyboard")
    publisher = node.create_publisher(Twist, "/cmd_vel", 10)
    settings = termios.tcgetattr(sys.stdin)
    forward, angular, last_key = 0.0, 0.0, 0.0
    print("W forward | S reverse | A/D forward turn | Z/C reverse turn | SPACE/X stop | Q quit")
    print("Keep pressing / holding a movement key. Commands stop after 0.4 s without input.")
    try:
        tty.setraw(sys.stdin.fileno())
        while rclpy.ok():
            readable, _, _ = select.select([sys.stdin], [], [], 0.05)
            if readable:
                key = sys.stdin.read(1).lower()
                if key in ("q", "\x03"):
                    break
                velocity = velocity_for_key(key)
                if velocity is not None:
                    forward, angular = velocity
                    last_key = time.monotonic()
            if time.monotonic() - last_key > 0.4:
                forward, angular = 0.0, 0.0
            message = Twist()
            message.linear.x, message.angular.z = forward, angular
            publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=0)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        try:
            if rclpy.ok():
                publisher.publish(Twist())
                rclpy.spin_once(node, timeout_sec=0.05)
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        print("\nRobot stopped.")


if __name__ == "__main__":
    main()
