#!/usr/bin/env python3
"""Check that the frozen frame tree resolves. Exits 0 if every hop does.

    ros2 run agc_slam check_frames ugv1
    ros2 run agc_slam check_frames ugv1 --timeout 10

Why this is not `ros2 run tf2_ros tf2_echo` in a shell script: tf2_echo is
built for a human reading a terminal, so a script has to scrape its output,
and its buffer is driven by the wall clock. Replaying a bag recorded in April
against an October system clock makes it spin on "Detected jump back in time.
Clearing TF buffer." and report a missing transform for a frame tree that is
perfectly fine.

This asks tf2 the question the consumers actually ask -- can the latest
available transform be resolved -- via Buffer.can_transform at
TimePointZero, which is time-domain agnostic.
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from tf2_ros import Buffer, TransformListener


class FrameCheck(Node):

    def __init__(self):
        super().__init__('agc_frame_check')
        # Match whatever clock the rest of the system is on, so a bag replay
        # and a live sim both work without special-casing.
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

    def resolve(self, parent, child, timeout):
        """Spin until the latest transform parent->child is available."""
        deadline = time.monotonic() + timeout
        last_error = 'no transform received'
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            try:
                if self.buffer.can_transform(parent, child, Time()):
                    t = self.buffer.lookup_transform(parent, child, Time())
                    return True, t
            except Exception as exc:  # tf2 raises several distinct types
                last_error = str(exc) or type(exc).__name__
        return False, last_error


def main(argv=None):
    argv = rclpy.utilities.remove_ros_args(sys.argv if argv is None else argv)
    parser = argparse.ArgumentParser(prog='check_frames')
    parser.add_argument('robot', nargs='?', default='ugv1')
    parser.add_argument('--map-frame', default='map')
    parser.add_argument('--timeout', type=float, default=10.0,
                        help='seconds to wait for each hop')
    args = parser.parse_args(argv[1:])

    hops = [
        (args.map_frame, f'{args.robot}/odom'),
        (f'{args.robot}/odom', f'{args.robot}/base_link'),
        (args.map_frame, f'{args.robot}/base_link'),
    ]

    rclpy.init()
    node = FrameCheck()
    failed = 0
    try:
        for parent, child in hops:
            ok, detail = node.resolve(parent, child, args.timeout)
            if ok:
                tr = detail.transform.translation
                print(f'  PASS  {parent} -> {child}  '
                      f'[{tr.x:.3f}, {tr.y:.3f}, {tr.z:.3f}]')
            else:
                failed += 1
                print(f'  FAIL  {parent} -> {child}')
                print(f'        {detail}')
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
