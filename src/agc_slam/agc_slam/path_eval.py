#!/usr/bin/env python3
"""Plot a SLAM path against simulator ground truth from one recorded bag.

    ros2 run agc_slam plot_path_vs_truth BAG OUT.png --origin -2 -6 0
    ros2 run agc_slam plot_path_vs_truth BAG OUT.png --origin -2 -6 0 --site mrs_test_site

The bag needs the SLAM odometry (default ``/ugv1/odom``) and a ground-truth
``tf2_msgs/TFMessage`` topic carrying ``<world frame> -> <robot model>``, which
is what bridging Gazebo's ``/model/<name>/pose`` (``Pose_V``) gives.

``--origin X Y YAW`` is the robot's world pose when SLAM started: SLAM's odom
frame starts there, so truth is moved into that frame by this pose only (no
best-fit alignment). That makes the error slightly conservative compared with
a standard ATE, and it's the right comparison for "how far has odometry
drifted from where the robot really is".

Error metrics are a plain function (``position_errors``) so they're
unit-tested without a bag or matplotlib.
"""

import argparse
import math
import sys

import numpy as np

# Walls and obstacles of mrs_ws's mrs_test_site world (mrs_sim/tools/make_test_site.py),
# world coordinates in metres, drawn under the paths with --site mrs_test_site.
SITES = {
    'mrs_test_site': {
        'walls': [((10, -10), (10, 0)), ((10, 3), (10, 14)),  # building west wall, 3 m door
                  ((10, 0), (27, 0)), ((10, 3), (27, 3))],    # corridor
        'columns': [(-3, 6), (6, 2), (-9, -1)],
        'crates': [(-6, -4), (-8, 8), (4, 9)],
        'labels': [('door', (10, 1.5), (-34, -4)), ('corridor', (20, 1.5), (0, -26))],
    },
}

SURFACE, INK, INK2, GRID, MUTED = '#fcfcfb', '#0b0b0b', '#52514e', '#e6e5e0', '#c9c8c1'
TRUTH_COLOR, SLAM_COLOR = '#2a78d6', '#eb6834'  # validated categorical pair (blue, orange)


def world_to_odom(xy, origin):
    """Move world (x, y) points into the odom frame that starts at origin (x, y, yaw)."""
    x0, y0, yaw0 = origin
    c, s = math.cos(yaw0), math.sin(yaw0)
    d = np.asarray(xy, dtype=float) - (x0, y0)
    return np.column_stack((c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]))


def position_errors(slam, truth, origin):
    """Compare a SLAM path with ground truth.

    slam:   (N, 3) rows of (t, x, y) in the odom frame
    truth:  (M, 3) rows of (t, x, y) in the world frame
    origin: (x, y, yaw) world pose where the odom frame starts

    Truth is interpolated at each SLAM timestamp. Returns a dict with the
    truth path in the odom frame ('truth_xy'), the per-sample error in metres
    ('err'), time from the first sample ('t'), path length, RMSE, mean, max
    and final error.
    """
    slam = np.asarray(slam, dtype=float)
    truth = np.asarray(truth, dtype=float)
    slam = slam[np.argsort(slam[:, 0])]
    truth = truth[np.argsort(truth[:, 0])]
    t_odom = world_to_odom(truth[:, 1:3], origin)
    tx = np.interp(slam[:, 0], truth[:, 0], t_odom[:, 0])
    ty = np.interp(slam[:, 0], truth[:, 0], t_odom[:, 1])
    err = np.hypot(slam[:, 1] - tx, slam[:, 2] - ty)
    return {
        't': slam[:, 0] - slam[0, 0],
        'slam_xy': slam[:, 1:3],
        'truth_xy': np.column_stack((tx, ty)),
        'err': err,
        'distance': float(np.sum(np.hypot(np.diff(tx), np.diff(ty)))),
        'rmse': float(math.sqrt(np.mean(err ** 2))),
        'mean': float(err.mean()),
        'max': float(err.max()),
        'final': float(err[-1]),
    }


def read_bag(bag, odom_topic, truth_topic, world_frame, robot_frame):
    """Return (slam, truth) arrays of (t, x, y) read from a rosbag2 folder."""
    import rosbag2_py
    from nav_msgs.msg import Odometry
    from rclpy.serialization import deserialize_message
    from tf2_msgs.msg import TFMessage

    def stamp(header):
        return header.stamp.sec + header.stamp.nanosec * 1e-9

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[odom_topic, truth_topic]))
    slam, truth = [], []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == odom_topic:
            msg = deserialize_message(data, Odometry)
            p = msg.pose.pose.position
            slam.append((stamp(msg.header), p.x, p.y))
        else:
            for tf in deserialize_message(data, TFMessage).transforms:
                if tf.header.frame_id == world_frame and tf.child_frame_id == robot_frame:
                    t = tf.transform.translation
                    truth.append((stamp(tf.header), t.x, t.y))
    if not slam:
        raise SystemExit(f'no messages on {odom_topic} in {bag}')
    if not truth:
        raise SystemExit(f'no {world_frame} -> {robot_frame} transform on {truth_topic} in {bag}')
    return np.array(slam), np.array(truth)


def plot(result, origin, site, title, out):
    """Write the two-panel figure: paths top-down, and error over time."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    plt.rcParams.update({'font.size': 11, 'text.color': INK, 'axes.labelcolor': INK2,
                         'xtick.color': INK2, 'ytick.color': INK2})
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(14, 6.2),
                                  gridspec_kw={'width_ratios': [1.15, 1]}, facecolor=SURFACE)
    for a in (ax, ax2):
        a.set_facecolor(SURFACE)
        a.grid(color=GRID, linewidth=0.8)
        a.set_axisbelow(True)
        for side in ('top', 'right'):
            a.spines[side].set_visible(False)
        for side in ('left', 'bottom'):
            a.spines[side].set_color(GRID)

    if site:
        layout = SITES[site]
        for i, (p1, p2) in enumerate(layout['walls']):
            line = world_to_odom([p1, p2], origin)
            ax.plot(line[:, 0], line[:, 1], color=MUTED, lw=3, solid_capstyle='butt', zorder=1,
                    label='Walls' if i == 0 else None)
        for kind, marker in (('columns', 'o'), ('crates', 's')):
            pts = world_to_odom(layout[kind], origin)
            ax.plot(pts[:, 0], pts[:, 1], marker, ms=9, color=MUTED, zorder=1,
                    label=kind.capitalize())
        for text, xy, offset in layout['labels']:
            ax.annotate(text, tuple(world_to_odom([xy], origin)[0]), xytext=offset,
                        textcoords='offset points', ha='center', color=INK2)

    truth, slam = result['truth_xy'], result['slam_xy']
    ax.plot(truth[:, 0], truth[:, 1], color=TRUTH_COLOR, lw=2, label='Ground truth', zorder=2)
    ax.plot(slam[:, 0], slam[:, 1], color=SLAM_COLOR, lw=2, ls=(0, (5, 2)), label='SLAM',
            zorder=3)
    ax.plot(*truth[0], 'o', ms=9, color=INK, mec=SURFACE, mew=2, zorder=4)
    ax.annotate('start', tuple(truth[0]), xytext=(8, -14), textcoords='offset points',
                color=INK2)
    ax.plot(*truth[-1], 'o', ms=9, mfc=SURFACE, mec=INK, mew=2, zorder=4)
    ax.annotate('end', tuple(truth[-1]), xytext=(8, 6), textcoords='offset points', color=INK2)
    ax.set_aspect('equal')
    ax.set_xlabel('x (m, SLAM odom frame)')
    ax.set_ylabel('y (m)')
    ax.set_title('Path, top-down', loc='left', fontsize=13, color=INK)
    ax.legend(frameon=False, loc='best', fontsize=10)

    t, err_cm = result['t'], result['err'] * 100
    ax2.plot(t, err_cm, color=SLAM_COLOR, lw=2)
    ax2.set_xlim(0, t[-1])
    ax2.set_ylim(0, max(err_cm.max() * 1.15, 1))
    ax2.set_xlabel('time (s)')
    ax2.set_ylabel('cm')
    ax2.set_title('Position error vs ground truth', loc='left', fontsize=13, color=INK)
    ax2.axhline(result['rmse'] * 100, color=INK2, lw=1, ls=(0, (2, 3)))
    ax2.annotate(f"RMSE {result['rmse'] * 100:.1f} cm", (0, result['rmse'] * 100),
                 xytext=(6, 5), textcoords='offset points', color=INK2)
    ax2.annotate(f'final {err_cm[-1]:.1f} cm', (t[-1], err_cm[-1]), xytext=(-4, -10),
                 textcoords='offset points', ha='right', va='top', color=INK2)

    fig.suptitle(f"{title}: {result['distance']:.1f} m in {t[-1] / 60:.1f} min",
                 x=0.01, ha='left', fontsize=15, color=INK)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(out, dpi=130, facecolor=SURFACE)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('bag', help='rosbag2 folder')
    parser.add_argument('out', help='output .png')
    parser.add_argument('--origin', nargs=3, type=float, required=True, metavar=('X', 'Y', 'YAW'),
                        help='world pose (m, m, rad) where SLAM started')
    parser.add_argument('--odom-topic', default='/ugv1/odom')
    parser.add_argument('--truth-topic', default='/a200_0001/ground_truth/pose')
    parser.add_argument('--world-frame', default='mrs_test_site')
    parser.add_argument('--robot-frame', default='a200_0001')
    parser.add_argument('--site', choices=sorted(SITES), help='draw this world\'s walls/obstacles')
    parser.add_argument('--title', default='SLAM vs ground truth')
    args = parser.parse_args(argv)

    slam, truth = read_bag(args.bag, args.odom_topic, args.truth_topic,
                           args.world_frame, args.robot_frame)
    result = position_errors(slam, truth, args.origin)
    print(f"samples {len(slam)}  duration {result['t'][-1]:.1f} s  "
          f"path {result['distance']:.1f} m")
    print(f"position error: RMSE {result['rmse'] * 100:.1f} cm  "
          f"mean {result['mean'] * 100:.1f} cm  max {result['max'] * 100:.1f} cm  "
          f"final {result['final'] * 100:.1f} cm")
    plot(result, args.origin, args.site, args.title, args.out)
    print(f'saved {args.out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
