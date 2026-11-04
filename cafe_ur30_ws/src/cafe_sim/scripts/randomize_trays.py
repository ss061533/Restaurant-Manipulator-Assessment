#!/usr/bin/env python3
"""Move trays in the running Gazebo world (no ROS needed, uses the `ign` CLI).

  randomize_trays.py                    # random poses for all trays (non-overlapping)
  randomize_trays.py --seed 3           # reproducible layout
  randomize_trays.py --move red         # move ONE tray mid-run  (tests dynamic update, 5.5)
  randomize_trays.py --remove blue      # take a tray away         (tests absence handling, 5.3)
  randomize_trays.py --restore blue     # put a removed tray back at a random place
"""
import argparse
import math
import os
import random
import shutil
import subprocess

WORLD = 'cafe'
TRAYS = ['red', 'blue', 'green']
X_RANGE, Y_RANGE = (0.60, 0.95), (-0.30, 0.30)   # inside the camera view and the arm's reach
MIN_DIST = 0.26
TRAY_Z = 0.772
# new Gazebo (`gz`) or Ignition Fortress (`ign`)? CAFE_GZ=true forces gz; otherwise use what is installed.
USE_GZ = (os.environ.get('CAFE_GZ', '').lower() == 'true' or
          (shutil.which('gz') is not None and shutil.which('ign') is None))
CLI, MSG = ('gz', 'gz.msgs') if USE_GZ else ('ign', 'ignition.msgs')


def set_pose(name, x, y, z, yaw):
    req = (f'name: "tray_{name}", position: {{x: {x:.4f}, y: {y:.4f}, z: {z:.4f}}}, '
           f'orientation: {{x: 0, y: 0, z: {math.sin(yaw / 2):.5f}, w: {math.cos(yaw / 2):.5f}}}')
    cmd = [CLI, 'service', '-s', f'/world/{WORLD}/set_pose', '--reqtype', f'{MSG}.Pose',
           '--reptype', f'{MSG}.Boolean', '--timeout', '3000', '--req', req]
    out = subprocess.run(cmd, capture_output=True, text=True)
    ok = 'true' in out.stdout.lower()
    print(f'  tray_{name:5s} -> ({x:+.2f}, {y:+.2f}) yaw={math.degrees(yaw):+4.0f}°  '
          f'{"ok" if ok else "FAILED: " + (out.stderr or out.stdout).strip()}')
    return ok


def sample(existing):
    for _ in range(500):
        x, y = random.uniform(*X_RANGE), random.uniform(*Y_RANGE)
        if all(math.hypot(x - ex, y - ey) > MIN_DIST for ex, ey in existing):
            return x, y, random.uniform(-math.pi / 2, math.pi / 2)
    raise RuntimeError('could not find a free spot')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('--seed', type=int)
    ap.add_argument('--move', choices=TRAYS)
    ap.add_argument('--remove', choices=TRAYS)
    ap.add_argument('--restore', choices=TRAYS)
    a = ap.parse_args()
    if a.seed is not None:
        random.seed(a.seed)
    if a.remove:
        set_pose(a.remove, 2.5, 1.5, TRAY_Z, 0.0)   # off the table (it falls to the floor)
        return
    if a.move or a.restore:
        n = a.move or a.restore
        x, y, yaw = sample([])
        set_pose(n, x, y, TRAY_Z, yaw)
        return
    placed = []
    for n in TRAYS:
        x, y, yaw = sample(placed)
        placed.append((x, y))
        set_pose(n, x, y, TRAY_Z, yaw)


if __name__ == '__main__':
    main()
