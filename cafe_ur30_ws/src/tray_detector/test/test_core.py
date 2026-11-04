import math

import cv2
import numpy as np

from tray_detector.core import (Tracker, find_blobs, normalize_yaw,
                                pixel_to_camera, robust_depth, segment)

RED = [0, 120, 70, 10, 255, 255, 170, 120, 70, 180, 255, 255]


def _scene(angle_deg, color_bgr, center=(320, 240), size=(120, 60)):
    img = np.full((480, 640, 3), 180, np.uint8)  # grey table
    rect = (center, size, angle_deg)
    cv2.fillPoly(img, [cv2.boxPoints(rect).astype(np.int32)], color_bgr)
    return img


def test_normalize_yaw():
    assert abs(normalize_yaw(math.pi) - 0.0) < 1e-9
    assert -math.pi / 2 <= normalize_yaw(2.9) < math.pi / 2


def test_detect_orientation_and_size():
    for angle in (0, 25, 70, 115, 160):
        img = _scene(angle, (0, 0, 255))
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        blobs = find_blobs(segment(hsv, RED), 500, 50000)
        assert len(blobs) == 1
        b = blobs[0]
        assert abs(b.center[0] - 320) < 2 and abs(b.center[1] - 240) < 2
        assert abs(b.long_px - 120) < 4 and abs(b.short_px - 60) < 4
        got = normalize_yaw(math.atan2(b.long_dir[1], b.long_dir[0]))
        want = normalize_yaw(math.radians(angle))
        assert abs(normalize_yaw(got - want)) < math.radians(2.5), (angle, got, want)


def test_colour_separation():
    img = _scene(0, (255, 80, 0))  # blue in BGR
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    assert len(find_blobs(segment(hsv, [0, 120, 70, 10, 255, 255]), 500, 50000)) == 0
    assert len(find_blobs(segment(hsv, [100, 100, 50, 135, 255, 255]), 500, 50000)) == 1


def test_depth_and_deprojection():
    depth = np.full((480, 640), 1.0, np.float32)
    depth[200:280, 260:380] = 0.96
    cnt = np.array([[[260, 200]], [[380, 200]], [[380, 280]], [[260, 280]]], np.int32)
    assert abs(robust_depth(depth, cnt) - 0.96) < 1e-4
    p = pixel_to_camera(320, 240, 0.96, 468, 468, 320, 240)
    assert np.allclose(p, [0, 0, 0.96])


def test_tracker_ids_and_stability():
    tr = Tracker(gate=0.08, window=5, stable_std=0.004, min_hits=3)
    pos = np.array([0.7, 0.1, 0.77])
    for i in range(6):
        out = tr.update([{'color': 'red', 'pos': pos + 0.0005 * i, 'yaw': 0.0}], float(i))
    _, trk = out[0]
    assert trk.id == 1 and tr.is_stable(trk)
    out = tr.update([{'color': 'red', 'pos': pos + np.array([0.2, 0, 0]), 'yaw': 0.0}], 7.0)
    assert out[0][1].id == 2 and not tr.is_stable(out[0][1])
    out = tr.update([{'color': 'blue', 'pos': pos + np.array([0.2, 0, 0]), 'yaw': 0.0}], 8.0)
    assert out[0][1].id == 3
