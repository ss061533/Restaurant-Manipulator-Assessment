"""Pure OpenCV / numpy helpers for the tray detector.

Nothing in here imports ROS, so it can be unit-tested without a ROS install.
"""
from collections import deque
from dataclasses import dataclass, field
import math

import cv2
import numpy as np


def normalize_yaw(yaw: float) -> float:
    """Map a 180°-symmetric axis angle to [-pi/2, pi/2)."""
    return (yaw + math.pi / 2.0) % math.pi - math.pi / 2.0


def segment(hsv: np.ndarray, ranges) -> np.ndarray:
    """Binary mask for a flat list of HSV ranges [h0,s0,v0,h1,s1,v1, ...]."""
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for i in range(0, len(ranges) - 5, 6):
        lo = np.array(ranges[i:i + 3], np.uint8)
        hi = np.array(ranges[i + 3:i + 6], np.uint8)
        mask |= cv2.inRange(hsv, lo, hi)
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    return mask


@dataclass
class Blob:
    contour: np.ndarray
    area: float
    center: np.ndarray        # pixel (u, v)
    long_dir: np.ndarray      # unit vector (pixels) along the long side
    long_px: float
    short_px: float
    touches_border: bool
    box: np.ndarray           # 4x2 rotated-rect corners


def find_blobs(mask: np.ndarray, min_area: float, max_area: float,
               border_margin: int = 2):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    h, w = mask.shape[:2]
    blobs = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < min_area or area > max_area:
            continue
        x, y, bw, bh = cv2.boundingRect(c)
        touches = (x <= border_margin or y <= border_margin or
                   x + bw >= w - border_margin or y + bh >= h - border_margin)
        rect = cv2.minAreaRect(c)
        box = cv2.boxPoints(rect)
        e1, e2 = box[1] - box[0], box[2] - box[1]
        n1, n2 = float(np.linalg.norm(e1)), float(np.linalg.norm(e2))
        if n1 >= n2:
            long_vec, long_n, short_n = e1, n1, n2
        else:
            long_vec, long_n, short_n = e2, n2, n1
        if long_n < 1e-6:
            continue
        blobs.append(Blob(c, area, np.array(rect[0], float), long_vec / long_n,
                          long_n, short_n, touches, box))
    return blobs


def robust_depth(depth: np.ndarray, contour: np.ndarray, erode_px: int = 5,
                 dmin: float = 0.1, dmax: float = 10.0):
    """Median depth over the (eroded) blob; None if not enough valid pixels."""
    blob = np.zeros(depth.shape[:2], np.uint8)
    cv2.drawContours(blob, [contour], -1, 255, thickness=cv2.FILLED)
    inner = cv2.erode(blob, np.ones((erode_px, erode_px), np.uint8))
    if cv2.countNonZero(inner) < 20:
        inner = blob
    vals = depth[inner > 0]
    vals = vals[np.isfinite(vals) & (vals > dmin) & (vals < dmax)]
    if vals.size < 20:
        return None
    return float(np.median(vals))


def pixel_to_camera(u, v, z, fx, fy, cx, cy) -> np.ndarray:
    return np.array([(u - cx) * z / fx, (v - cy) * z / fy, z], float)


# ----------------------------------------------------------------------------
# Tracking: associates detections over time so ids are persistent and so we can
# tell whether a tray is still moving (stability) -> avoids stale / moving poses.
# ----------------------------------------------------------------------------
@dataclass
class Track:
    id: int
    color: str
    pos: np.ndarray
    yaw: float
    first_seen: float
    last_seen: float
    hits: int = 1
    hist: deque = field(default_factory=deque)


class Tracker:
    def __init__(self, gate=0.08, window=5, stable_std=0.004, min_hits=3,
                 timeout=30.0):
        self.gate = gate
        self.window = window
        self.stable_std = stable_std
        self.min_hits = min_hits
        self.timeout = timeout
        self.tracks = {}
        self._next_id = 1

    def update(self, dets, stamp):
        """dets: list of dicts with 'color','pos'(np3),'yaw'. Returns [(det, track)]."""
        pairs = []
        for di, d in enumerate(dets):
            for tid, t in self.tracks.items():
                if t.color != d['color']:
                    continue
                dist = float(np.linalg.norm(t.pos[:2] - d['pos'][:2]))
                if dist <= self.gate:
                    pairs.append((dist, di, tid))
        pairs.sort()
        used_d, used_t, result = set(), set(), {}
        for _, di, tid in pairs:
            if di in used_d or tid in used_t:
                continue
            used_d.add(di)
            used_t.add(tid)
            t = self.tracks[tid]
            t.pos, t.yaw, t.last_seen = dets[di]['pos'].copy(), dets[di]['yaw'], stamp
            t.hits += 1
            t.hist.append(t.pos.copy())
            while len(t.hist) > self.window:
                t.hist.popleft()
            result[di] = t
        for di, d in enumerate(dets):
            if di in used_d:
                continue
            t = Track(self._next_id, d['color'], d['pos'].copy(), d['yaw'], stamp, stamp,
                      hist=deque([d['pos'].copy()]))
            self._next_id += 1
            self.tracks[t.id] = t
            result[di] = t
        for tid in [k for k, t in self.tracks.items() if stamp - t.last_seen > self.timeout]:
            del self.tracks[tid]
        return [(dets[i], result[i]) for i in sorted(result)]

    def is_stable(self, t: Track) -> bool:
        if t.hits < self.min_hits or len(t.hist) < self.window:
            return False
        xy = np.array([p[:2] for p in t.hist])
        return float(np.max(np.std(xy, axis=0))) < self.stable_std

    def confidence(self, t: Track) -> float:
        return min(1.0, t.hits / float(max(1, 2 * self.min_hits)))
