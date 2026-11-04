#!/usr/bin/env python3
"""RGB-D tray detector.

Pipeline per frame
  1. HSV colour segmentation per tray colour  (core.segment)
  2. contour -> rotated rectangle              (core.find_blobs)
  3. depth of the tray top (median, eroded)    (core.robust_depth)
  4. pixel + depth -> camera frame -> `target_frame` via TF2   <- camera->robot transform
  5. sanity checks (ROI, height above table, plausible size, not cut by image border)
  6. temporal tracking -> persistent ids + `stable` flag (tray has stopped moving)

Only detections seen in the *current* frame are published, so consumers can never
act on stale coordinates; the header stamp is the image stamp.
"""
import math

import cv2
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from cafe_interfaces.msg import TrayDetection, TrayDetectionArray
from tray_detector.core import (Tracker, find_blobs, normalize_yaw, pixel_to_camera,
                                robust_depth, segment)

DRAW_BGR = {'red': (0, 0, 255), 'green': (0, 200, 0), 'blue': (255, 90, 0)}
MARKER_RGB = {'red': (1.0, 0.1, 0.1), 'green': (0.1, 0.8, 0.1), 'blue': (0.2, 0.3, 1.0)}

DEFAULT_HSV = {
    'red': [0, 100, 60, 10, 255, 255, 170, 100, 60, 180, 255, 255],
    'green': [35, 60, 40, 85, 255, 255],
    'blue': [100, 100, 40, 135, 255, 255],
}


def quat_to_matrix(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class TrayDetector(Node):
    def __init__(self):
        super().__init__('tray_detector')
        p = self.declare_parameter
        p('camera_frame', 'camera_optical_frame')
        p('target_frame', 'world')
        p('rgb_topic', '/camera/image')
        p('depth_topic', '/camera/depth_image')
        p('info_topic', '/camera/camera_info')
        p('colors', ['red', 'blue', 'green'])
        for c, r in DEFAULT_HSV.items():
            p(f'hsv_{c}', r)
        p('min_area_px', 600.0)
        p('max_area_px', 60000.0)
        p('table_z', 0.75)
        p('min_top_height', 0.015)
        p('max_top_height', 0.09)
        p('size_length_range', [0.12, 0.26])
        p('size_width_range', [0.06, 0.16])
        p('roi_x', [0.40, 1.10])
        p('roi_y', [-0.50, 0.50])
        p('track_gate', 0.08)
        p('track_window', 5)
        p('track_min_hits', 3)
        p('stable_std', 0.004)
        p('track_timeout', 30.0)
        p('publish_debug', True)

        g = self.get_parameter
        self.camera_frame = g('camera_frame').value
        self.target_frame = g('target_frame').value
        self.colors = list(g('colors').value)
        self.hsv = {c: list(g(f'hsv_{c}').value) for c in self.colors
                    if self.has_parameter(f'hsv_{c}')}
        self.min_area = g('min_area_px').value
        self.max_area = g('max_area_px').value
        self.table_z = g('table_z').value
        self.min_top_h = g('min_top_height').value
        self.max_top_h = g('max_top_height').value
        self.len_rng = list(g('size_length_range').value)
        self.wid_rng = list(g('size_width_range').value)
        self.roi_x = list(g('roi_x').value)
        self.roi_y = list(g('roi_y').value)
        self.publish_debug = g('publish_debug').value
        self.tracker = Tracker(g('track_gate').value, int(g('track_window').value),
                               g('stable_std').value, int(g('track_min_hits').value),
                               g('track_timeout').value)

        self.bridge = CvBridge()
        self.K = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(CameraInfo, g('info_topic').value, self.on_info, 10)
        rgb = message_filters.Subscriber(self, Image, g('rgb_topic').value,
                                         qos_profile=qos_profile_sensor_data)
        depth = message_filters.Subscriber(self, Image, g('depth_topic').value,
                                           qos_profile=qos_profile_sensor_data)
        self.sync = message_filters.ApproximateTimeSynchronizer([rgb, depth], 5, 0.05)
        self.sync.registerCallback(self.on_images)

        self.pub = self.create_publisher(TrayDetectionArray, '~/detections', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '~/markers', 10)
        self.pub_dbg = self.create_publisher(Image, '~/debug_image', 1)
        self.get_logger().info(
            f'Tray detector up: camera_frame={self.camera_frame} -> {self.target_frame}')

    # ------------------------------------------------------------------ helpers
    def on_info(self, msg: CameraInfo):
        if self.K is None:
            self.K = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])  # fx fy cx cy
            self.get_logger().info(f'Camera intrinsics received: {self.K}')

    def lookup_T(self):
        """4x4 camera->target transform (static in this setup, looked up anyway)."""
        try:
            tf = self.tf_buffer.lookup_transform(self.target_frame, self.camera_frame, Time())
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'Waiting for TF {self.target_frame}<-{self.camera_frame}: {e}',
                                   throttle_duration_sec=5.0)
            return None
        q, t = tf.transform.rotation, tf.transform.translation
        T = np.eye(4)
        T[:3, :3] = quat_to_matrix(q.x, q.y, q.z, q.w)
        T[:3, 3] = [t.x, t.y, t.z]
        return T

    # --------------------------------------------------------------- main loop
    def on_images(self, rgb_msg: Image, depth_msg: Image):
        if self.K is None:
            return
        T = self.lookup_T()
        if T is None:
            return
        fx, fy, cx, cy = self.K
        bgr = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
        depth = np.asarray(self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough'), np.float32)
        if depth.shape[:2] != bgr.shape[:2]:
            self.get_logger().warn('RGB / depth size mismatch', throttle_duration_sec=5.0)
            return
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        stamp = Time.from_msg(rgb_msg.header.stamp).nanoseconds * 1e-9

        dets = []
        for color in self.colors:
            if color not in self.hsv:
                continue
            mask = segment(hsv, self.hsv[color])
            for b in find_blobs(mask, self.min_area, self.max_area):
                if b.touches_border:
                    continue  # partially visible tray -> pose would be biased
                z = robust_depth(depth, b.contour)
                if z is None:
                    continue
                # centre and a point along the long axis, both at tray-top depth
                p_c = pixel_to_camera(b.center[0], b.center[1], z, fx, fy, cx, cy)
                e_px = b.center + 0.5 * b.long_px * b.long_dir
                p_e = pixel_to_camera(e_px[0], e_px[1], z, fx, fy, cx, cy)
                P = T @ np.append(p_c, 1.0)
                E = T @ np.append(p_e, 1.0)
                top_h = float(P[2] - self.table_z)
                if not (self.min_top_h <= top_h <= self.max_top_h):
                    continue  # implausible depth -> not a tray lying on the table
                if not (self.roi_x[0] <= P[0] <= self.roi_x[1] and
                        self.roi_y[0] <= P[1] <= self.roi_y[1]):
                    continue
                yaw = normalize_yaw(math.atan2(E[1] - P[1], E[0] - P[0]))
                length = b.long_px * z / fx
                width = b.short_px * z / fx
                if not (self.len_rng[0] <= length <= self.len_rng[1] and
                        self.wid_rng[0] <= width <= self.wid_rng[1]):
                    continue
                dets.append({'color': color, 'pos': np.array([P[0], P[1], self.table_z + top_h / 2]),
                             'yaw': yaw, 'length': length, 'width': width,
                             'height': top_h, 'blob': b})

        matched = self.tracker.update(dets, stamp)

        arr = TrayDetectionArray()
        arr.header.stamp = rgb_msg.header.stamp
        arr.header.frame_id = self.target_frame
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for det, trk in matched:
            msg = TrayDetection()
            msg.header = arr.header
            msg.id = trk.id
            msg.color = det['color']
            msg.pose = self.make_pose(det['pos'], det['yaw'])
            msg.yaw = float(det['yaw'])
            msg.length, msg.width, msg.height = (float(det['length']), float(det['width']),
                                                 float(det['height']))
            msg.stable = bool(self.tracker.is_stable(trk))
            msg.confidence = float(self.tracker.confidence(trk))
            arr.trays.append(msg)
            markers.markers += self.make_markers(msg, arr.header)
        self.pub.publish(arr)
        self.pub_markers.publish(markers)

        if self.publish_debug:
            for det, trk in matched:
                b = det['blob']
                col = DRAW_BGR.get(det['color'], (255, 255, 255))
                cv2.polylines(bgr, [b.box.astype(np.int32)], True, col, 2)
                tip = (b.center + 0.5 * b.long_px * b.long_dir).astype(int)
                cv2.arrowedLine(bgr, tuple(b.center.astype(int)), tuple(tip), (255, 255, 255), 2)
                tag = f"#{trk.id} {det['color']} {'STABLE' if self.tracker.is_stable(trk) else 'moving'}"
                cv2.putText(bgr, tag, (int(b.center[0]) - 40, int(b.center[1]) - 35),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            out = self.bridge.cv2_to_imgmsg(bgr, 'bgr8')
            out.header = rgb_msg.header
            self.pub_dbg.publish(out)

    @staticmethod
    def make_pose(pos, yaw):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = map(float, pos)
        pose.orientation.z = math.sin(yaw / 2.0)
        pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    @staticmethod
    def make_markers(msg: TrayDetection, header):
        r, g, b = MARKER_RGB.get(msg.color, (1.0, 1.0, 1.0))
        cube = Marker()
        cube.header = header
        cube.ns, cube.id, cube.type, cube.action = 'tray', msg.id, Marker.CUBE, Marker.ADD
        cube.pose = msg.pose
        cube.scale.x, cube.scale.y, cube.scale.z = msg.length, msg.width, max(msg.height, 0.01)
        cube.color.r, cube.color.g, cube.color.b = r, g, b
        cube.color.a = 0.9 if msg.stable else 0.45
        text = Marker()
        text.header = header
        text.ns, text.id, text.type, text.action = 'label', msg.id, Marker.TEXT_VIEW_FACING, Marker.ADD
        text.pose = msg.pose
        text.pose.position.z += 0.08
        text.scale.z = 0.05
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0
        text.text = f'#{msg.id} {msg.color}'
        return [cube, text]


def main():
    rclpy.init()
    node = TrayDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
