#!/usr/bin/env python3
"""Task manager: the brain that ties detection and manipulation together.

            +------+  executor ready   +------+  candidates   +-----------+
   start -> | INIT | ----------------> | IDLE | ------------> | EXECUTING |
            +------+                   +------+ <------------ +-----------+
                                          ^   result (ok / fail+retry / fail+skip)
                                          |
        no tray / stale / unstable: stay idle and re-scan every `rescan_interval`

Rules implemented
  5.3  no tray                 -> stay idle, re-scan after an interval
  5.4  pick/grasp failure      -> retry up to `max_retries`, then blacklist (skip) that tray
  5.5  tray moved              -> only fresh + stable detections are used; a moved tray gets a
                                  new track id / clears its blacklist entry
  5.6  priority                -> red > blue > green (configurable), nearest first within a class
"""
import json
import math
from enum import Enum

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger

from cafe_interfaces.action import PickPlace
from cafe_interfaces.msg import TrayDetectionArray

ERRORS = {0: 'NONE', 1: 'INVALID_INPUT', 2: 'PLANNING_FAILED', 3: 'EXECUTION_FAILED',
          4: 'GRASP_FAILED', 5: 'TRAY_DROPPED', 6: 'GRIPPER_ERROR', 7: 'CANCELED'}


class State(Enum):
    INIT = 'INIT'
    IDLE = 'IDLE'
    EXECUTING = 'EXECUTING'


class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager', automatically_declare_parameters_from_overrides=True)
        self.frame = self._p('frame', 'world')
        self.max_retries = int(self._p('max_retries', 2))
        self.rescan_interval = float(self._p('rescan_interval', 5.0))
        self.retry_delay = float(self._p('retry_delay', 2.0))
        self.post_pick_delay = float(self._p('post_pick_delay', 1.5))
        self.max_age = float(self._p('max_detection_age', 1.0))
        self.require_stable = bool(self._p('require_stable', True))
        self.priority_enabled = bool(self._p('priority_enabled', True))
        order = list(self._p('priority_order', ['red', 'blue', 'green']))
        self.prio = {c: i for i, c in enumerate(order)}
        self.move_tol = float(self._p('move_tolerance', 0.03))
        self.blacklist_timeout = float(self._p('blacklist_timeout', 0.0))  # 0 = until moved
        self.max_slots = int(self._p('max_slots', 3))
        self.slot_offset = list(self._p('slot_offset', [0.0, 0.14]))
        self.paused = not bool(self._p('autostart', True))
        self.zones = {k: list(v.value) for k, v in
                      self.get_parameters_by_prefix('drop_zones').items()}
        if not self.zones:
            self.get_logger().error('No drop_zones.* parameters found!')

        self.state = State.INIT
        self.latest = None
        self.target = None
        self.attempts = {}      # track id -> failed attempts
        self.blacklist = {}     # track id -> {'t','x','y'}
        self.slots = {c: 0 for c in self.zones}
        self.next_scan = 0.0
        self.init_future = None
        self.stats = {'attempts': 0, 'success': 0, 'failures': 0, 'skipped': 0}

        self.create_subscription(TrayDetectionArray, '/tray_detector/detections',
                                 self.on_detections, 10)
        self.client = ActionClient(self, PickPlace, '/pick_place_executor/pick_place')
        self.observe_cli = self.create_client(Trigger, '/pick_place_executor/go_observe')
        self.pub_state = self.create_publisher(String, '~/state', 10)
        self.pub_stats = self.create_publisher(String, '~/stats', 10)
        self.create_service(Trigger, '~/reset', self.on_reset)
        self.create_service(Trigger, '~/start', self.on_start)
        self.create_service(Trigger, '~/pause', self.on_pause)
        self.create_timer(0.2, self.tick)
        self.create_timer(1.0, self.publish_status)
        self.get_logger().info(f'Task manager ready. drop zones: {self.zones}, '
                               f'priority: {order if self.priority_enabled else "disabled"}')

    # ------------------------------------------------------------------ utils
    def _p(self, name, default):
        if not self.has_parameter(name):
            self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def set_state(self, s):
        if s != self.state:
            self.get_logger().info(f'State: {self.state.value} -> {s.value}')
            self.state = s

    def on_detections(self, msg):
        self.latest = msg

    # --------------------------------------------------------------- services
    def on_reset(self, _, res):
        self.attempts.clear()
        self.blacklist.clear()
        self.slots = {c: 0 for c in self.zones}
        res.success, res.message = True, 'blacklist, attempts and drop slots cleared'
        return res

    def on_start(self, _, res):
        self.paused = False
        res.success, res.message = True, 'running'
        return res

    def on_pause(self, _, res):
        self.paused = True
        res.success, res.message = True, 'paused (current pick, if any, completes)'
        return res

    # ------------------------------------------------------------ state machine
    def tick(self):
        if self.state == State.INIT:
            self.tick_init()
        elif self.state == State.IDLE:
            self.tick_idle()

    def tick_init(self):
        if self.init_future is None:
            if self.client.server_is_ready() and self.observe_cli.service_is_ready():
                self.get_logger().info('Executor online -> moving arm to observe pose')
                self.init_future = self.observe_cli.call_async(Trigger.Request())
            else:
                self.get_logger().info('Waiting for pick_place_executor ...',
                                       throttle_duration_sec=5.0)
            return
        if self.init_future.done():
            r = self.init_future.result()
            if r is not None and r.success:
                self.set_state(State.IDLE)
            else:
                self.get_logger().warn(f'go_observe failed ({r.message if r else "no reply"}); retrying')
                self.init_future = None

    def tick_idle(self):
        now = self.now_s()
        if self.paused or now < self.next_scan:
            return
        self.refresh_blacklist(now)
        cands = self.candidates(now)
        if not cands:
            self.next_scan = now + self.rescan_interval
            self.get_logger().info(
                f'No pickable tray (none visible / stale / still moving / skipped). '
                f'Idle, re-scan in {self.rescan_interval:.1f}s', throttle_duration_sec=2.0)
            return
        tray = self.select(cands)
        self.send_goal(tray, now)

    # ------------------------------------------------------------- selection
    def refresh_blacklist(self, now):
        trays = {t.id: t for t in self.latest.trays} if self.latest else {}
        for tid in list(self.blacklist):
            e = self.blacklist[tid]
            expired = self.blacklist_timeout > 0 and now - e['t'] > self.blacklist_timeout
            t = trays.get(tid)
            moved = t is not None and math.hypot(t.pose.position.x - e['x'],
                                                 t.pose.position.y - e['y']) > self.move_tol
            if expired or moved:
                self.get_logger().info(f'Tray #{tid} un-skipped ({"timeout" if expired else "it was moved"})')
                del self.blacklist[tid]
                self.attempts.pop(tid, None)

    def candidates(self, now):
        msg = self.latest
        if msg is None:
            self.get_logger().info('No detections received yet', throttle_duration_sec=5.0)
            return []
        age = now - Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        if age > self.max_age:
            self.get_logger().info(f'Detections are stale ({age:.1f}s old) - not acting on them',
                                   throttle_duration_sec=5.0)
            return []
        out = []
        for t in msg.trays:
            if t.id in self.blacklist:
                continue
            if self.require_stable and not t.stable:
                continue
            if t.color not in self.zones:
                self.get_logger().warn(f'No drop zone for colour "{t.color}"', throttle_duration_sec=10.0)
                continue
            if t.header.frame_id and t.header.frame_id != self.frame:
                continue
            out.append(t)
        return out

    def select(self, cands):
        def key(t):
            pr = self.prio.get(t.color, 99) if self.priority_enabled else 0
            return (pr, math.hypot(t.pose.position.x, t.pose.position.y))
        return min(cands, key=key)

    # --------------------------------------------------------------- execution
    def place_pose_for(self, color):
        x, y, yaw = self.zones[color]
        k = self.slots[color] % max(1, self.max_slots)
        ps = PoseStamped()
        ps.header.frame_id = self.frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = x + k * self.slot_offset[0]
        ps.pose.position.y = y + k * self.slot_offset[1]
        ps.pose.orientation.z = math.sin(yaw / 2.0)
        ps.pose.orientation.w = math.cos(yaw / 2.0)
        return ps

    def send_goal(self, tray, now):
        goal = PickPlace.Goal()
        goal.tray = tray
        goal.place_pose = self.place_pose_for(tray.color)
        self.target = tray
        self.stats['attempts'] += 1
        n = self.attempts.get(tray.id, 0)
        self.get_logger().info(
            f'Picking tray #{tray.id} ({tray.color}) at ({tray.pose.position.x:.3f}, '
            f'{tray.pose.position.y:.3f}) yaw={math.degrees(tray.yaw):.0f}deg '
            f'-> drop ({goal.place_pose.pose.position.x:.2f}, {goal.place_pose.pose.position.y:.2f})'
            f'  [attempt {n + 1}/{self.max_retries + 1}]')
        self.set_state(State.EXECUTING)
        fut = self.client.send_goal_async(goal, feedback_callback=self.on_feedback)
        fut.add_done_callback(self.on_goal_response)

    def on_feedback(self, fb):
        self.get_logger().info(f'  stage: {fb.feedback.stage}', throttle_duration_sec=0.5)

    def on_goal_response(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self.get_logger().warn('Goal rejected by executor (busy?) - retrying later')
            self.stats['attempts'] -= 1
            self.next_scan = self.now_s() + self.retry_delay
            self.set_state(State.IDLE)
            return
        gh.get_result_async().add_done_callback(self.on_result)

    def on_result(self, fut):
        res = fut.result().result
        tray, now = self.target, self.now_s()
        if res.success:
            self.stats['success'] += 1
            self.attempts.pop(tray.id, None)
            self.slots[tray.color] += 1
            self.get_logger().info(f'SUCCESS: tray #{tray.id} ({tray.color}) placed. {res.message}')
            self.next_scan = now + self.post_pick_delay
        else:
            self.stats['failures'] += 1
            n = self.attempts.get(tray.id, 0) + 1
            self.attempts[tray.id] = n
            err = ERRORS.get(res.error_code, str(res.error_code))
            if n > self.max_retries:
                self.blacklist[tray.id] = {'t': now, 'x': tray.pose.position.x,
                                           'y': tray.pose.position.y}
                self.stats['skipped'] += 1
                self.get_logger().error(
                    f'FAILED [{err}] tray #{tray.id}: {res.message}  -> SKIPPING after {n} attempts')
            else:
                self.get_logger().warn(
                    f'FAILED [{err}] tray #{tray.id}: {res.message}  -> will retry '
                    f'({n}/{self.max_retries + 1} used)')
            self.next_scan = now + self.retry_delay
        self.target = None
        self.set_state(State.IDLE)

    # ---------------------------------------------------------------- status
    def publish_status(self):
        s = String()
        s.data = self.state.value + (' (paused)' if self.paused else '')
        self.pub_state.publish(s)
        st = String()
        st.data = json.dumps({**self.stats, 'blacklisted': sorted(self.blacklist),
                              'retries_in_progress': self.attempts, 'slots': self.slots})
        self.pub_stats.publish(st)


def main():
    rclpy.init()
    node = TaskManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
