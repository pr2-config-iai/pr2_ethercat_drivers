#!/usr/bin/env python3
"""Velocity steps on one joint of a pid_controller velocity controller, with a watchdog (D27).

Holds every joint of the controller at 0 rad/s and moves one of them: hold, +velocity, hold,
-velocity, hold (so it ends near where it started), then halts the motors. Calls
/pr2_ethercat/halt_motors and stops at once if

  - the moved joint is faster than --max-velocity, or leaves start +- --window or its URDF soft
    limits (minus --margin),
  - another joint of the controller moves more than --others-window or faster than
    --others-velocity,
  - any other joint of the robot moves faster than --rest-velocity,
  - /joint_states stops, or the controller is not listening.

Needs the controller active and the motors released. Writes every sample to --csv.

  velocity_step_test.py --controller r_arm_velocity_controller --joint r_forearm_roll_joint \\
      --velocity 0.3 --step 2
  velocity_step_test.py --controller r_arm_velocity_controller --joint none --hold 5   # hold only
"""
import argparse
import math
import sys
import threading
import time
import xml.etree.ElementTree as ET

import rclpy
from control_msgs.msg import MultiDOFCommand
from rcl_interfaces.srv import GetParameters
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Empty

STALE = 0.2

ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
ap.add_argument('--controller', required=True)
ap.add_argument('--joint', required=True, help="joint to move, or 'none' to only hold")
ap.add_argument('--velocity', type=float, default=0.2, help='rad/s (default 0.2)')
ap.add_argument('--step', type=float, default=1.5, help='seconds per direction (default 1.5)')
ap.add_argument('--pause', type=float, default=2.0, help='seconds of hold between steps (default 2)')
ap.add_argument('--hold', type=float, default=3.0, help='seconds of hold at the start (default 3)')
ap.add_argument('--max-velocity', type=float, default=None, help='default 2 x --velocity + 0.2')
ap.add_argument('--window', type=float, default=0.5, help='rad around the start (default 0.5)')
ap.add_argument('--margin', type=float, default=0.1, help='rad inside the soft limits (default 0.1)')
ap.add_argument('--others-window', type=float, default=0.15, help='rad (default 0.15)')
ap.add_argument('--others-velocity', type=float, default=0.3, help='rad/s (default 0.3)')
ap.add_argument('--rest-velocity', type=float, default=0.5, help='rad/s (default 0.5)')
ap.add_argument('--csv', default='velocity_step_test.csv')
args = ap.parse_args()
target = None if args.joint == 'none' else args.joint
max_velocity = args.max_velocity if args.max_velocity is not None else 2 * abs(args.velocity) + 0.2

rclpy.init()
node = rclpy.create_node('velocity_step_test')
latest = {}
urdf = {}


def on_js(msg):
    latest['msg'] = msg
    latest['t'] = time.monotonic()


node.create_subscription(JointState, '/joint_states', on_js, 10)
node.create_subscription(String, '/robot_description', lambda m: urdf.update(xml=m.data),
                         QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
pub = node.create_publisher(MultiDOFCommand, '/%s/reference' % args.controller, 10)
halt = node.create_client(Empty, '/pr2_ethercat/halt_motors')
params = node.create_client(GetParameters, '/%s/get_parameters' % args.controller)
ex = SingleThreadedExecutor()
ex.add_node(node)
spin_thread = threading.Thread(target=ex.spin)
spin_thread.start()

dofs = []


def send(velocity):
    m = MultiDOFCommand()
    m.dof_names = list(dofs)
    m.values = [velocity if d == target else 0.0 for d in dofs]
    pub.publish(m)


def do_halt(reason):
    print('HALT: ' + reason)
    if dofs:
        for _ in range(5):
            send(0.0)
    future = halt.call_async(Empty.Request())
    end = time.monotonic() + 2.0
    while not future.done() and time.monotonic() < end:
        time.sleep(0.01)
    print('halt_motors ' + ('answered' if future.done() else 'DID NOT ANSWER'))


def wait_for(condition, seconds):
    end = time.monotonic() + seconds
    while not condition() and time.monotonic() < end:
        time.sleep(0.02)
    return condition()


def setup():
    if not halt.wait_for_service(timeout_sec=5.0):
        return 'halt_motors not available'
    if not params.wait_for_service(timeout_sec=5.0):
        return 'controller %s not loaded' % args.controller
    req = GetParameters.Request()
    req.names = ['dof_names']
    future = params.call_async(req)
    if not wait_for(future.done, 5.0) or not future.result().values:
        return 'could not read dof_names of %s' % args.controller
    dofs.extend(future.result().values[0].string_array_value)
    if target and target not in dofs:
        return '%s is not a joint of %s (%s)' % (target, args.controller, ', '.join(dofs))
    if not wait_for(lambda: pub.get_subscription_count() > 0, 3.0):
        return 'no controller listening on /%s/reference' % args.controller
    if not wait_for(lambda: 'msg' in latest, 3.0):
        return 'no /joint_states'
    if not wait_for(lambda: 'xml' in urdf, 3.0):
        return 'no /robot_description'
    return None


def soft_limits(name):
    for j in ET.fromstring(urdf['xml']).findall('joint'):
        if j.get('name') == name:
            s = j.find('safety_controller')
            if j.get('type') == 'continuous' or s is None or s.get('soft_lower_limit') is None:
                return -math.inf, math.inf
            return float(s.get('soft_lower_limit')), float(s.get('soft_upper_limit'))
    return -math.inf, math.inf


reason = None
stats = []  # per phase: (ref, rows)
try:
    reason = setup()
    if not reason:
        msg = latest['msg']
        idx = {n: i for i, n in enumerate(msg.name)}
        start = {d: msg.position[idx[d]] for d in dofs}
        lo = hi = None
        if target:
            soft_lo, soft_hi = soft_limits(target)
            lo = max(start[target] - args.window, soft_lo + args.margin)
            hi = min(start[target] + args.window, soft_hi - args.margin)
            if not lo < start[target] < hi:
                reason = '%s at %.3f is not inside its allowed range [%.3f, %.3f]' % (target, start[target], lo, hi)
            else:
                print('%s starts at %+.3f, allowed [%+.3f, %+.3f]' % (target, start[target], lo, hi))
        schedule = [(0.0, args.hold)]
        if target:
            schedule += [(args.velocity, args.step), (0.0, args.pause), (-args.velocity, args.step), (0.0, args.pause)]
        t0 = time.monotonic()
        with open(args.csv, 'w') as log:
            log.write('t,ref,' + ','.join('%s_pos,%s_vel,%s_eff' % (d, d, d) for d in dofs) + '\n')
            for ref, duration in schedule:
                if reason:
                    break
                print('%s %+.2f rad/s, others 0, for %.1f s' % (target or 'all', ref, duration))
                rows = []
                stats.append((ref, rows))
                end = time.monotonic() + duration
                while time.monotonic() < end and not reason:
                    send(ref)
                    msg = latest['msg']
                    if time.monotonic() - latest['t'] > STALE:
                        reason = '/joint_states stale'
                        break
                    row = {d: (msg.position[idx[d]], msg.velocity[idx[d]], msg.effort[idx[d]]) for d in dofs}
                    rows.append(row)
                    log.write('%.3f,%.3f,' % (time.monotonic() - t0, ref) +
                              ','.join('%.5f,%.5f,%.5f' % row[d] for d in dofs) + '\n')
                    for d in dofs:
                        pos, vel, _ = row[d]
                        if d == target:
                            if abs(vel) > max_velocity:
                                reason = '%s at %.2f rad/s' % (d, vel)
                            elif not lo <= pos <= hi:
                                reason = '%s at %.3f left [%.3f, %.3f]' % (d, pos, lo, hi)
                        elif abs(pos - start[d]) > args.others_window:
                            reason = '%s moved %+.3f rad' % (d, pos - start[d])
                        elif abs(vel) > args.others_velocity:
                            reason = '%s at %.2f rad/s' % (d, vel)
                        if reason:
                            break
                    if not reason:
                        for n, v in zip(msg.name, msg.velocity):
                            if n not in start and abs(v) > args.rest_velocity:
                                reason = '%s (not controlled) at %.2f rad/s' % (n, v)
                                break
                    time.sleep(0.01)
finally:
    do_halt(reason or 'sequence finished')
    ex.shutdown()
    spin_thread.join()
    node.destroy_node()
    rclpy.shutdown()

for ref, rows in stats:
    if not rows:
        continue
    line = 'ref %+.2f, %d samples:' % (ref, len(rows))
    if target:
        tail = rows[len(rows) // 2:]
        line += ' %s moved %+.3f rad, velocity 2nd half %+.3f rad/s, effort %+.2f Nm |' % (
            target, rows[-1][target][0] - rows[0][target][0], sum(r[target][1] for r in tail) / len(tail),
            sum(r[target][2] for r in rows) / len(rows))
    others = [d for d in dofs if d != target]
    worst = max(others, key=lambda d: max(abs(r[d][0] - rows[0][d][0]) for r in rows))
    line += ' others: largest drift %s %+.4f rad; largest |effort| %s' % (
        worst, max((r[worst][0] - rows[0][worst][0] for r in rows), key=abs),
        ', '.join('%s %.2f' % (d[2:-6] if d.startswith(('r_', 'l_')) and d.endswith('_joint') else d, max(abs(r[d][2]) for r in rows)) for d in others))
    print(line)
sys.exit(1 if reason else 0)
