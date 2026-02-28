#!/usr/bin/env python3
import time
import math

import rclpy
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


PTS = [
    (0.424, 0.249, 0.00652),
    (0.713, 0.419, 0.00686),
    (0.94, 0.601, 0.00715),
    (1.09, 0.894, 0.00744),
    (1.04, 1.21, 0.00757),
    (0.913, 1.39, 0.00757),
    (0.775, 1.6, 0.00757),
    (0.297, 2.22, 0.00753),
    (-0.119, 2.9, 0.00757),
    (-0.381, 3.64, 0.00777),
    (-0.208, 4.22, 0.00824),
    (-0.17, 4.76, 0.00261),
    (0.171, 5.29, 0.0032),
    (0.141, 5.73, 0.00342),
    (0.115, 5.85, 0.00346),
    (0.0269, 5.98, 0.00347),
    (-0.21, 6.09, 0.00333),
    (-0.401, 6.09, 0.00318),
    (-0.616, 5.89, 0.00289),
    (-0.966, 5.65, 0.00842),
    (-1.3, 5.5, 0.00806),
    (-1.69, 5.29, 0.00762),
    (-1.8, 5.21, 0.00748),
    (-2.13, 4.94, 0.00706),
    (-2.61, 4.61, 0.00648),
    (-2.9, 4.41, 0.00612),
    (-3.11, 4.16, 0.00581),
    (-3.19, 3.84, 0.00557),
]


STEP_M = 0.12
SMOOTH_PASSES = 3
SEND_EVERY_N = 3
PAUSE_S = 0.05


def dist(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])


def lerp(a, b, t):
    return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)


def resample_polyline(points_xy, step=0.12):
    pts = points_xy[:]
    out = [pts[0]]
    acc = 0.0
    i = 0

    while i < len(pts) - 1:
        p = pts[i]
        q = pts[i + 1]
        seg = dist(p, q)

        if seg < 1e-9:
            i += 1
            continue

        need = step - acc
        if need <= seg:
            t = need / seg
            newp = lerp(p, q, t)
            out.append(newp)
            pts[i] = newp
            acc = 0.0
        else:
            acc += seg
            i += 1

    return out


def smooth_chaikin(points_xy, passes=3):
    pts = points_xy[:]
    for _ in range(passes):
        new_pts = [pts[0]]
        for i in range(len(pts) - 1):
            p = pts[i]
            q = pts[i + 1]
            Q = (0.75 * p[0] + 0.25 * q[0], 0.75 * p[1] + 0.25 * q[1])
            R = (0.25 * p[0] + 0.75 * q[0], 0.25 * p[1] + 0.75 * q[1])
            new_pts.extend([Q, R])
        new_pts.append(pts[-1])
        pts = new_pts
    return pts


def yaw_to_quat_zw(yaw):
    return math.sin(0.5 * yaw), math.cos(0.5 * yaw)


def build_smoothed_waypoints(pts_xyz):
    xy = [(x, y) for (x, y, _) in pts_xyz]

    dense = resample_polyline(xy, step=STEP_M)
    sm = smooth_chaikin(dense, passes=SMOOTH_PASSES)
    sm = resample_polyline(sm, step=STEP_M)

    out = []
    for i in range(len(sm)):
        if i == 0:
            p0, p1 = sm[i], sm[i + 1]
        elif i == len(sm) - 1:
            p0, p1 = sm[i - 1], sm[i]
        else:
            p0, p1 = sm[i - 1], sm[i + 1]

        yaw = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
        qz, qw = yaw_to_quat_zw(yaw)
        out.append((sm[i][0], sm[i][1], qz, qw))

    if SEND_EVERY_N > 1:
        out = out[::SEND_EVERY_N]

    return out


def make_goal(frame_id, x, y, qz, qw):
    goal = NavigateToPose.Goal()
    ps = PoseStamped()
    ps.header.frame_id = frame_id
    ps.pose.position.x = float(x)
    ps.pose.position.y = float(y)
    ps.pose.orientation.z = float(qz)
    ps.pose.orientation.w = float(qw)
    goal.pose = ps
    return goal


def main():
    rclpy.init()
    node = rclpy.create_node("nav2_course_sender")

    client = ActionClient(node, NavigateToPose, "/navigate_to_pose")
    node.get_logger().info("Waiting for /navigate_to_pose action server...")
    if not client.wait_for_server(timeout_sec=20.0):
        node.get_logger().error("No /navigate_to_pose action server.")
        node.destroy_node()
        rclpy.shutdown()
        return

    smoothed = build_smoothed_waypoints(PTS)
    node.get_logger().info(f"Rough points: {len(PTS)} -> Goals: {len(smoothed)}")

    for i, (x, y, qz, qw) in enumerate(smoothed):
        node.get_logger().info(f"Goal {i + 1}/{len(smoothed)}: x={x:.3f}, y={y:.3f}")

        goal = make_goal("map", x, y, qz, qw)
        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, send_future)
        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            node.get_logger().error("Goal rejected.")
            break

        result_future = goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(node, timeout_sec=0.2)

        res = result_future.result()
        status = res.status if res is not None else -1
        node.get_logger().info(f"Finished status={status}")

        time.sleep(PAUSE_S)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()