#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FC(MAVROS) 与 外置 IMU(MAVLink) 姿态 / 空间位移验证节点.

数据源
------
- 飞控: MAVROS 发布
    /mavros/imu/data            (EKF 姿态, ENU)
    /mavros/local_position/pose (位置, frame 通常为 map)
- 外置 IMU: L431_ADI 固件通过 MAVLink v2 HIGHRES_IMU 输出
    → livox_ros_driver2 ExtImuBridge → /livox/imu (角速度/加速度, 无姿态)

处理
----
- 对外置 IMU 角速度做四元数积分得到姿态 (运行 1s 后与飞控航向对齐一次)
- 外置 IMU 安装于飞控右侧 5 cm (ENU: x前 y左 z上 → 默认偏移 [0, -0.05, 0])
- 发布 TF:
      <fc_pose_frame> → fc_base → fc       (飞控姿态)
                              └→ ext_imu   (外置IMU姿态 + 右侧5cm平移)
- 发布两条轨迹 Path: /verify/fc_path, /verify/ext_imu_path (空间位移)
- 发布姿态差: /verify/attitude_diff (roll/pitch/yaw, 度)

用法
----
    ros2 run fc_ext_imu_verify verify_node
    或
    ros2 launch fc_ext_imu_verify verify.launch.py
"""

import json
import math
import threading

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3
from nav_msgs.msg import Path
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# mavros 发布为 BEST_EFFORT, 必须匹配否则收不到数据
MAVROS_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10)


# ── 四元数工具 (约定: (x, y, z, w), 与 geometry_msgs/Quaternion 一致) ──
def quat_normalize(q):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / n, y / n, z / n, w / n)


def quat_multiply(q, r):
    """q ⊗ r"""
    x0, y0, z0, w0 = q
    x1, y1, z1, w1 = r
    return (w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1,
            w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1,
            w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1,
            w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1)


def quat_from_axis_angle(axis, angle):
    """绕 axis(自动归一化) 旋转 angle 弧度."""
    x, y, z = axis
    n = math.sqrt(x * x + y * y + z * z)
    if n < 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    s = math.sin(angle * 0.5)
    return (x * s / n, y * s / n, z * s / n, math.cos(angle * 0.5))


def quat_rotate(q, v):
    """v' = q ⊗ v ⊗ q⁻¹"""
    x, y, z, w = q
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (vx + w * tx + y * tz - z * ty,
            vy + w * ty + z * tx - x * tz,
            vz + w * tz + x * ty - y * tx)


def quat_to_rpy(q):
    x, y, z, w = q
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def wrap_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class FcExtImuVerify(Node):
    """对比飞控与外置 IMU 的姿态/位移, 输出 TF、Path 与姿态差."""

    def __init__(self):
        super().__init__('fc_ext_imu_verify')

        # ── 参数 ──
        self.declare_parameter('fc_imu_topic', '/mavros/imu/data')
        self.declare_parameter('fc_pose_topic', '/mavros/local_position/pose')
        self.declare_parameter('ext_imu_topic', '/livox/imu')
        self.declare_parameter('ext_imu_offset', [0.0, -0.05, 0.0])  # ENU: y 向左 → 右侧 = -y
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('align_initial_yaw', True)
        self.declare_parameter('use_ext_orientation', True)
        self.declare_parameter('max_path_points', 4000)

        fc_imu_topic = self.get_parameter('fc_imu_topic').value
        fc_pose_topic = self.get_parameter('fc_pose_topic').value
        ext_imu_topic = self.get_parameter('ext_imu_topic').value
        self._offset = self._parse_offset(self.get_parameter('ext_imu_offset').value)
        self._publish_rate = float(self.get_parameter('publish_rate').value)
        self._align_requested = bool(self.get_parameter('align_initial_yaw').value)
        self._use_ext_orientation = bool(self.get_parameter('use_ext_orientation').value)
        self._max_path_points = int(self.get_parameter('max_path_points').value)

        # ── 状态 ──
        self._lock = threading.Lock()
        self._fc_att = None            # (x, y, z, w)
        self._fc_pos = None            # (x, y, z)
        self._fc_frame = 'map'
        self._fc_ang = 0.0             # 飞控角速度模 (rad/s, 静止检测)
        self._ext_ang = 0.0            # 外置 IMU 角速度模 (rad/s, 静止检测)
        self._fc_acc = 0.0             # 飞控比力幅值 (m/s²)
        self._ext_acc = 0.0            # 外置 IMU 比力幅值 (m/s²)
        self._ext_q = (0.0, 0.0, 0.0, 1.0)
        self._ext_yaw_bias = 0.0        # 持久航向偏置 (rad), 逐帧施加到原始姿态
        self._ext_last_time = None     # rclpy Time
        self._ext_runtime = 0.0
        self._aligned = False

        self._fc_imu_count = 0
        self._fc_pose_count = 0
        self._ext_imu_count = 0

        # ── 订阅 ──
        self._fc_imu_sub = self.create_subscription(Imu, fc_imu_topic, self._fc_imu_cb, MAVROS_QOS)
        self._fc_pose_sub = self.create_subscription(PoseStamped, fc_pose_topic, self._fc_pose_cb, MAVROS_QOS)
        self._ext_imu_sub = self.create_subscription(Imu, ext_imu_topic, self._ext_imu_cb, 200)

        # ── 发布 ──
        self._tf_bc = TransformBroadcaster(self)
        self._att_diff_pub = self.create_publisher(Vector3, '/verify/attitude_diff', 10)
        self._fc_path_pub = self.create_publisher(Path, '/verify/fc_path', 10)
        self._ext_path_pub = self.create_publisher(Path, '/verify/ext_imu_path', 10)

        self._fc_path = Path()
        self._ext_path = Path()

        # ── 定时器 ──
        period = 1.0 / max(1.0, self._publish_rate)
        self._pub_timer = self.create_timer(period, self._publish_timer_cb)
        self._log_timer = self.create_timer(1.0, self._log_timer_cb)

        self.get_logger().info(
            'FC IMU: %s | FC pose: %s | 外置IMU: %s | 外置IMU偏移: %s m'
            % (fc_imu_topic, fc_pose_topic, ext_imu_topic, list(self._offset)))

    # ── 参数解析 ────────────────────────────────────────────────────
    def _parse_offset(self, value):
        try:
            if isinstance(value, (list, tuple)) and len(value) == 3:
                arr = [float(v) for v in value]
            elif isinstance(value, str):
                arr = [float(v) for v in json.loads(value)]
            else:
                raise ValueError(f'invalid value: {value!r}')
            return (arr[0], arr[1], arr[2])
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(
                'ext_imu_offset 解析失败 (%s), 使用默认 [0, -0.05, 0]' % e)
            return (0.0, -0.05, 0.0)

    # ── 订阅回调 ────────────────────────────────────────────────────
    def _fc_imu_cb(self, msg):
        o = msg.orientation
        av = msg.angular_velocity
        la = msg.linear_acceleration
        q = quat_normalize((o.x, o.y, o.z, o.w))
        with self._lock:
            self._fc_att = q
            self._fc_ang = math.sqrt(av.x * av.x + av.y * av.y + av.z * av.z)
            self._fc_acc = math.sqrt(la.x * la.x + la.y * la.y + la.z * la.z)
            self._fc_imu_count += 1
        self._maybe_align()

    def _fc_pose_cb(self, msg):
        p = msg.pose.position
        with self._lock:
            self._fc_pos = (p.x, p.y, p.z)
            self._fc_frame = msg.header.frame_id if msg.header.frame_id else 'map'
            self._fc_pose_count += 1

    def _ext_imu_cb(self, msg):
        use_orient = self._use_ext_orientation and msg.orientation_covariance[0] >= 0.0
        av = msg.angular_velocity
        la = msg.linear_acceleration
        w = math.sqrt(av.x * av.x + av.y * av.y + av.z * av.z)
        now = rclpy.time.Time.from_msg(msg.header.stamp)
        with self._lock:
            dt = 0.0
            if self._ext_last_time is not None:
                dt = (now - self._ext_last_time).nanoseconds * 1e-9
            if use_orient:
                # 桥接器已解算 ENU 姿态 (固件 ATTITUDE/Mahony), 施加持久航向偏置
                o = msg.orientation
                q_raw = quat_normalize((o.x, o.y, o.z, o.w))
                qb = quat_from_axis_angle((0.0, 0.0, 1.0), self._ext_yaw_bias)
                self._ext_q = quat_normalize(quat_multiply(qb, q_raw))
                if 0.0 < dt < 0.1:
                    self._ext_runtime += dt
            else:
                # 无姿态时退化为角速度积分
                if 0.0 < dt < 0.1:
                    dq = quat_from_axis_angle((av.x, av.y, av.z), w * dt)
                    self._ext_q = quat_normalize(quat_multiply(self._ext_q, dq))
                    self._ext_runtime += dt
            self._ext_ang = w
            self._ext_acc = math.sqrt(la.x * la.x + la.y * la.y + la.z * la.z)
            self._ext_last_time = now
            self._ext_imu_count += 1
        self._maybe_align()

    def _maybe_align(self):
        """外置 IMU 初始航向未知: 运行 1s 后与飞控航向对齐一次."""
        if not self._align_requested:
            return
        dyaw = None
        with self._lock:
            if self._aligned or self._fc_att is None or self._ext_runtime < 1.0:
                return
            _, _, fc_yaw = quat_to_rpy(self._fc_att)
            _, _, ext_yaw = quat_to_rpy(self._ext_q)
            dyaw = wrap_pi(fc_yaw - ext_yaw)
            self._ext_yaw_bias = wrap_pi(self._ext_yaw_bias + dyaw)
            qz = quat_from_axis_angle((0.0, 0.0, 1.0), dyaw)
            self._ext_q = quat_normalize(quat_multiply(qz, self._ext_q))
            self._aligned = True
        self.get_logger().info('外置 IMU 航向已与飞控对齐 (Δyaw=%.2f°)' % math.degrees(dyaw))

    def _maybe_relock(self):
        """准静止时持续把外置 IMU 航向重新对齐到飞控 (Mahony 无磁航向会漂移)."""
        if not self._align_requested:
            return
        dyaw = None
        with self._lock:
            if not self._aligned or self._fc_att is None:
                return
            # 旋转较快时不锁 (避免掩盖真实旋转)
            if self._fc_ang > 0.08 or self._ext_ang > 0.08:
                return
            # 比力偏离重力较远时不锁 (机动加速段)
            if abs(self._fc_acc - 9.81) > 0.6 or abs(self._ext_acc - 9.81) > 0.6:
                return
            _, _, fc_yaw = quat_to_rpy(self._fc_att)
            _, _, ext_yaw = quat_to_rpy(self._ext_q)
            dyaw = wrap_pi(fc_yaw - ext_yaw)
            if abs(dyaw) < math.radians(0.5):
                return
            self._ext_yaw_bias = wrap_pi(self._ext_yaw_bias + dyaw)
            qz = quat_from_axis_angle((0.0, 0.0, 1.0), dyaw)
            self._ext_q = quat_normalize(quat_multiply(qz, self._ext_q))
        self.get_logger().info('准静止航向重锁 (Δyaw=%.2f°, 累计偏置=%.1f°)'
                              % (math.degrees(dyaw), math.degrees(self._ext_yaw_bias)))

    # ── 定时器 ──────────────────────────────────────────────────────
    def _publish_timer_cb(self):
        with self._lock:
            fc_att = self._fc_att
            fc_pos = self._fc_pos
            fc_frame = self._fc_frame
            ext_q = self._ext_q

        now = self.get_clock().now().to_msg()

        # fc_base: 父坐标系 = 飞控位置 (无位置时固定在原点)
        base = TransformStamped()
        base.header.stamp = now
        base.header.frame_id = fc_frame
        base.child_frame_id = 'fc_base'
        if fc_pos is not None:
            base.transform.translation.x = fc_pos[0]
            base.transform.translation.y = fc_pos[1]
            base.transform.translation.z = fc_pos[2]
        base.transform.rotation.w = 1.0
        self._tf_bc.sendTransform(base)

        # fc: 飞控姿态 (EKF, ENU)
        fc_tf = TransformStamped()
        fc_tf.header.stamp = now
        fc_tf.header.frame_id = 'fc_base'
        fc_tf.child_frame_id = 'fc'
        if fc_att is not None:
            fc_tf.transform.rotation.x = fc_att[0]
            fc_tf.transform.rotation.y = fc_att[1]
            fc_tf.transform.rotation.z = fc_att[2]
            fc_tf.transform.rotation.w = fc_att[3]
        else:
            fc_tf.transform.rotation.w = 1.0
        self._tf_bc.sendTransform(fc_tf)

        # ext_imu: 外置 IMU (飞控右侧 5cm, 姿态为积分结果)
        ext_tf = TransformStamped()
        ext_tf.header.stamp = now
        ext_tf.header.frame_id = 'fc_base'
        ext_tf.child_frame_id = 'ext_imu'
        ext_tf.transform.translation.x = self._offset[0]
        ext_tf.transform.translation.y = self._offset[1]
        ext_tf.transform.translation.z = self._offset[2]
        ext_tf.transform.rotation.x = ext_q[0]
        ext_tf.transform.rotation.y = ext_q[1]
        ext_tf.transform.rotation.z = ext_q[2]
        ext_tf.transform.rotation.w = ext_q[3]
        self._tf_bc.sendTransform(ext_tf)

        # 空间位移轨迹 (外置 IMU = 飞控位置 + 刚体旋转的 5cm 杆臂)
        if fc_pos is not None:
            self._append_path(self._fc_path, now, fc_frame, fc_pos)
            ext_pos = fc_pos
            if fc_att is not None:
                ext_pos = tuple(a + b for a, b in zip(fc_pos, quat_rotate(fc_att, self._offset)))
            else:
                ext_pos = tuple(a + b for a, b in zip(fc_pos, self._offset))
            self._append_path(self._ext_path, now, fc_frame, ext_pos)
            self._fc_path_pub.publish(self._fc_path)
            self._ext_path_pub.publish(self._ext_path)

        # 姿态差 (度)
        if fc_att is not None:
            fr, fp, fy = quat_to_rpy(fc_att)
            er, ep, ey = quat_to_rpy(ext_q)
            diff = Vector3()
            diff.x = math.degrees(wrap_pi(fr - er))
            diff.y = math.degrees(wrap_pi(fp - ep))
            diff.z = math.degrees(wrap_pi(fy - ey))
            self._att_diff_pub.publish(diff)

    def _append_path(self, path, now, frame, pos):
        if len(path.poses) >= self._max_path_points:
            path.poses.pop(0)
        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = frame
        pose.pose.position.x = pos[0]
        pose.pose.position.y = pos[1]
        pose.pose.position.z = pos[2]
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)

    def _log_timer_cb(self):
        self._maybe_relock()
        with self._lock:
            fc_att = self._fc_att
            fc_pos = self._fc_pos
            ext_q = self._ext_q
            n_fc_imu = self._fc_imu_count
            n_fc_pose = self._fc_pose_count
            n_ext = self._ext_imu_count
            self._fc_imu_count = 0
            self._fc_pose_count = 0
            self._ext_imu_count = 0
            aligned = self._aligned

        parts = []
        if fc_att is not None:
            r, p, y = quat_to_rpy(fc_att)
            parts.append(f'FC: roll={math.degrees(r):6.1f} pitch={math.degrees(p):6.1f} yaw={math.degrees(y):6.1f}°')
        else:
            parts.append('FC: 无数据 (检查 MAVROS)')

        r, p, y = quat_to_rpy(ext_q)
        tag = '' if aligned else ' [未对齐]'
        parts.append(f'EXT: roll={math.degrees(r):6.1f} pitch={math.degrees(p):6.1f} yaw={math.degrees(y):6.1f}°{tag}')

        if fc_att is not None:
            fr, fp, fy = quat_to_rpy(fc_att)
            er, ep, ey = quat_to_rpy(ext_q)
            parts.append(
                f'Δroll/pitch/yaw: {math.degrees(wrap_pi(fr - er)):5.1f} / '
                f'{math.degrees(wrap_pi(fp - ep)):5.1f} / {math.degrees(wrap_pi(fy - ey)):5.1f}°')

        if fc_pos is not None:
            parts.append(f'FC位置: ({fc_pos[0]:.2f}, {fc_pos[1]:.2f}, {fc_pos[2]:.2f}) m')

        self.get_logger().info(' | '.join(parts))
        self.get_logger().info('Hz: fc_imu=%d fc_pose=%d ext_imu=%d' % (n_fc_imu, n_fc_pose, n_ext))


def main(args=None):
    rclpy.init(args=args)
    node = FcExtImuVerify()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
