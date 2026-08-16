#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
save_map.py — 独立的 FAST-LIO 地图保存脚本

订阅 FAST-LIO 发布的累积地图话题(默认 /Laser_map),
按回车键(或等待 --timeout 秒)后把最新一帧地图保存为 PCD 文件。

用法:
    ros2 run fast_lio save_map.py
    ros2 run fast_lio save_map.py --topic /Laser_map --output ./map.pcd
    ros2 run fast_lio save_map.py --timeout 10 --output ./map.pcd   # 10 秒后自动保存

    # 直接调用 FAST-LIO 节点内置的 map_save 服务(保存到配置文件里的 map_file_path):
    ros2 run fast_lio save_map.py --use-service

前提:
    FAST-LIO 配置中 publish.map_en = true(默认配置已开启),
    否则 /Laser_map 话题没有数据。
"""

import argparse
import os
import sys
import time
from datetime import datetime

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import PointCloud2, PointField
from std_srvs.srv import Trigger

# ROS PointField 数据类型 -> numpy dtype 映射
_TYPE_MAP = {
    PointField.INT8: np.dtype('int8'),
    PointField.UINT8: np.dtype('uint8'),
    PointField.INT16: np.dtype('int16'),
    PointField.UINT16: np.dtype('uint16'),
    PointField.INT32: np.dtype('int32'),
    PointField.UINT32: np.dtype('uint32'),
    PointField.FLOAT32: np.dtype('float32'),
    PointField.FLOAT64: np.dtype('float64'),
}


def pc2_to_numpy(msg):
    """把 sensor_msgs/PointCloud2 转成 (N, 4) float32 数组, 列为 x y z intensity."""
    names = [f.name for f in msg.fields]
    formats = [_TYPE_MAP[f.datatype] for f in msg.fields]
    offsets = [f.offset for f in msg.fields]

    dtype = np.dtype({
        'names': names,
        'formats': formats,
        'offsets': offsets,
        'itemsize': msg.point_step,
    })
    arr = np.frombuffer(msg.data, dtype=dtype)

    cols = []
    for name in ('x', 'y', 'z'):
        if name not in names:
            raise RuntimeError(f"点云缺少字段: {name}")
        cols.append(arr[name].astype(np.float32))

    if 'intensity' in names:
        cols.append(arr['intensity'].astype(np.float32))
    else:
        cols.append(np.zeros(len(arr), dtype=np.float32))

    return np.column_stack(cols)


def write_pcd(path, points, binary=True):
    """把 (N, 4) float32 数组(x y z intensity)写成 PCD 文件."""
    n = points.shape[0]
    header = [
        '# .PCD v0.7 - Point Cloud Data file format',
        'VERSION 0.7',
        'FIELDS x y z intensity',
        'SIZE 4 4 4 4',
        'TYPE F F F F',
        'COUNT 1 1 1 1',
        f'WIDTH {n}',
        'HEIGHT 1',
        'VIEWPOINT 0 0 0 1 0 0 0',
        f'POINTS {n}',
    ]

    if binary:
        header.append('DATA binary')
        with open(path, 'wb') as f:
            f.write(('\n'.join(header) + '\n').encode('ascii'))
            f.write(points.astype('<f4').tobytes())
    else:
        header.append('DATA ascii')
        with open(path, 'w') as f:
            f.write('\n'.join(header) + '\n')
            for p in points:
                f.write(f'{p[0]:.6f} {p[1]:.6f} {p[2]:.6f} {p[3]:.3f}\n')


class MapSaver(Node):
    def __init__(self, topic, output, timeout, binary):
        super().__init__('map_saver')
        self.output = output
        self.timeout = timeout
        self.binary = binary
        self.latest_msg = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
        self.sub = self.create_subscription(
            PointCloud2, topic, self._cb, qos)

    def _cb(self, msg):
        self.latest_msg = msg
        n = msg.width * msg.height
        self.get_logger().info(
            f'收到地图点云: {n} 点', throttle_duration_sec=2.0)

    def has_data(self):
        return self.latest_msg is not None and \
            self.latest_msg.width * self.latest_msg.height > 0

    def save(self):
        points = pc2_to_numpy(self.latest_msg)
        path = self.output
        if path is None:
            path = 'fastlio_map_' + \
                datetime.now().strftime('%Y%m%d_%H%M%S') + '.pcd'
        write_pcd(path, points, self.binary)
        self.get_logger().info(
            f'地图已保存: {path} ({points.shape[0]} 点)')
        return path


def call_save_service(node):
    """调用 FAST-LIO 节点内置的 map_save 服务."""
    cli = node.create_client(Trigger, 'map_save')
    if not cli.wait_for_service(timeout_sec=5.0):
        node.get_logger().error('等待 map_save 服务超时, 请确认 FAST-LIO 正在运行')
        return False

    req = Trigger.Request()
    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    if future.result() is None:
        node.get_logger().error('map_save 服务调用超时')
        return False

    res = future.result()
    if res.success:
        node.get_logger().info(f'服务保存成功: {res.message}')
    else:
        node.get_logger().error(f'服务保存失败: {res.message}')
    return res.success


def main():
    parser = argparse.ArgumentParser(description='FAST-LIO 地图保存脚本')
    parser.add_argument('--topic', default='/Laser_map',
                        help='地图话题名 (默认 /Laser_map)')
    parser.add_argument('--output', '-o', default=None,
                        help='输出 PCD 文件路径 (默认按时间戳命名)')
    parser.add_argument('--timeout', type=float, default=0.0,
                        help='等待 N 秒后自动保存, 0 表示按回车键手动保存')
    parser.add_argument('--ascii', action='store_true',
                        help='保存为 ASCII 格式 (默认二进制)')
    parser.add_argument('--use-service', action='store_true',
                        help='调用 FAST-LIO 内置 map_save 服务后退出')
    args = parser.parse_args()

    rclpy.init()
    node = MapSaver(args.topic, args.output, args.timeout,
                    binary=not args.ascii)

    try:
        if args.use_service:
            ok = call_save_service(node)
            sys.exit(0 if ok else 1)

        # 等待第一帧地图数据
        node.get_logger().info(f'等待地图话题 {args.topic} ...')
        while rclpy.ok() and not node.has_data():
            rclpy.spin_once(node, timeout_sec=0.1)

        if not node.has_data():
            node.get_logger().error('未收到任何地图数据, 请检查 publish.map_en 是否为 true')
            sys.exit(1)

        node.get_logger().info('已收到地图数据')

        if args.timeout > 0:
            deadline = time.time() + args.timeout
            while rclpy.ok() and time.time() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            node.save()
        else:
            input('按回车键保存地图 ...\n')
            # 按回车后短暂继续接收, 确保拿到最新地图
            end = time.time() + 1.0
            while rclpy.ok() and time.time() < end:
                rclpy.spin_once(node, timeout_sec=0.1)
            node.save()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
