#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from std_msgs.msg import Header
from .edep_common import edep_msg_types, extract_edep_value

def ensure_use_sim_time(node, default=True):
    """declare_parameter('use_sim_time', ...) が既に設定済みなら何もしない"""
    try:
        if not node.has_parameter('use_sim_time'):
            node.declare_parameter('use_sim_time', default)
    except Exception:
        # 既に宣言済み or 何らかの理由で失敗しても無視
        pass


class EdepGridNode(Node):
    """
    /odom と /g4/edep から 2D グリッドを作成して /g4/edep_grid を配信。
    グリッド値は Edep の累積（mode=sum/mean/max）を 0-100 に正規化して出力。
    """
    def __init__(self):
        super().__init__('edep_grid_node')
        # parameters
        ensure_use_sim_time(self, True)
        self.declare_parameter('resolution_m', 0.25)
        self.declare_parameter('range_m', 10.0)
        self.declare_parameter('decay_alpha', 1.0)
        self.declare_parameter('mode', 'sum')  # 'sum'|'mean'|'max'
        self.declare_parameter('edep_max', 1.0)
        self.declare_parameter('csv_dir', '')
        self.declare_parameter('pub_rate_hz', 5.0)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('edep_topic', '/g4/edep')

        self.res = float(self.get_parameter('resolution_m').value)
        self.rng = float(self.get_parameter('range_m').value)
        self.alpha = float(self.get_parameter('decay_alpha').value)
        self.mode = str(self.get_parameter('mode').value).lower()
        self.edep_max = float(self.get_parameter('edep_max').value)
        self.csv_dir = str(self.get_parameter('csv_dir').value)
        self.pub_rate = float(self.get_parameter('pub_rate_hz').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.edep_topic = str(self.get_parameter('edep_topic').value)

        if self.csv_dir:
            os.makedirs(self.csv_dir, exist_ok=True)

        # grid size & origin
        self.w = int(np.ceil(2*self.rng / self.res))
        self.h = int(np.ceil(2*self.rng / self.res))
        self.origin_xy = (-self.rng, -self.rng)

        # buffers
        self.accum = np.zeros((self.h, self.w), dtype=np.float32)
        self.count = np.zeros((self.h, self.w), dtype=np.int32)  # for mean
        self.odom_xy = None  # (x, y)

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1000)
        self.create_subscription(Odometry, self.odom_topic, self._on_odom, qos)
        self._subs = [self.create_subscription(t, self.edep_topic, self._on_edep, qos) for t in edep_msg_types()]

        self.pub_grid = self.create_publisher(OccupancyGrid, '/g4/edep_grid', 10)
        self.timer = self.create_timer(1.0/max(1e-3, self.pub_rate), self._on_tick)
        self.srv_reset = self.create_service(Empty, '~/reset', self._on_reset)

        self.get_logger().info(f'EDep grid started: {self.w}x{self.h}@{self.res}m mode={self.mode} edep_max={self.edep_max}')

    def _xy_to_rc(self, x: float, y: float):
        ox, oy = self.origin_xy
        c = int((x - ox) / self.res)
        r = int((y - oy) / self.res)
        if 0 <= r < self.h and 0 <= c < self.w:
            return r, c
        return None

    def _on_odom(self, msg: Odometry):
        self.odom_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _on_edep(self, msg):
        if self.odom_xy is None:
            return
        v = extract_edep_value(msg)
        if v is None:
            return
        rc = self._xy_to_rc(*self.odom_xy)
        if rc is None:
            return
        r, c = rc
        if self.mode == 'sum':
            self.accum[r, c] += v
        elif self.mode == 'max':
            self.accum[r, c] = max(self.accum[r, c], v)
        else:
            # mean
            self.accum[r, c] += v
            self.count[r, c] += 1

    def _on_tick(self):
        # decay
        if self.alpha < 1.0:
            self.accum *= self.alpha
            if self.mode == 'mean':
                self.count = (self.count.astype(np.float32) * self.alpha).astype(np.int32)

        if self.mode == 'mean':
            with np.errstate(divide='ignore', invalid='ignore'):
                grid_val = np.where(self.count > 0, self.accum / np.maximum(self.count, 1), 0.0)
        else:
            grid_val = self.accum

        # 0-100 へ正規化
        denom = max(self.edep_max, 1e-6)
        scaled = np.clip((grid_val / denom) * 100.0, 0.0, 100.0).astype(np.int8)

        # OccupancyGrid 生成（row-major -> 1D）
        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.info.resolution = self.res
        msg.info.width = self.w
        msg.info.height = self.h
        msg.info.origin.position.x = self.origin_xy[0]
        msg.info.origin.position.y = self.origin_xy[1]
        msg.info.origin.position.z = 0.0
        msg.data = scaled.flatten(order='C').tolist()
        self.pub_grid.publish(msg)

    def _on_reset(self, request, response):
        self.accum.fill(0.0)
        self.count.fill(0)
        self.get_logger().info('grid reset.')
        # ついでにCSV保存（任意）
        if self.csv_dir:
            try:
                np.savetxt(os.path.join(self.csv_dir, 'edep_grid.csv'), self.accum, delimiter=',')
            except Exception:
                pass
        return response

def main():
    rclpy.init()
    node = EdepGridNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
