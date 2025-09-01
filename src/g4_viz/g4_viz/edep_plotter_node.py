#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import time
from collections import deque
import numpy as np
import matplotlib
matplotlib.use('Agg')  # ヘッドレスでPNG出力
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
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


class EdepPlotter(Node):
    """
    /g4/edep を受けて時系列PNGとCSVを定期出力
    """
    def __init__(self):
        super().__init__('edep_plotter')
        ensure_use_sim_time(self, True)
        self.declare_parameter('edep_max', 1.0)
        self.declare_parameter('csv_dir', '')
        self.declare_parameter('window_size', 1024)
        self.declare_parameter('save_interval_sec', 2.0)
        self.declare_parameter('topic', '/g4/edep')

        self.use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        self.edep_max = float(self.get_parameter('edep_max').value)
        self.csv_dir = str(self.get_parameter('csv_dir').value)
        self.window_size = int(self.get_parameter('window_size').value)
        self.save_interval = float(self.get_parameter('save_interval_sec').value)
        self.topic = str(self.get_parameter('topic').value)

        if self.csv_dir:
            os.makedirs(self.csv_dir, exist_ok=True)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1000
        )

        self._xs = deque(maxlen=self.window_size)
        self._ys = deque(maxlen=self.window_size)

        # 型の異なる二重購読（合致する方のみ接続される）
        self._subs = []
        for t in edep_msg_types():
            self._subs.append(self.create_subscription(t, self.topic, self._on_edep, qos))

        self._last_save = 0.0
        self._timer = self.create_timer(0.2, self._on_tick)
        self.get_logger().info(f'EDep plotter started: topic={self.topic} edep_max={self.edep_max} csv_dir={self.csv_dir}')

    def _now(self) -> float:
        # rclpy.clock は Time 型で来るので、ここは壁時計で良い（PNG/CSV命名用）
        return time.time()

    def _on_edep(self, msg):
        v = extract_edep_value(msg)
        if v is None:
            return
        t = self._now()
        self._xs.append(t)
        self._ys.append(v)

    def _on_tick(self):
        if not self._ys:
            return
        if self._now() - self._last_save < self.save_interval:
            return
        self._last_save = self._now()

        xs = np.array(self._xs)
        ys = np.array(self._ys)

        # 時刻を t0 からの相対秒に直して描画
        xs_rel = xs - xs[0]
        fig = plt.figure(figsize=(8, 3))
        ax = fig.add_subplot(111)
        ax.plot(xs_rel, ys)
        ax.set_xlabel('time [s]')
        ax.set_ylabel('Edep [MeV]')
        ax.set_ylim(0, max(self.edep_max, float(np.max(ys)) if ys.size else 1.0))
        ax.grid(True)
        fig.tight_layout()

        if self.csv_dir:
            png = os.path.join(self.csv_dir, 'edep_timeseries.png')
            csv = os.path.join(self.csv_dir, 'edep_timeseries.csv')
            try:
                fig.savefig(png, dpi=120)
                np.savetxt(csv, np.vstack([xs_rel, ys]).T, delimiter=',', header='t_sec,edep_mev', comments='')
                self.get_logger().debug(f'saved: {png}, {csv}')
            except Exception as e:
                self.get_logger().warn(f'failed to save plot/csv: {e}')
        plt.close(fig)

def main():
    rclpy.init()
    node = EdepPlotter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
