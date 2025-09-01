#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, Int32MultiArray
from .edep_common import edep_msg_types, extract_edep_value

def ensure_use_sim_time(node, default=True):
    """declare_parameter('use_sim_time', ...) が既に設定済みなら何もしない"""
    try:
        if not node.has_parameter('use_sim_time'):
            node.declare_parameter('use_sim_time', default)
    except Exception:
        # 既に宣言済み or 何らかの理由で失敗しても無視
        pass


class EdepHistNode(Node):
    """
    /g4/edep をヒスト化して edges / counts / stats を配信し、CSV/PNG 保存
    """
    def __init__(self):
        super().__init__('edep_hist_node')
        ensure_use_sim_time(self, True)
        self.declare_parameter('csv_dir', '')
        self.declare_parameter('edep_max', 1.0)
        self.declare_parameter('bins', 50)
        self.declare_parameter('topic', '/g4/edep')
        self.declare_parameter('save_interval_sec', 2.0)

        self.csv_dir = str(self.get_parameter('csv_dir').value)
        self.edep_max = float(self.get_parameter('edep_max').value)
        self.bins = int(self.get_parameter('bins').value)
        self.topic = str(self.get_parameter('topic').value)
        self.save_interval = float(self.get_parameter('save_interval_sec').value)

        if self.csv_dir:
            os.makedirs(self.csv_dir, exist_ok=True)

        self.edges = np.linspace(0.0, self.edep_max, self.bins + 1, dtype=np.float32)
        self.counts = np.zeros(self.bins, dtype=np.int32)
        self._samples = []

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1000)

        self._subs = [self.create_subscription(t, self.topic, self._on_edep, qos)
                      for t in edep_msg_types()]

        self.pub_edges = self.create_publisher(Float32MultiArray, '/g4/edep_hist_edges', 10)
        self.pub_counts = self.create_publisher(Int32MultiArray, '/g4/edep_hist_counts', 10)
        self.pub_stats = self.create_publisher(Float32MultiArray, '/g4/edep_stats', 10)

        self._last_save = self.get_clock().now()

        self.timer = self.create_timer(0.5, self._on_tick)
        self.get_logger().info(f'EDep histogram started: max={self.edep_max} bins={self.bins} csv_dir={self.csv_dir}')

    def _on_edep(self, msg):
        v = extract_edep_value(msg)
        if v is None:
            return
        self._samples.append(v)
        # 1サンプルごと即時ビン追加
        idx = np.searchsorted(self.edges, v, side='right') - 1
        if 0 <= idx < self.counts.size:
            self.counts[idx] += 1

    def _on_tick(self):
        # publish edges
        mea = Float32MultiArray()
        mea.layout.dim = [MultiArrayDimension(label='edge', size=len(self.edges), stride=len(self.edges))]
        mea.data = self.edges.tolist()
        self.pub_edges.publish(mea)

        cnt = Int32MultiArray()
        cnt.layout.dim = [MultiArrayDimension(label='bin', size=len(self.counts), stride=len(self.counts))]
        cnt.data = self.counts.tolist()
        self.pub_counts.publish(cnt)

        # stats: [n, mean, std, min, max]
        if self._samples:
            arr = np.array(self._samples, dtype=np.float32)
            stats = [float(arr.size), float(arr.mean()), float(arr.std(ddof=0)), float(arr.min()), float(arr.max())]
        else:
            stats = [0.0, 0.0, 0.0, 0.0, 0.0]
        st = Float32MultiArray()
        st.layout.dim = [MultiArrayDimension(label='stat', size=5, stride=5)]
        st.data = stats
        self.pub_stats.publish(st)

        # save CSV/PNG 時刻条件
        now = self.get_clock().now()
        if (now - self._last_save).nanoseconds * 1e-9 < self.save_interval:
            return
        self._last_save = now
        try:
            if self.csv_dir:
                import matplotlib
                matplotlib.use('Agg')
                import matplotlib.pyplot as plt
                np.savetxt(os.path.join(self.csv_dir, 'edep_hist_edges.csv'), self.edges, delimiter=',', header='edge_mev', comments='')
                np.savetxt(os.path.join(self.csv_dir, 'edep_hist_counts.csv'), self.counts, delimiter=',', header='count', comments='')
                fig = plt.figure(figsize=(4,3))
                ax = fig.add_subplot(111)
                ax.bar(0.5*(self.edges[:-1]+self.edges[1:]), self.counts, width=np.diff(self.edges), align='center')
                ax.set_xlabel('Edep [MeV]'); ax.set_ylabel('count'); ax.grid(True)
                fig.tight_layout()
                fig.savefig(os.path.join(self.csv_dir, 'edep_hist.png'), dpi=120)
                plt.close(fig)
        except Exception as e:
            self.get_logger().warn(f'failed to save hist: {e}')

def main():
    rclpy.init()
    node = EdepHistNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
