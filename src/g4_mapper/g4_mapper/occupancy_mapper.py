#!/usr/bin/env python3
import math
import numpy as np
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import Float32, Float64
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose

class EdepOccupancyMapper(Node):
    """
    /odom と /g4/edep(Float32/Float64 想定) を購読し、2Dグリッドに強度を蓄積。
    一定周期で OccupancyGrid として /g4/edep_grid を配信。
    data は 0..100 で正規化した“強度ヒートマップ”。
    """

    def __init__(self):
        super().__init__('g4_edep_occupancy_mapper')

        # ====== パラメータ ======
        self.declare_parameter('origin_x', 0.0)     # マップ左下X
        self.declare_parameter('origin_y', 0.0)     # マップ左下Y
        self.declare_parameter('resolution', 0.25)  # [m/cell]
        self.declare_parameter('width', 80)         # [cells]
        self.declare_parameter('height', 80)        # [cells]

        self.declare_parameter('odom_topic', '/odom')
        # Float32/Float64 どちらの発行にも対応できるよう、2本の購読を用意
        self.declare_parameter('edep_topic', '/g4/edep')         # Float32 用
        self.declare_parameter('edep_topic_f64', '/g4/edep')     # Float64 用（同名でOK）

        self.declare_parameter('grid_topic', '/g4/edep_grid')
        self.declare_parameter('publish_rate_hz', 5.0)

        self.declare_parameter('decay_factor', 0.0)  # 0.0=無効, 例:0.99
        self.declare_parameter('cell_cap', 1e6)
        self.declare_parameter('frame_id', 'map')

        # ====== 取得 ======
        self.origin_x = float(self.get_parameter('origin_x').value)
        self.origin_y = float(self.get_parameter('origin_y').value)
        self.resolution = float(self.get_parameter('resolution').value)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)

        self.odom_topic = self.get_parameter('odom_topic').value
        self.edep_topic = self.get_parameter('edep_topic').value
        self.edep_topic_f64 = self.get_parameter('edep_topic_f64').value

        self.grid_topic = self.get_parameter('grid_topic').value
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self.decay_factor = float(self.get_parameter('decay_factor').value)
        self.cell_cap = float(self.get_parameter('cell_cap').value)
        self.frame_id = self.get_parameter('frame_id').value

        # ====== 状態 ======
        self.latest_odom: Optional[Odometry] = None
        self.grid_accum = np.zeros((self.height, self.width), dtype=np.float64)
        self.max_seen_value = 1e-9  # 正規化用（0割回避）

        # ====== QoS ======
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        qos_latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        # ====== I/O ======
        self.sub_odom = self.create_subscription(Odometry, self.odom_topic, self.cb_odom, qos_sensor)

        # Float32 と Float64 の両方を“同名トピック”で待ち受ける
        self.sub_edep32 = self.create_subscription(Float32, self.edep_topic, self.cb_edep32, qos_sensor)
        self.sub_edep64 = self.create_subscription(Float64, self.edep_topic_f64, self.cb_edep64, qos_sensor)

        self.pub_grid = self.create_publisher(OccupancyGrid, self.grid_topic, qos_latched)
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.publish_grid)

        # メタ情報
        self.map_info = MapMetaData()
        self.map_info.resolution = self.resolution
        self.map_info.width = self.width
        self.map_info.height = self.height
        self.map_info.origin = Pose()
        self.map_info.origin.position.x = self.origin_x
        self.map_info.origin.position.y = self.origin_y
        self.map_info.origin.position.z = 0.0  # quaternionはデフォルト(0,0,0,1)

        self.get_logger().info(
            f'Init mapper: area=({self.width}x{self.height}) res={self.resolution} origin=({self.origin_x},{self.origin_y}) '
            f'odom="{self.odom_topic}" edep32="{self.edep_topic}" edep64="{self.edep_topic_f64}" -> grid="{self.grid_topic}"'
        )

    # ========== コールバック ==========
    def cb_odom(self, msg: Odometry):
        self.latest_odom = msg

    def cb_edep32(self, msg: Float32):
        self._accumulate_value(float(msg.data))

    def cb_edep64(self, msg: Float64):
        self._accumulate_value(float(msg.data))

    def _accumulate_value(self, val: float):
        if self.latest_odom is None:
            return

        x = self.latest_odom.pose.pose.position.x
        y = self.latest_odom.pose.pose.position.y

        ix = int(math.floor((x - self.origin_x) / self.resolution))
        iy = int(math.floor((y - self.origin_y) / self.resolution))

        if 0 <= ix < self.width and 0 <= iy < self.height:
            if 0.0 < self.decay_factor < 1.0:
                self.grid_accum *= self.decay_factor  # 全体減衰（簡易）

            new_v = min(self.grid_accum[iy, ix] + val, self.cell_cap)
            self.grid_accum[iy, ix] = new_v
            if new_v > self.max_seen_value:
                self.max_seen_value = new_v

    # ========== 出力 ==========
    def publish_grid(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.info = self.map_info

        norm = (self.grid_accum / self.max_seen_value) * 100.0
        norm = np.clip(norm, 0.0, 100.0)

        # RVizの見え方に合わせて上下反転
        norm_flip = np.flipud(norm)

        msg.data = norm_flip.astype(np.int8).flatten().tolist()
        self.pub_grid.publish(msg)


def main():
    rclpy.init()
    node = EdepOccupancyMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
