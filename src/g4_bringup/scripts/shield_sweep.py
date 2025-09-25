#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
shield_sweep.py  (param-driven, sync wait, Float64)
- 角度ごとに /g4_odom_subscriber のパラメータ 'shield.angle_deg' を更新
- /odom を心拍送信して Geant4 を駆動
- /g4/edep(std_msgs/Float64) をタイムアウト付きで待つ
- 結果をCSV保存＋端末表示

実行:
  /usr/bin/python3 /home/morita/g4_ros_ws/src/g4_bringup/scripts/shield_sweep.py

ROS パラメータ:
  rotation_angles_deg: double[]  既定 [0,90,180,270,360,450,540,630]
  measure_timeout:     double    既定 2.0   # /g4/edep を待つ上限
  csv_path:            string    既定 /home/morita/g4_ros_ws/results/shield_sweep.csv
  param_node:          string    既定 "g4_odom_subscriber"  # geant4_embed_node の名前
  edep_topic:          string    既定 "/g4/edep"
"""

import os
import math
import csv
import time
from typing import Optional, List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters

from nav_msgs.msg import Odometry
from std_msgs.msg import Float64


def _ensure_dir(path: str) -> None:
    d = os.path.dirname(path)
    if d and not os.path.isdir(d):
        os.makedirs(d, exist_ok=True)


class ShieldSweep(Node):
    def __init__(self):
        super().__init__('shield_sweep')

        # ---- parameters ----
        self.declare_parameter('rotation_angles_deg',
                               [0.0, 90.0, 180.0, 270.0, 360.0, 450.0, 540.0, 630.0])
        self.declare_parameter('measure_timeout', 2.0)
        self.declare_parameter('csv_path', '/home/morita/g4_ros_ws/results/shield_sweep.csv')
        self.declare_parameter('param_node', 'g4_odom_subscriber')
        self.declare_parameter('edep_topic', '/g4/edep')

        self.angles_deg: List[float] = [float(x) for x in
                                        self.get_parameter('rotation_angles_deg').value]
        self.timeout: float = float(self.get_parameter('measure_timeout').value)
        self.csv_path: str = str(self.get_parameter('csv_path').value)
        self.param_node: str = str(self.get_parameter('param_node').value)
        self.edep_topic: str = str(self.get_parameter('edep_topic').value)

        # ---- comms ----
        self.setparam_cli = self.create_client(SetParameters, f'/{self.param_node}/set_parameters')
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        # EDep は Float64 を購読（混在対策として片方に固定）
        self._edep_future: Optional[Future] = None
        self.sub_edep = self.create_subscription(Float64, self.edep_topic, self._on_edep64, 10)

        _ensure_dir(self.csv_path)
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['angle_deg', 'edep', 'method'])

        self.get_logger().info(
            f'Angles={self.angles_deg}, timeout={self.timeout}s, csv={self.csv_path}, '
            f'param_node=/{self.param_node}, edep_topic={self.edep_topic}'
        )

    # ---------- callbacks ----------
    def _on_edep64(self, msg: Float64):
        if self._edep_future is not None and not self._edep_future.done():
            fut = self._edep_future
            self._edep_future = None
            fut.set_result(float(msg.data))

    # ---------- helpers ----------
    def _send_odom_heartbeat(self, n: int = 10, dt: float = 0.05):
        """G4 を駆動するための簡易 /odom。速度0、姿勢単位Quat。"""
        for _ in range(max(1, n)):
            msg = Odometry()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'odom'
            msg.child_frame_id = 'base_link'
            msg.pose.pose.orientation.w = 1.0
            self.pub_odom.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(max(0.0, dt))

    def _set_shield_angle(self, angle_deg: float) -> bool:
        if not self.setparam_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'set_parameters service not available on /{self.param_node}')
            return False
        req = SetParameters.Request()
        p = Parameter(name='shield.angle_deg', value=float(angle_deg)).to_parameter_msg()
        req.parameters.append(p)
        future = self.setparam_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if future.result() is None:
            self.get_logger().warn('set_parameters call failed (timeout)')
            return False
        # 結果判定
        for r in future.result().results:
            if not r.successful:
                self.get_logger().warn(f'set_parameters unsuccessful: {r.reason}')
                return False
        return True

    def _wait_edep_after_trigger(self, timeout: float) -> Optional[float]:
        """トリガ前に future をセット→トリガ→spin しながら待つ（取りこぼし防止）"""
        self._edep_future = Future()
        deadline = time.time() + max(0.0, float(timeout))
        # トリガ1: /odom 連続送信
        self._send_odom_heartbeat(n=6, dt=0.05)
        # 待機
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._edep_future is None:
                break
        val = None
        if self._edep_future and self._edep_future.done():
            try:
                val = float(self._edep_future.result())
            except Exception:
                val = None
        self._edep_future = None
        return val

    # ---------- main ----------
    def run(self):
        try:
            for a in self.angles_deg:
                # 1) 角度をノードパラメータで反映
                ok = self._set_shield_angle(a)
                method = 'param+edep' if ok else 'edep_only'

                # 2) EDep を同期取得（トリガは /odom）
                v = self._wait_edep_after_trigger(self.timeout)
                if v is None:
                    v = float('nan')
                    method += '_timeout'

                self.get_logger().info(f'angle_deg={a:.1f}  edep={v:.6f}  method={method}')
                self.csv_writer.writerow([f'{a:.1f}', f'{v:.9f}', method])
                self.csv_file.flush()

            self.get_logger().info('Sweep finished. CSV saved.')
        finally:
            try:
                self.csv_file.close()
            except Exception:
                pass


def main():
    rclpy.init()
    node = ShieldSweep()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
