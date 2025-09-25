#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
robot_measurement_node.py

Gazebo上の移動ロボットを計測点へ移動させ、各点で遮蔽体角度を順に変えつつ
STEP2 の新インタフェース:
  - /measurement_request (g4_interfaces/MeasurementRequest)
  - /g4/measure          (g4_interfaces/Measurement サービス, 任意)
を用いて計測をトリガするノード。

動作:
- /odom を購読して現在位置を取得
- /cmd_vel を発行して waypoint へ移動
- 各 waypoint 到着ごとに rotation_angles を順に回し、MeasurementRequest を publish
- /g4/measure が立っていれば同内容でサービス呼び出し（非ブロッキング）

Parameters
----------
waypoints_yaml : str        "[[x,y],[x,y],...]" 形式（fallback）
waypoints_flat : double[]   [x1,y1,x2,y2,...] の一次元（優先）
rotation_angles : double[]  遮蔽体角度 [deg]
linear_speed : double
angular_speed : double
pos_tolerance : double
measure_delay : double

本実装では遮蔽体を 1/8 球殻としてモデル化し、
全方向をカバーするため Z 軸回り 0,90,180,270 度と Y 軸回り 0,180 度の
組合せ全8通りをデフォルトの rotation_angles に設定している。
rotation_angles の各値は deg 表記だが、360 度以上の値は
360 度を引いた角度で Z 軸回転し、引いた回数 1 回につき Y 軸回転 180 度を追加
することで裏表を切り替えるために利用する（DetectorConstruction.cc の実装参照）。
"""

import math
import json
from typing import List, Tuple, Optional

try:
    import yaml  # optional
    _HAS_YAML = True
except Exception:
    _HAS_YAML = False

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.duration import Duration

from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry

from g4_interfaces.msg import MeasurementRequest
from g4_interfaces.srv import Measurement as MeasurementSrv


def euler_yaw_from_quat(x, y, z, w) -> float:
    """Quaternion -> yaw [rad]."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _parse_waypoints_yaml(s: str) -> List[Tuple[float, float]]:
    if not s or not isinstance(s, str):
        return [(0.0, 0.0)]
    obj = None
    try:
        obj = json.loads(s)
    except Exception:
        if _HAS_YAML:
            try:
                obj = yaml.safe_load(s)
            except Exception:
                obj = None
    if not isinstance(obj, list):
        return [(0.0, 0.0)]
    out: List[Tuple[float, float]] = []
    for it in obj:
        if isinstance(it, (list, tuple)) and len(it) == 2:
            try:
                out.append((float(it[0]), float(it[1])))
            except Exception:
                pass
    return out if out else [(0.0, 0.0)]


def _parse_waypoints_flat(arr) -> List[Tuple[float, float]]:
    out: List[Tuple[float, float]] = []
    if not isinstance(arr, (list, tuple)) or len(arr) < 2 or (len(arr) % 2) != 0:
        return out
    try:
        for i in range(0, len(arr), 2):
            out.append((float(arr[i]), float(arr[i + 1])))
    except Exception:
        return []
    return out


class RobotMeasurementNode(Node):
    def __init__(self):
        super().__init__('robot_measurement_node')

        # ---- Parameters ----
        self.declare_parameter('waypoints_yaml', '')   # ← 空文字
        self.declare_parameter('waypoints_flat', [])   # ← 空リスト
        self.declare_parameter('rotation_angles', [])  # ← 空リスト

        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('angular_speed', 1.0)
        self.declare_parameter('pos_tolerance', 0.05)
        self.declare_parameter('measure_delay', 2.0)
        # ★ use_sim_time は Jazzy で既に宣言済みの場合があるため、ここでは宣言しない

        waypoints_yaml = self.get_parameter('waypoints_yaml').value
        waypoints_flat = self.get_parameter('waypoints_flat').value
        rotation_angles_deg = [float(a) for a in self.get_parameter('rotation_angles').value]

        self.waypoints: List[Tuple[float, float]] = _parse_waypoints_flat(waypoints_flat) or _parse_waypoints_yaml(waypoints_yaml)
        self.rot_deg: List[float] = rotation_angles_deg
        self.linear_speed: float = float(self.get_parameter('linear_speed').value)
        self.angular_speed: float = float(self.get_parameter('angular_speed').value)
        self.pos_tol: float = float(self.get_parameter('pos_tolerance').value)
        self.measure_delay: float = float(self.get_parameter('measure_delay').value)

        # ---- QoS ----
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.history = HistoryPolicy.KEEP_LAST

        # ---- Pubs/Subs ----
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', qos)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self._on_odom, qos)

        # MeasurementRequest を流す
        meas_qos = QoSProfile(depth=10)
        meas_qos.reliability = ReliabilityPolicy.RELIABLE
        meas_qos.history = HistoryPolicy.KEEP_LAST
        meas_qos.durability = DurabilityPolicy.VOLATILE
        self.pub_meas_req = self.create_publisher(MeasurementRequest, '/measurement_request', meas_qos)

        # /g4/measure サービス（あれば叩く）
        self.cli_measure = self.create_client(MeasurementSrv, '/g4/measure')

        # ---- State ----
        self.current_odom: Optional[Odometry] = None
        self.idx_wp = 0
        self.idx_rot = 0
        self.state = 'MOVE' if self.waypoints else 'DONE'
        self.last_measure_t = self.get_clock().now()

        self.get_logger().info(
            f'waypoints={self.waypoints}, rotation_angles(deg)={self.rot_deg}, '
            f'linear={self.linear_speed}, angular={self.angular_speed}, tol={self.pos_tol}'
        )

        self.timer = self.create_timer(0.05, self._loop)

    # ---------------- Callbacks / Loop ----------------
    def _on_odom(self, msg: Odometry) -> None:
        self.current_odom = msg

    def _loop(self) -> None:
        if self.state == 'DONE':
            self._stop()
            return
        if self.current_odom is None:
            return

        if self.state == 'MOVE':
            self._do_move()
        elif self.state == 'MEASURE':
            self._do_measure()

    # ---------------- Motion ----------------
    def _do_move(self) -> None:
        gx, gy = self.waypoints[self.idx_wp]
        x = self.current_odom.pose.pose.position.x
        y = self.current_odom.pose.pose.position.y

        dx = gx - x
        dy = gy - y
        dist = math.hypot(dx, dy)

        cmd = Twist()
        if dist > self.pos_tol:
            yaw = euler_yaw_from_quat(
                self.current_odom.pose.pose.orientation.x,
                self.current_odom.pose.pose.orientation.y,
                self.current_odom.pose.pose.orientation.z,
                self.current_odom.pose.pose.orientation.w,
            )
            desired = math.atan2(dy, dx)
            yaw_err = self._wrap(desired - yaw)
            cmd.linear.x = max(min(self.linear_speed, dist), 0.0)
            cmd.angular.z = max(min(self.angular_speed, yaw_err), -self.angular_speed)
            self.pub_cmd.publish(cmd)
        else:
            self._stop()
            self.idx_rot = 0
            self.state = 'MEASURE'
            self.last_measure_t = self.get_clock().now()
            self.get_logger().info(f'Arrived at waypoint {self.idx_wp}/{len(self.waypoints)-1}. Start MEASURE.')

    def _stop(self) -> None:
        self.pub_cmd.publish(Twist())

    # ---------------- Measurement ----------------
    def _do_measure(self) -> None:
        now = self.get_clock().now()
        if (now - self.last_measure_t) < Duration(seconds=self.measure_delay):
            return

        if self.idx_rot >= len(self.rot_deg):
            self.idx_wp += 1
            if self.idx_wp >= len(self.waypoints):
                self.get_logger().info('All waypoints completed. DONE.')
                self.state = 'DONE'
                return
            self.state = 'MOVE'
            self.get_logger().info(f'Move to next waypoint {self.idx_wp}')
            return

        # 角度[deg] → [rad]（STEP2の msg はラジアン）
        rot_deg = float(self.rot_deg[self.idx_rot])
        rot_rad = math.radians(rot_deg)

        # MeasurementRequest を publish
        req = MeasurementRequest()
        req.stamp = now.to_msg()

        # 検出器位置は現在位置（Z=0 基準）
        p = Point()
        p.x = self.current_odom.pose.pose.position.x
        p.y = self.current_odom.pose.pose.position.y
        p.z = 0.0
        req.detector_position = p

        req.shield_rotation_angle = rot_rad

        # 3D_estimation 互換の仮の線源 [x,y,z,q]
        req.source_parameters = [3.5, 3.5, 0.8, 100.0]

        self.pub_meas_req.publish(req)
        self.get_logger().info(
            f'Publish MeasurementRequest: pos=({p.x:.2f},{p.y:.2f}), rot={rot_deg:.1f} deg'
        )

        # サービスがあれば即時に呼ぶ（非ブロッキング）
        if self.cli_measure.service_is_ready():
            srv_req = MeasurementSrv.Request()
            srv_req.request = req
            future = self.cli_measure.call_async(srv_req)

            def _done(_):
                try:
                    res = future.result()
                    if res is not None:
                        self.get_logger().info(
                            f'/g4/measure -> counts={res.result.counts:.3f}, noise={res.result.noise:.3f}'
                        )
                except Exception as e:
                    self.get_logger().warn(f'/g4/measure call failed: {e!r}')

            future.add_done_callback(_done)

        self.idx_rot += 1
        self.last_measure_t = now

    # ---------------- Helpers ----------------
    @staticmethod
    def _wrap(a: float) -> float:
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a


def main(args=None):
    rclpy.init(args=args)
    node = RobotMeasurementNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()