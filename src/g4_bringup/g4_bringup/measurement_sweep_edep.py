#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
measurement_sweep_edep.py

このスクリプトは `/g4/edep` トピックを購読して 1/8 球殻遮蔽体の全8方向に
対する放射線計測をまとめて実行し、結果を CSV に保存します。従来の
`measurement_sweep.py` は Geant4 側が提供する `/g4/measure` サービスを
使っていましたが、diffbot_main での計測ではサービスが用意されていない
ため、代わりにエネルギー付与トピック `/g4/edep` を直接購読して
イベント数（カウント数）を求めます。

特徴:
  * 現在のロボット位置を `/odom` から取得し、自動的に測定器座標として使用します。
  * 回転角度は度単位で `rotation_angles_deg` パラメータに指定し、デフォルトでは
    [0, 90, 180, 270, 360, 450, 540, 630] の8通りを用いて裏表を切り替えます。
  * 各角度ごとに Geant4 ノードの `shield.angle_deg` パラメータを更新し、
    一定時間 (`measure_timeout` 秒) `/g4/edep` を購読して発生回数をカウントします。
  * CSV にはロボットの x,y,z 位置とカウント数を記録します。

使用方法:

```
ros2 run g4_bringup measurement_sweep_edep --ros-args \
  -p rotation_angles_deg:="[0,90,180,270,360,450,540,630]" \
  -p csv_path:="/result/measurement.csv" \
  -p source_parameters:="[3.5, 2.5, 10.0, 50.0]" \
  -p measure_timeout:=5.0
```

依存:
  * ROS2 rclpy
  * nav_msgs/msg/Odometry
  * g4_viz パッケージの edep_common.py (edep_msg_types, extract_edep_value)
  * rcl_interfaces/srv/SetParameters （パラメータ更新用）

"""

import csv
import math
import time
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import SetParameters

try:
    # edep_common は g4_viz パッケージ内にあり、エネルギーメッセージの型判定と抽出を行います
    from g4_viz.edep_common import edep_msg_types, extract_edep_value
except Exception:
    # パッケージ名が変わった場合はローカルディレクトリから読み込む
    from edep_common import edep_msg_types, extract_edep_value  # type: ignore


class MeasurementSweepEdep(Node):
    """/g4/edep を用いた全方向測定ノード"""

    def __init__(self) -> None:
        super().__init__('measurement_sweep_edep')

        # パラメータ宣言
        self.declare_parameter('rotation_angles_deg', [0.0, 90.0, 180.0, 270.0, 360.0, 450.0, 540.0, 630.0])
        self.declare_parameter('csv_path', 'measurement.csv')
        # `source_parameters` はサービス不使用時には未使用だが互換のため保持
        self.declare_parameter('source_parameters', [3.5, 3.5, 0.8, 100.0])
        self.declare_parameter('measure_timeout', 5.0)
        self.declare_parameter('g4_node_name', 'g4_odom_subscriber')
        self.declare_parameter('shield_param_name', 'shield.angle_deg')

        # パラメータ取得
        angles = self.get_parameter('rotation_angles_deg').value
        self.rotation_angles_deg: List[float] = [float(a) for a in angles]
        self.csv_path: str = str(self.get_parameter('csv_path').value)
        # source_parameters は現状未使用
        self.source_parameters: List[float] = [float(x) for x in self.get_parameter('source_parameters').value]
        self.measure_timeout: float = float(self.get_parameter('measure_timeout').value)
        self.g4_node_name: str = str(self.get_parameter('g4_node_name').value)
        self.shield_param_name: str = str(self.get_parameter('shield_param_name').value)

        # 最新の Odometry を保持
        self.current_odom: Optional[Odometry] = None

        # カウント用フラグとカウント値
        self._collecting: bool = False
        self._count: int = 0

        # Odometry 購読
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)

        # /g4/edep 購読（型が複数存在し得るため二重購読）
        self._edep_subs = []
        for msg_type in edep_msg_types():
            self._edep_subs.append(
                self.create_subscription(msg_type, '/g4/edep', self._on_edep, 10)
            )

        # パラメータ設定用クライアント
        # Note: SetParameters は対象ノード名に依存したサービス名を持つ
        # 例: node_name = g4_odom_subscriber -> '/g4_odom_subscriber/set_parameters'
        srv_name = f'/{self.g4_node_name}/set_parameters'
        self.param_client = self.create_client(SetParameters, srv_name)

        self.get_logger().info(
            f'MeasurementSweepEdep started: angles={self.rotation_angles_deg}, csv_path="{self.csv_path}", '
            f'timeout={self.measure_timeout}s, g4_node={self.g4_node_name}, shield_param={self.shield_param_name}'
        )

    def _on_odom(self, msg: Odometry) -> None:
        """最新のオドメトリを保存"""
        self.current_odom = msg

    def _on_edep(self, msg) -> None:
        """/g4/edep のコールバック: 測定中のみカウントを増加"""
        if not self._collecting:
            return
        # 任意の値を抽出し、存在すれば1カウントとして数える
        val = extract_edep_value(msg)
        # edep が None の場合はカウントしない
        if val is not None:
            # 0 でもカウントする（入射が無くても1回計測したイベントとみなす）
            self._count += 1

    def _wait_for_param_service(self, timeout_sec: float = 10.0) -> bool:
        """SetParameters サービスが利用可能になるまで待つ"""
        return self.param_client.wait_for_service(timeout_sec=timeout_sec)

    def _set_shield_angle(self, deg: float) -> bool:
        """指定した角度でシールドパラメータを更新する。成功時 True"""
        # Parameter オブジェクトを作成
        param = Parameter(self.shield_param_name, Parameter.Type.DOUBLE, deg)
        req = SetParameters.Request()
        req.parameters.append(param.to_parameter_msg())
        future = self.param_client.call_async(req)
        # タイムアウトまで待機
        end_time = time.time() + 5.0
        while rclpy.ok() and time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.05)
            if future.done():
                break
        if not future.done():
            self.get_logger().warn(f'set_parameters({deg:.1f}deg) タイムアウト')
            return False
        try:
            res = future.result()
            # res.results は list[rcl_interfaces.msg.SetParametersResult]
            return all(r.successful for r in res.results)
        except Exception as e:
            self.get_logger().error(f'set_parameters({deg:.1f}deg) 応答エラー: {e!r}')
            return False

    def run(self) -> None:
        """計測を実行して結果を CSV に保存"""
        # オドメトリが得られるまで待つ
        t_start = time.time()
        while rclpy.ok() and self.current_odom is None:
            if time.time() - t_start > 10.0:
                self.get_logger().error('Odometry が取得できませんでした')
                return
            rclpy.spin_once(self, timeout_sec=0.1)

        # パラメータサービスが利用可能か確認
        if not self._wait_for_param_service(timeout_sec=10.0):
            self.get_logger().error(f'{self.param_client.srv_name} サービスが利用できません')
            return

        # CSV ファイルを開く
        try:
            csv_file = open(self.csv_path, 'w', newline='')
        except Exception as e:
            self.get_logger().error(f'CSV ファイルを開けません: {e!r}')
            return
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(['x', 'y', 'z', 'counts'])

        # 角度ごとに計測
        for deg in self.rotation_angles_deg:
            # シールド角度設定
            if not self._set_shield_angle(deg):
                self.get_logger().warn(f'角度 {deg:.1f}°: パラメータ更新に失敗しました')
                continue
            # 少し待ってから計測を開始
            time.sleep(0.2)

            # 現在位置を取得
            if self.current_odom is None:
                self.get_logger().warn('Odometry が未取得のため測定をスキップ')
                continue
            x = float(self.current_odom.pose.pose.position.x)
            y = float(self.current_odom.pose.pose.position.y)
            z = float(self.current_odom.pose.pose.position.z)

            # カウント開始
            self._count = 0
            self._collecting = True
            end_time = time.time() + self.measure_timeout
            while rclpy.ok() and time.time() < end_time:
                rclpy.spin_once(self, timeout_sec=0.05)
            self._collecting = False

            counts = float(self._count)
            self.get_logger().info(
                f'角度 {deg:.1f}°: pos=({x:.2f},{y:.2f},{z:.2f}) counts={counts:.0f}'
            )
            csv_writer.writerow([f'{x:.3f}', f'{y:.3f}', f'{z:.3f}', f'{counts:.0f}'])
            csv_file.flush()

        csv_file.close()
        self.get_logger().info(f'計測が完了しました。結果は "{self.csv_path}" に保存されました。')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MeasurementSweepEdep()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()