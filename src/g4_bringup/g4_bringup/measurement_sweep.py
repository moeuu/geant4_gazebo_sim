#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
measurement_sweep.py

このスクリプトは、Geant4 の計測サービス `/g4/measure` を用いて
1/8 球殻遮蔽体の全8方向に対する計測をまとめて実行し、結果を CSV に保存します。

特徴:
  * 現在のロボット位置を `/odom` から取得し、自動的に測定器座標として使用します。
  * 回転角度は度単位で `rotation_angles_deg` パラメータに指定し、デフォルトでは
    [0, 90, 180, 270, 360, 450, 540, 630] の8通りを用いて裏表を切り替えます。
  * 各角度ごとに `/g4/measure` サービスを呼び出し、応答に含まれるカウント値
    (MeasurementResult.counts) を取得して CSV に記録します。
  * CSV にはロボットの x,y,z 位置とカウント数を記録します。

使用方法:

```
ros2 run g4_bringup measurement_sweep.py --ros-args \
  -p rotation_angles_deg:="[0,90,180,270,360,450,540,630]" \
  -p csv_path:="/home/ユーザー名/g4_results/measurement.csv" \
  -p source_parameters:="[3.5, 3.5, 0.8, 100.0]" \
  -p measure_timeout:=5.0
```

依存:
  * ROS2 rclpy
  * g4_interfaces パッケージの Measurement サービス
  * nav_msgs/msg/Odometry

"""

import csv
import math
import time
from typing import List, Optional

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from g4_interfaces.srv import Measurement as MeasurementSrv


class MeasurementSweep(Node):
    """ノード: 全方向の計測をまとめて実行し CSV に保存"""

    def __init__(self):
        super().__init__('measurement_sweep')

        # パラメータ宣言
        self.declare_parameter('rotation_angles_deg', [0.0, 90.0, 180.0, 270.0, 360.0, 450.0, 540.0, 630.0])
        self.declare_parameter('csv_path', 'measurement.csv')
        self.declare_parameter('source_parameters', [3.5, 3.5, 0.8, 100.0])
        self.declare_parameter('measure_timeout', 5.0)

        # パラメータ取得
        angles = self.get_parameter('rotation_angles_deg').value
        self.rotation_angles_deg: List[float] = [float(a) for a in angles]
        self.csv_path: str = str(self.get_parameter('csv_path').value)
        self.source_parameters: List[float] = [float(x) for x in self.get_parameter('source_parameters').value]
        self.measure_timeout: float = float(self.get_parameter('measure_timeout').value)

        # 最新の Odometry を保持
        self.current_odom: Optional[Odometry] = None

        # Odometry 購読
        qos_odom = rclpy.qos.QoSProfile(depth=10)
        self.create_subscription(Odometry, '/odom', self._on_odom, qos_odom)

        # サービスクライアント
        self.cli_measure = self.create_client(MeasurementSrv, '/g4/measure')

        self.get_logger().info(
            f'MeasurementSweep started: angles={self.rotation_angles_deg}, csv_path="{self.csv_path}", '
            f'source_parameters={self.source_parameters}, timeout={self.measure_timeout}s'
        )

    def _on_odom(self, msg: Odometry) -> None:
        """最新のオドメトリを保存"""
        self.current_odom = msg

    def run(self) -> None:
        """計測を実行して結果を CSV に保存"""
        # サービスの準備を待つ
        if not self.cli_measure.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('/g4/measure サービスが利用できません')
            return

        # オドメトリが得られるまで待つ
        t_start = time.time()
        while rclpy.ok() and self.current_odom is None:
            if time.time() - t_start > 10.0:
                self.get_logger().error('Odometry が取得できませんでした')
                return
            rclpy.spin_once(self, timeout_sec=0.1)

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
            rot_rad = math.radians(float(deg))

            # 現在位置を取得
            if self.current_odom is None:
                self.get_logger().warn('Odometry が未取得のため測定をスキップ')
                continue
            x = float(self.current_odom.pose.pose.position.x)
            y = float(self.current_odom.pose.pose.position.y)
            z = float(self.current_odom.pose.pose.position.z)

            # MeasurementRequest を構築
            request = MeasurementSrv.Request()
            # builtin_interfaces/Time はサービス側で補完されるので stamp は無視
            # 検出器位置
            p = Point()
            p.x = x
            p.y = y
            p.z = z
            request.request.detector_position = p
            # 遮蔽体角（ラジアン）
            request.request.shield_rotation_angle = float(rot_rad)
            # 線源パラメータ
            request.request.source_parameters = self.source_parameters

            # 非同期サービス呼び出し
            future = self.cli_measure.call_async(request)

            # 結果待ち
            end_time = time.time() + self.measure_timeout
            while rclpy.ok() and time.time() < end_time:
                rclpy.spin_once(self, timeout_sec=0.05)
                if future.done():
                    break

            if not future.done():
                self.get_logger().warn(f'角度 {deg:.1f}°: サービス応答がタイムアウトしました')
                continue

            # 結果取得
            try:
                res = future.result()
                if res is None:
                    self.get_logger().warn(f'角度 {deg:.1f}°: サービス応答がありません')
                    continue
                counts = float(res.result.counts)
                self.get_logger().info(
                    f'角度 {deg:.1f}°: pos=({x:.2f},{y:.2f},{z:.2f}) counts={counts:.3f}'
                )
                csv_writer.writerow([f'{x:.3f}', f'{y:.3f}', f'{z:.3f}', f'{counts:.6f}'])
                csv_file.flush()
            except Exception as e:
                self.get_logger().warn(f'角度 {deg:.1f}°: 応答処理中にエラー: {e!r}')
                continue

        csv_file.close()
        self.get_logger().info(f'計測が完了しました。結果は "{self.csv_path}" に保存されました。')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MeasurementSweep()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()