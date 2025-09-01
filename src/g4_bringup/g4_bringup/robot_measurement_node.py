#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
robot_measurement_node.py

Gazebo上の移動ロボットを計測点へ移動させ、各点で遮蔽体角度を順に変えつつ
Geant4側へ測定リクエスト（位置x,y,角度deg）を投げるノード。

- /odom を購読して現在位置を取得
- /cmd_vel を発行して移動
- /measurement_request (Float32MultiArray) に [x, y, angle_deg] を出力

Parameters
----------
waypoints_yaml : str
    "[[x,y],[x,y],...]" 形式の文字列。waypoints_flat が有効でない場合に使用。
waypoints_flat : double[]
    [x1,y1,x2,y2,...] の一次元配列。偶数長かつ2以上なら優先採用。
rotation_angles : double[]
    遮蔽体角度[deg] の配列。
linear_speed : double
angular_speed : double
pos_tolerance : double
measure_delay : double
"""

import math
import time
from typing import List, Tuple

import json
try:
    import yaml  # optional
    _HAS_YAML = True
except Exception:
    _HAS_YAML = False

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray


def euler_from_quaternion(x, y, z, w) -> float:
    """Convert quaternion (x, y, z, w) to yaw [rad]."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _parse_waypoints_yaml(s: str) -> List[Tuple[float, float]]:
    """Parse waypoints from a YAML/JSON-like string: [[x,y],[x,y],...]."""
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
    """Parse waypoints from a flat list [x1,y1,x2,y2,...]."""
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
    """Node controlling robot movement and measurement triggering."""

    def __init__(self):
        super().__init__('robot_measurement_node')
        # ParameterDescriptorは使わず、デフォルトで型を確定させる
        self.declare_parameter('waypoints_yaml', '[[0.0, 0.0]]')
        # 空配列だとBYTE_ARRAYに推定されるため、double配列っぽい初期値にする
        self.declare_parameter('waypoints_flat', [0.0, 0.0])
        self.declare_parameter('rotation_angles', [0.0, 90.0, 180.0, 270.0])
        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('angular_speed', 1.0)
        self.declare_parameter('pos_tolerance', 0.05)
        self.declare_parameter('measure_delay', 2.0)

        # Get parameters
        waypoints_yaml = self.get_parameter('waypoints_yaml').value
        waypoints_flat = self.get_parameter('waypoints_flat').value

        # Prefer flat if valid, else parse yaml
        wps = _parse_waypoints_flat(waypoints_flat)
        if not wps:
            wps = _parse_waypoints_yaml(waypoints_yaml)

        self.waypoints: List[Tuple[float, float]] = wps
        self.rotation_angles: List[float] = [float(a) for a in self.get_parameter('rotation_angles').value]
        self.linear_speed: float = float(self.get_parameter('linear_speed').value)
        self.angular_speed: float = float(self.get_parameter('angular_speed').value)
        self.pos_tolerance: float = float(self.get_parameter('pos_tolerance').value)
        self.measure_delay: float = float(self.get_parameter('measure_delay').value)

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.measure_pub = self.create_publisher(Float32MultiArray, '/measurement_request', qos_profile)

        # Subscriber
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Internal state
        self.current_pose = None  # type: Odometry | None
        self.waypoint_index = 0
        self.rotation_index = 0
        self.state = 'move'  # 'move', 'measure', 'wait'
        self.last_measurement_time = 0.0

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info(f'RobotMeasurementNode initialized. waypoints={self.waypoints}, angles={self.rotation_angles}')

    def odom_callback(self, msg: Odometry):
        self.current_pose = msg

    def publish_twist(self, linear: float, angular: float):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_pub.publish(twist)

    def publish_measurement_request(self, x: float, y: float, angle_deg: float):
        msg = Float32MultiArray()
        msg.data = [float(x), float(y), float(angle_deg)]
        self.measure_pub.publish(msg)
        self.get_logger().info(f'MEAS REQ: x={x:.2f}, y={y:.2f}, angle={angle_deg:.1f} deg')

    def compute_control(self, goal_x: float, goal_y: float):
        if self.current_pose is None:
            return 0.0, 0.0
        p = self.current_pose.pose.pose.position
        x, y = p.x, p.y
        q = self.current_pose.pose.pose.orientation
        yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)

        dx, dy = goal_x - x, goal_y - y
        angle_to_goal = math.atan2(dy, dx)
        angle_diff = self._normalize_angle(angle_to_goal - yaw)

        linear_vel = self.linear_speed if abs(angle_diff) < 0.2 else 0.0
        angular_vel = max(min(self.angular_speed * angle_diff, self.angular_speed), -self.angular_speed)
        return linear_vel, angular_vel

    def _normalize_angle(self, angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def has_reached_waypoint(self, goal_x: float, goal_y: float) -> bool:
        if self.current_pose is None:
            return False
        p = self.current_pose.pose.pose.position
        return math.hypot(goal_x - p.x, goal_y - p.y) < self.pos_tolerance

    def timer_callback(self):
        if not self.waypoints or self.waypoint_index >= len(self.waypoints):
            self.publish_twist(0.0, 0.0)
            return

        goal_x, goal_y = self.waypoints[self.waypoint_index]

        if self.state == 'move':
            if self.has_reached_waypoint(goal_x, goal_y):
                self.publish_twist(0.0, 0.0)
                self.rotation_index = 0
                self.state = 'measure'
                self.get_logger().info(f'Reached waypoint {self.waypoint_index}: ({goal_x:.2f}, {goal_y:.2f})')
            else:
                lin, ang = self.compute_control(goal_x, goal_y)
                self.publish_twist(lin, ang)

        elif self.state == 'measure':
            angle_deg = self.rotation_angles[self.rotation_index]
            if self.current_pose is not None:
                p = self.current_pose.pose.pose.position
                self.publish_measurement_request(p.x, p.y, angle_deg)
            else:
                self.publish_measurement_request(goal_x, goal_y, angle_deg)
            self.last_measurement_time = time.time()
            self.state = 'wait'

        elif self.state == 'wait':
            if (time.time() - self.last_measurement_time) >= self.measure_delay:
                self.rotation_index += 1
                if self.rotation_index < len(self.rotation_angles):
                    self.state = 'measure'
                else:
                    self.waypoint_index += 1
                    self.state = 'move'


def main(args=None):
    rclpy.init(args=args)
    node = RobotMeasurementNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_twist(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
