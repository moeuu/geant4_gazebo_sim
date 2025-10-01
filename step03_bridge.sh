#!/usr/bin/env bash
# 最小で確実に動く版：setup.bash を source する前後で set -u を無効化
set -eo pipefail

echo "[*] ros_gz_bridge を2本起動します（/cmd_vel と /odom を確実にリマップ）"

# --- ROS 2 Jazzy 環境読み込み（未定義変数で落ちないよう一時的に -u を外す）---
set +u
# 未定義でも安全にするためデフォルト値をセット（保険）
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-0}"
source /opt/ros/jazzy/setup.bash
set -u
# -------------------------------------------------------------------------------

# 1) /cmd_vel: ROS(geometry_msgs/Twist) <-> GZ(gz.msgs.Twist)
#    ROSの /cmd_vel を GZ の /model/turtlebot3_burger/cmd_vel にリマップ
ros2 run ros_gz_bridge parameter_bridge \
  /model/turtlebot3_burger/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist \
  --ros-args -r /model/turtlebot3_burger/cmd_vel:=/cmd_vel &
BRIDGE1=$!
echo "[*] BRIDGE1 (cmd_vel) PID=$BRIDGE1"

# 2) /odom: GZ(gz.msgs.Odometry) <-> ROS(nav_msgs/Odometry)
#    GZの /model/turtlebot3_burger/odom を ROS の /odom にリマップ
ros2 run ros_gz_bridge parameter_bridge \
  /model/turtlebot3_burger/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry \
  --ros-args -r /model/turtlebot3_burger/odom:=/odom &
BRIDGE2=$!
echo "[*] BRIDGE2 (odom->odom) PID=$BRIDGE2"

cleanup() {
  echo; echo "[*] 停止処理: kill $BRIDGE1 $BRIDGE2"
  kill $BRIDGE1 $BRIDGE2 2>/dev/null || true
  wait $BRIDGE1 $BRIDGE2 2>/dev/null || true
}
trap cleanup INT TERM

wait
