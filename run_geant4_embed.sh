#!/usr/bin/env bash
set -eo pipefail

echo "[*] Launching geant4_embed_node (env auto-setup)"

# うるさいトレース環境変数は未設定に
unset COLCON_TRACE || true
unset AMENT_TRACE_SETUP_FILES || true

# nounset 環境でも安全に source する関数
safe_source() {
  local __ns=0
  if set -o | grep -q 'nounset *on'; then __ns=1; fi
  set +u
  [ -f "$1" ] && . "$1"
  if [ "$__ns" -eq 1 ]; then set -u; fi
  unset __ns
}

# Geant4 / 自WSを確実に優先
export CMAKE_PREFIX_PATH="/opt/geant4-11.2.1:${CMAKE_PREFIX_PATH:-}"
export AMENT_PREFIX_PATH="$HOME/g4_ros_ws/install:${AMENT_PREFIX_PATH:-}"

# 読み込み順：ROS2 → Geant4 → 自WS（local_setup推奨）
safe_source /opt/ros/jazzy/setup.bash
safe_source /opt/geant4-11.2.1/bin/geant4.sh
safe_source "$HOME/g4_ros_ws/install/local_setup.bash"

# 見えるか軽くチェック
if ! ros2 pkg prefix geant4_embed >/dev/null 2>&1; then
  echo "[!] geant4_embed が見えていません。ビルド or 環境を確認してください。" >&2
  echo "    ヒント: cd ~/g4_ros_ws && colcon build --merge-install" >&2
  exit 1
fi

# 起動！
exec ros2 run geant4_embed geant4_embed_node
