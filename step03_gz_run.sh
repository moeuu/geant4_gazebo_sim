#!/usr/bin/env bash
set -eo pipefail
echo "[*] Gazebo（ヘッドレス）で step03_odom_demo.sdf を起動します。"
gz sim -r -s -v 2 ./step03_odom_demo.sdf
