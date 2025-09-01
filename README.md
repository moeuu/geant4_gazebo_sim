# g4_ros_ws — Geant4 × ROS 2 可視化パイプライン

## 概要
- Geant4 のエネルギー付与（Edep）を ROS 2 トピックに流し、  
  **(1) プロッタ**, **(2) ヒスト**, **(3) 2D グリッド（OccupancyGrid）** を配信します。
- 再現性：`ros2 bag` 再生で同等結果を再生可能。

## 依存
- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo (gz) Harmonic
- Geant4 11.2.x
- Python: numpy, matplotlib（可視化ノードが使う場合）

> Docker/Devcontainer で固定環境も提供（`Dockerfile`, `.devcontainer/`）。

## セットアップ
```bash
# 初回
rosdep update
rosdep install --from-paths src --ignore-src -y -r

# ビルド
colcon build --symlink-install --merge-install
source install/setup.bash
