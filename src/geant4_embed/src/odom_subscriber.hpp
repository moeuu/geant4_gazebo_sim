#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <vector>

// /odom トピック購読ノード
class OdomSubscriber : public rclcpp::Node
{
public:
  OdomSubscriber();

private:
  void callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  double map_speed_to_energy(double v_mps) const;
  void on_timer();  // Edep 配信と統計更新

  // エネルギー変換パラメータ（E[MeV] = offset + scale * v を max でクリップ）
  double offset_mev_;
  double scale_mev_per_mps_;
  double max_mev_;

  // /odom 購読
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;

  // Edep パブリッシャ
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr edep_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr edep_stats_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // 統計（Welford 法）
  long   last_b_seen_;
  long   stats_count_;
  double stats_mean_;
  double stats_M2_;
};
