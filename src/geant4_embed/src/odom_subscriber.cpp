#include "odom_subscriber.hpp"
#include "sim_shared.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

static int map_particle_to_pdg(const std::string& in)
{
  std::string s = in;
  std::transform(s.begin(), s.end(), s.begin(), ::tolower);
  bool numeric = !s.empty() && (std::isdigit(s[0]) || s[0] == '-');
  if (numeric) {
    try { return std::stoi(s); } catch (...) {}
  }
  if (s == "geantino") return -1;
  if (s == "e-" || s == "electron") return 11;
  if (s == "e+" || s == "positron") return -11;
  if (s == "gamma" || s == "photon") return 22;
  if (s == "proton" || s == "p") return 2212;
  if (s == "neutron" || s == "n") return 2112;
  if (s == "mu-" || s == "muon-") return 13;
  if (s == "mu+" || s == "muon+") return -13;
  if (s == "pi+" || s == "piplus") return 211;
  if (s == "pi-" || s == "piminus") return -211;
  return -1;
}

OdomSubscriber::OdomSubscriber()
: rclcpp::Node("g4_odom_subscriber"),
  last_b_seen_(0),
  stats_count_(0),
  stats_mean_(0.0),
  stats_M2_(0.0)
{
  // エネルギー変換パラメータ
  offset_mev_        = this->declare_parameter<double>("energy.offset_mev",        0.5);
  scale_mev_per_mps_ = this->declare_parameter<double>("energy.scale_mev_per_mps", 2.0);
  max_mev_           = this->declare_parameter<double>("energy.max_mev",          10.0);

  const auto particle = this->declare_parameter<std::string>("beam.particle", "geantino");

  // 粒子種を共有へ反映
  const int pdg = map_particle_to_pdg(particle);
  g_particle_pdg.store(pdg, std::memory_order_relaxed);

  RCLCPP_INFO(this->get_logger(),
              "Energy mapping: E[MeV] = %.3f + %.3f * v[m/s], max=%.3f ; particle='%s'(PDG=%d)",
              offset_mev_, scale_mev_per_mps_, max_mev_, particle.c_str(), pdg);

  // 線源パラメータ読み込み
  auto src_pos = this->declare_parameter<std::vector<double>>("source.position", {0.0, 0.0, 0.0});
  if (src_pos.size() >= 3) {
    g_source_x.store(src_pos[0], std::memory_order_relaxed);
    g_source_y.store(src_pos[1], std::memory_order_relaxed);
    g_source_z.store(src_pos[2], std::memory_order_relaxed);
  }
  double src_intensity = this->declare_parameter<double>("source.intensity", 1.0);
  g_source_intensity.store(src_intensity, std::memory_order_relaxed);

  // ノイズ設定読み込み
  bool noise_enabled = this->declare_parameter<bool>("noise.enabled", false);
  g_noise_enabled.store(noise_enabled, std::memory_order_relaxed);
  double noise_strength = this->declare_parameter<double>("noise.strength", 0.0);
  g_noise_strength.store(noise_strength, std::memory_order_relaxed);

  // 検出器位置および遮蔽体角度（測定時に使用）
  auto det_pos = this->declare_parameter<std::vector<double>>("detector.position", {0.0, 0.0, 0.0});
  if (det_pos.size() >= 3) {
    g_detector_x.store(det_pos[0], std::memory_order_relaxed);
    g_detector_y.store(det_pos[1], std::memory_order_relaxed);
    g_detector_z.store(det_pos[2], std::memory_order_relaxed);
  }
  double shield_angle = this->declare_parameter<double>("shield.angle_deg", 0.0);
  g_shield_angle_deg.store(shield_angle, std::memory_order_relaxed);

  // /odom 購読
  rclcpp::QoS qos(rclcpp::KeepLast(10));
  qos.reliable();
  sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", qos,
    std::bind(&OdomSubscriber::callback, this, std::placeholders::_1));

  // 出力パブリッシャ
  edep_pub_ = this->create_publisher<std_msgs::msg::Float64>("/g4/edep", rclcpp::QoS(10).reliable());
  edep_stats_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/g4/edep_stats", rclcpp::QoS(10).reliable());

  // タイマ（Edep 配信＋統計更新）：既定 10 ms
  const int period_ms = this->declare_parameter<int>("edep.publisher_period_ms", 10);
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(period_ms),
    std::bind(&OdomSubscriber::on_timer, this));
}

static double yaw_from_quaternion(double x, double y, double z, double w)
{
  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double OdomSubscriber::map_speed_to_energy(double v_mps) const
{
  if (!std::isfinite(v_mps) || v_mps < 0.0) v_mps = 0.0;
  double e = offset_mev_ + scale_mev_per_mps_ * v_mps;
  if (!std::isfinite(e)) e = offset_mev_;
  if (e < 0.0) e = 0.0;
  if (e > max_mev_) e = max_mev_;
  return e;
}

void OdomSubscriber::callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // 速度 [m/s]
  const auto & v = msg->twist.twist.linear;
  const double speed = std::sqrt(v.x*v.x + v.y*v.y + v.z*v.z);

  // yaw [rad]
  const auto & q = msg->pose.pose.orientation;
  const double yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w);

  // 速度→エネルギー
  const double e_mev = map_speed_to_energy(speed);

  // 共有変数更新
  g_last_speed_mps.store(speed, std::memory_order_relaxed);
  g_energy_MeV.store(e_mev, std::memory_order_relaxed);
  g_yaw_rad.store(std::isfinite(yaw) ? yaw : 0.0, std::memory_order_relaxed);

  // Geant4 へ1イベント要求
  g_pending_events.fetch_add(1, std::memory_order_relaxed);

  // ログは N 回に 1 回
  static std::atomic<int> cnt{0};
  constexpr int kLogEvery = 10;
  int n = ++cnt;
  if (n % kLogEvery == 0) {
    std::cout << "[/odom] speed=" << speed << " [m/s], yaw=" << yaw
              << " rad -> E=" << e_mev << " [MeV] (every " << kLogEvery << ")"
              << std::endl;
  }
}

void OdomSubscriber::on_timer()
{
  // Geant4 側で進んだイベント数取得
  long b_cur = g_b.load(std::memory_order_relaxed);
  if (b_cur <= last_b_seen_) return;

  // 最新 Edep 取得
  const double x = g_last_edep_MeV.load(std::memory_order_relaxed);

  // Welford 法による統計更新
  long new_events = b_cur - last_b_seen_;
  for (long i = 0; i < new_events; ++i) {
    ++stats_count_;
    const double delta  = x - stats_mean_;
    stats_mean_        += delta / static_cast<double>(stats_count_);
    const double delta2 = x - stats_mean_;
    stats_M2_          += delta * delta2;
  }
  last_b_seen_ = b_cur;

  // /g4/edep 出力
  std_msgs::msg::Float64 edep_msg;
  edep_msg.data = x; // [MeV]
  edep_pub_->publish(edep_msg);

  // /g4/edep_stats 出力 -> [count, mean, variance]（母分散）
  std_msgs::msg::Float64MultiArray stats_msg;
  stats_msg.data.resize(3);
  stats_msg.data[0] = static_cast<double>(stats_count_);
  stats_msg.data[1] = stats_mean_;
  stats_msg.data[2] = (stats_count_ > 0) ? (stats_M2_ / static_cast<double>(stats_count_)) : 0.0;
  edep_stats_pub_->publish(stats_msg);
}
