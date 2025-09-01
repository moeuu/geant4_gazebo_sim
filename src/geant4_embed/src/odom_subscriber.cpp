#include "odom_subscriber.hpp"
#include "sim_shared.hpp"

#include <cmath>
#include <iostream>
#include <atomic>
#include <string>
#include <algorithm>

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
  // パラメータ宣言
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

  // /odom購読
  rclcpp::QoS qos(rclcpp::KeepLast(10));
  qos.reliable();
  sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", qos,
    std::bind(&OdomSubscriber::callback, this, std::placeholders::_1));

  // 出力パブリッシャ
  edep_pub_ = this->create_publisher<std_msgs::msg::Float64>("/g4/edep", rclcpp::QoS(10).reliable());
  edep_stats_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/g4/edep_stats", rclcpp::QoS(10).reliable());

  // タイマ（Edep配信＋統計更新）：既定 10 ms
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
  // 速度[m/s]
  const auto & v = msg->twist.twist.linear;
  const double speed = std::sqrt(v.x*v.x + v.y*v.y + v.z*v.z);

  // yaw[rad]
  const auto & q = msg->pose.pose.orientation;
  const double yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w);

  // 速度→エネルギー
  const double e_mev = map_speed_to_energy(speed);

  // 共有変数を更新
  g_last_speed_mps.store(speed, std::memory_order_relaxed);
  g_energy_MeV.store(e_mev, std::memory_order_relaxed);
  g_yaw_rad.store(std::isfinite(yaw) ? yaw : 0.0, std::memory_order_relaxed);

  // Geant4 へ1イベント要求（main側で BeamOn 実行）
  g_pending_events.fetch_add(1, std::memory_order_relaxed);

  // ログはN回に1回
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
  // Geant4 側で進んだイベント数を取得
  long b_cur = g_b.load(std::memory_order_relaxed);
  if (b_cur <= last_b_seen_) return;

  // 最新 Edep を取得
  const double x = g_last_edep_MeV.load(std::memory_order_relaxed);

  // ---- Welford 更新（b_cur - last_b_seen_ 分だけ進めたいが、
  //      BeamOn を 1 にしているため通常は 1 件差）----
  long new_events = b_cur - last_b_seen_;
  for (long i = 0; i < new_events; ++i) {
    ++stats_count_;
    const double delta  = x - stats_mean_;
    stats_mean_        += delta / static_cast<double>(stats_count_);
    const double delta2 = x - stats_mean_;
    stats_M2_          += delta * delta2;
  }
  last_b_seen_ = b_cur;

  // 出力: /g4/edep
  std_msgs::msg::Float64 edep_msg;
  edep_msg.data = x; // [MeV]
  edep_pub_->publish(edep_msg);

  // 出力: /g4/edep_stats -> [count, mean, variance] （母分散）
  std_msgs::msg::Float64MultiArray stats_msg;
  stats_msg.data.resize(3);
  stats_msg.data[0] = static_cast<double>(stats_count_);
  stats_msg.data[1] = stats_mean_;
  stats_msg.data[2] = (stats_count_ > 0) ? (stats_M2_ / static_cast<double>(stats_count_)) : 0.0;
  edep_stats_pub_->publish(stats_msg);
}
