#include "sim_shared_multi.hpp"  // ★ ここを sim_shared.hpp から置き換え
// 共有変数の実体定義

std::atomic<long>   g_pending_events{0};
std::atomic<long>   g_b{0};
std::atomic<double> g_energy_MeV{1.0};
std::atomic<double> g_last_speed_mps{0.0};
std::atomic<double> g_yaw_rad{0.0};
std::atomic<int>    g_particle_pdg{-1};
std::atomic<double> g_last_edep_MeV{0.0};

// 追加パラメータ実体（単一線源互換）
std::atomic<double> g_source_x{0.0};
std::atomic<double> g_source_y{0.0};
std::atomic<double> g_source_z{0.0};
std::atomic<double> g_source_intensity{1.0};
std::atomic<bool>   g_noise_enabled{false};
std::atomic<double> g_noise_strength{0.0};
std::atomic<double> g_detector_x{0.0};
std::atomic<double> g_detector_y{0.0};
std::atomic<double> g_detector_z{0.0};
std::atomic<double> g_shield_angle_deg{0.0};

// 複数線源サポート（ここが今回リンクエラーの実体定義）
// g_sources: 各要素に [x,y,z] [m] の座標
// g_source_intensities: 各線源の強度（空なら g_source_intensity を全線源に適用）
std::vector<std::array<double, 3>> g_sources{};
std::vector<double>                g_source_intensities{};
