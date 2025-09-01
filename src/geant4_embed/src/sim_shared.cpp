#include "sim_shared.hpp"

// 共有変数の実体定義

std::atomic<long>   g_pending_events{0};
std::atomic<long>   g_b{0};
std::atomic<double> g_energy_MeV{1.0};
std::atomic<double> g_last_speed_mps{0.0};
std::atomic<double> g_yaw_rad{0.0};
std::atomic<int>    g_particle_pdg{-1};
std::atomic<double> g_last_edep_MeV{0.0};

// 追加パラメータ実体
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
