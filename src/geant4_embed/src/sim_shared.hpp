#pragma once
#include <atomic>

// 未処理イベント数
extern std::atomic<long> g_pending_events;

// 測定値 b（イベント数カウント）
extern std::atomic<long> g_b;

// 直近の入射エネルギー [MeV]
extern std::atomic<double> g_energy_MeV;

// 直近の速度 [m/s]
extern std::atomic<double> g_last_speed_mps;

// 直近のyaw [rad]（発射方向に使用）
extern std::atomic<double> g_yaw_rad;

// 選択中の粒子（PDGコード）。-1 のときは geantino
extern std::atomic<int> g_particle_pdg;

// 直近イベントの検出器エネルギー付与量 [MeV]（ログ・可視化用）
extern std::atomic<double> g_last_edep_MeV;

// --- 追加パラメータ ---
// 線源位置 [m]
extern std::atomic<double> g_source_x;
extern std::atomic<double> g_source_y;
extern std::atomic<double> g_source_z;

// 線源強度（イベント数のスケーリングなどに使用）
extern std::atomic<double> g_source_intensity;

// ノイズの有無
extern std::atomic<bool> g_noise_enabled;

// ノイズ強度（Poisson 分布の平均を計算する際の係数）
extern std::atomic<double> g_noise_strength;

// 検出器の配置位置 [m]
extern std::atomic<double> g_detector_x;
extern std::atomic<double> g_detector_y;
extern std::atomic<double> g_detector_z;

// 遮蔽体の回転角度 [deg]
extern std::atomic<double> g_shield_angle_deg;
