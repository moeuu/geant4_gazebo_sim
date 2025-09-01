#pragma once
#include <atomic>

// /odom 受信ごとに増える「未処理イベント数」
extern std::atomic<int>  g_pending_events;

// 測定値 b（ここではイベント数をそのままカウント）
extern std::atomic<long> g_b;

// 直近の入射エネルギー [MeV]（PrimaryGenerator が読む）
extern std::atomic<double> g_energy_MeV;

// 直近の速度 [m/s]（ログ用）
extern std::atomic<double> g_last_speed_mps;

// 直近のyaw [rad]（入射方向に使う）
extern std::atomic<double> g_yaw_rad;

// 選択中の粒子（PDGコード）。-1 のときは geantino を使う
extern std::atomic<int> g_particle_pdg;

// 直近イベントの検出器エネルギー付与量 [MeV]（ログ・可視化用）
extern std::atomic<double> g_last_edep_MeV;
