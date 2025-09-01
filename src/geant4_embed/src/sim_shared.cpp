#include "sim_shared.hpp"

std::atomic<int>    g_pending_events{0};
std::atomic<long>   g_b{0};
std::atomic<double> g_energy_MeV{1.0};
std::atomic<double> g_last_speed_mps{0.0};
std::atomic<double> g_yaw_rad{0.0};
std::atomic<int>    g_particle_pdg{-1};     // -1 => geantino
std::atomic<double> g_last_edep_MeV{0.0};
