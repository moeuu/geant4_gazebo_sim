#include "EventAction.hh"
#include "sim_shared.hpp"

#include <G4Event.hh>
#include <G4HCofThisEvent.hh>
#include <G4SDManager.hh>
#include <G4THitsMap.hh>
#include <G4VHitsCollection.hh>
#include <G4SystemOfUnits.hh>
#include <random>
#include <iostream>

void EventAction::EndOfEventAction(const G4Event* event)
{
  // イベント数カウント（b をインクリメント）
  g_b.fetch_add(1, std::memory_order_relaxed);

  // Edep の合計を取得 [MeV]
  auto* hce = event->GetHCofThisEvent();
  double edepMeV = 0.0;
  if (hce) {
    auto* sdman = G4SDManager::GetSDMpointer();
    int cid = sdman->GetCollectionID("Det/Edep"); // MFD名/プリミティブ名
    if (cid >= 0) {
      auto* hitsMap = dynamic_cast<G4THitsMap<double>*>(hce->GetHC(cid));
      if (hitsMap) {
        const auto& m = *hitsMap->GetMap();
        for (const auto& kv : m) {
          if (kv.second) edepMeV += *(kv.second) / MeV;
        }
      }
    }
  }

  // Poisson ノイズの追加（必要に応じて）
  bool noiseEnabled  = g_noise_enabled.load(std::memory_order_relaxed);
  double noiseFactor = g_noise_strength.load(std::memory_order_relaxed);
  double measuredEdep = edepMeV;
  if (noiseEnabled && edepMeV > 0.0 && noiseFactor > 0.0) {
    // Poisson 分布の平均: edep * noiseFactor
    static thread_local std::mt19937 rng(std::random_device{}());
    std::poisson_distribution<> dist(edepMeV * noiseFactor);
    double noise = static_cast<double>(dist(rng));
    measuredEdep = edepMeV + noise;
  }

  // 共有変数へ格納
  g_last_edep_MeV.store(measuredEdep, std::memory_order_relaxed);

  // たまに表示（うるさくならないように）
  static const long print_every = 10;
  long b_now = g_b.load(std::memory_order_relaxed);
  if (b_now % print_every == 0) {
    std::cout << "[G4] Event #" << b_now
              << " : Edep=" << measuredEdep << " MeV"
              << " (PDG=" << g_particle_pdg.load()
              << ", E=" << g_energy_MeV.load() << " MeV)"
              << std::endl;
  }
}
