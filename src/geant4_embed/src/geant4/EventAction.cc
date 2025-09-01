#include "EventAction.hh"
#include "sim_shared.hpp"
#include <G4Event.hh>
#include <G4SDManager.hh>
#include <G4HCofThisEvent.hh>
#include <G4THitsMap.hh>
#include <G4SystemOfUnits.hh>
#include <iostream>

void EventAction::EndOfEventAction(const G4Event* event)
{
  // イベント数カウント（従来の b ）
  g_b.fetch_add(1, std::memory_order_relaxed);

  // Edep の合計を取得
  auto* hce = event->GetHCofThisEvent();
  double edepMeV = 0.0;
  if (hce) {
    auto* sdman = G4SDManager::GetSDMpointer();
    int cid = sdman->GetCollectionID("Det/Edep"); // MFD名/プリミティブ名
    if (cid >= 0) {
      auto* hitsMap = dynamic_cast<G4THitsMap<G4double>*>(hce->GetHC(cid));
      if (hitsMap) {
        const auto& m = *hitsMap->GetMap();
        for (const auto& kv : m) {
          if (kv.second) edepMeV += *(kv.second) / MeV;
        }
      }
    }
  }

  g_last_edep_MeV.store(edepMeV, std::memory_order_relaxed);

  // たまにだけ表示（うるさくならないように）
  static long print_every = 10;
  long b_now = g_b.load(std::memory_order_relaxed);
  if (b_now % print_every == 0) {
    std::cout << "[G4] Event #" << b_now
              << " : Edep=" << edepMeV << " MeV"
              << " (PDG=" << g_particle_pdg.load() 
              << ", E=" << g_energy_MeV.load() << " MeV)"
              << std::endl;
  }
}
