//
//  EventAction.cc — 非指向性検出器用イベント処理
//
//  各イベント終了時に、多機能検出器が記録したフラックス（トラック数）
//  を取得し、共有変数に格納する。エネルギー付与ではなく、CellFlux
//  プリミティブによって蓄積された値を用いる。計数結果は二次的に
//  Poisson 雑音を付加してシミュレーションのばらつきを再現する。

#include "EventAction.hh"
#include "sim_shared.hpp"

#include <G4Event.hh>
#include <G4HCofThisEvent.hh>
#include <G4SDManager.hh>
#include <G4THitsMap.hh>
#include <G4VHitsCollection.hh>
#include <random>
#include <iostream>

void EventAction::EndOfEventAction(const G4Event* event)
{
  // イベント数カウント（b をインクリメント）
  g_b.fetch_add(1, std::memory_order_relaxed);

  // Flux（トラック数）の合計を取得
  auto* hce = event->GetHCofThisEvent();
  double fluxCount = 0.0;
  if (hce) {
    auto* sdman = G4SDManager::GetSDMpointer();
    // 「Det/Flux」という名前で登録したスコアリングプリミティブを取得
    int cid = sdman->GetCollectionID("Det/Flux");
    if (cid >= 0) {
      auto* hitsMap = dynamic_cast<G4THitsMap<double>*>(hce->GetHC(cid));
      if (hitsMap) {
        const auto& m = *hitsMap->GetMap();
        for (const auto& kv : m) {
          if (kv.second) fluxCount += *(kv.second);
        }
      }
    }
  }

  // Poisson ノイズの追加（必要に応じて）。測定がランダムにばらつく状況を模擬する
  bool noiseEnabled  = g_noise_enabled.load(std::memory_order_relaxed);
  double noiseFactor = g_noise_strength.load(std::memory_order_relaxed);
  double measuredFlux = fluxCount;
  if (noiseEnabled && fluxCount > 0.0 && noiseFactor > 0.0) {
    // Poisson 分布の平均: fluxCount * noiseFactor
    static thread_local std::mt19937 rng(std::random_device{}());
    std::poisson_distribution<> dist(fluxCount * noiseFactor);
    double noise = static_cast<double>(dist(rng));
    measuredFlux = fluxCount + noise;
  }

  // 共有変数へ格納（エネルギーではなくカウントだが変数名はそのまま流用）
  g_last_edep_MeV.store(measuredFlux, std::memory_order_relaxed);

  // たまに表示（うるさくならないように）
  static const long print_every = 10;
  long b_now = g_b.load(std::memory_order_relaxed);
  if (b_now % print_every == 0) {
    std::cout << "[G4] Event #" << b_now
              << " : Flux=" << measuredFlux
              << " (PDG=" << g_particle_pdg.load()
              << ", E=" << g_energy_MeV.load() << " MeV)"
              << std::endl;
  }
}