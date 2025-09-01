#include "Measurement.hpp"
#include "DetectorConstruction.hh"
#include "ActionInitialization.hh"
#include "sim_shared.hpp"

#include <G4RunManagerFactory.hh>
#include <G4PhysListFactory.hh>
#include <G4UImanager.hh>
#include <G4SystemOfUnits.hh>

#include <numeric>
#include <vector>

/**
 * @brief 測定を実行する実装
 *
 * 新しい RunManager を生成し、DetectorConstruction・PhysicsList・ActionInitialization
 * をセットアップして初期化後、ジオメトリを再配置し指定回数のイベントを実行する。
 */
MeasurementResult performMeasurement(const G4ThreeVector& detectorPos,
                                     double shieldAngleDeg,
                                     long numEvents)
{
  // RunManager の生成（シリアル実行）
  auto* runManager = G4RunManagerFactory::CreateRunManager(G4RunManagerType::Serial);

  // Detector
  auto* det = new DetectorConstruction();
  runManager->SetUserInitialization(det);

  // PhysicsList
  G4PhysListFactory factory;
  auto phys = factory.GetReferencePhysList("FTFP_BERT");
  runManager->SetUserInitialization(phys);

  // ActionInitialization
  runManager->SetUserInitialization(new ActionInitialization());

  // 初期化
  runManager->Initialize();

  // 詳細出力を抑制
  auto* UI = G4UImanager::GetUIpointer();
  UI->ApplyCommand("/run/verbose 0");
  UI->ApplyCommand("/event/verbose 0");
  UI->ApplyCommand("/tracking/verbose 0");
  UI->ApplyCommand("/process/verbose 0");

  // 検出器位置と遮蔽体角度を更新
  det->UpdateGeometry(detectorPos, shieldAngleDeg);

  // イベント実行と測定結果の収集
  std::vector<double> edeps;
  edeps.reserve(static_cast<std::size_t>(numEvents));

  for (long i = 0; i < numEvents; ++i) {
    runManager->BeamOn(1);
    double edep = g_last_edep_MeV.load(std::memory_order_relaxed);
    edeps.push_back(edep);
  }

  // 統計計算
  double sum = std::accumulate(edeps.begin(), edeps.end(), 0.0);
  double mean = numEvents > 0 ? (sum / static_cast<double>(numEvents)) : 0.0;
  double var  = 0.0;
  if (numEvents > 0) {
    for (double v : edeps) {
      double diff = v - mean;
      var += diff * diff;
    }
    var /= static_cast<double>(numEvents);
  }

  // RunManager の解放
  delete runManager;

  return { numEvents, mean, var };
}
