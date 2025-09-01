#pragma once
#include <G4ThreeVector.hh>

/**
 * @brief 測定結果を保持する構造体
 *
 * events    : 実行したイベント数
 * mean      : Edep の平均値 [MeV]
 * variance  : Edep の母分散 [MeV^2]
 */
struct MeasurementResult {
  long   events;
  double mean;
  double variance;
};

/**
 * @brief 検出器位置と遮蔽体角度に応じてジオメトリを再配置し測定を実施する
 *
 * @param detectorPos    検出器の位置 [m]（G4ThreeVector）
 * @param shieldAngleDeg 遮蔽体の回転角度 [deg]（Z 軸回り）
 * @param numEvents      実行するイベント数
 * @return MeasurementResult 平均・分散などを格納した結果
 *
 * この関数は内部で RunManager を生成し、1 イベントずつ実行して統計を取得します。
 */
MeasurementResult performMeasurement(const G4ThreeVector& detectorPos,
                                     double shieldAngleDeg,
                                     long numEvents);
