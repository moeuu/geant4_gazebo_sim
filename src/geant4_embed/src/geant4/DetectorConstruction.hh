//
//  DetectorConstruction.hh — 非指向性検出器用ジオメトリ定義
//
//  Gazebo 上のロボットに搭載する等方性（非指向性）検出器を
//  定義するための Geant4 ジオメトリクラスです。検出器は小さな球形
//  ボリュームとして実装され、全方向から入射する放射線のトラック数を
//  測定します。遮蔽体を持たず、UpdateGeometry() では検出器の位置のみ
//  を更新します。

#pragma once

#include <G4VUserDetectorConstruction.hh>
#include <G4ThreeVector.hh>

class G4LogicalVolume;
class G4PVPlacement;

/**
 * @brief 非指向性検出器ジオメトリの構築クラス
 *
 * シンプルな球形検出器（半径数 cm）を実装し、位置の更新にのみ対応
 * します。放射線遮蔽体は存在せず、全方向からの放射線入射を計測する
 * ことを目的としています。
 */
class DetectorConstruction : public G4VUserDetectorConstruction
{
public:
  DetectorConstruction();
  ~DetectorConstruction() override = default;

  /// ジオメトリ構築
  G4VPhysicalVolume* Construct() override;

  /**
   * @brief 検出器位置を更新する
   *
   * @param detPos        新しい検出器の配置位置 [m]
   * @param shieldAngleDeg 未使用（互換性維持のため）
   */
  void UpdateGeometry(const G4ThreeVector& detPos, double shieldAngleDeg);

private:
  // 検出器および遮蔽体の論理/物理ボリューム
  G4LogicalVolume* fLogicDet;
  G4PVPlacement*   fDetPV;
  G4LogicalVolume* fLogicShield;
  G4PVPlacement*   fShieldPV;
};