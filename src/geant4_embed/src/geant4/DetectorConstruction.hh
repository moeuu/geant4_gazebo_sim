#pragma once
#include <G4VUserDetectorConstruction.hh>
#include <G4ThreeVector.hh>

class G4LogicalVolume;
class G4PVPlacement;

/**
 * @brief シミュレーション用検出器および遮蔽体の構築クラス
 *
 * 1/8 球殻遮蔽体を含むジオメトリを定義し、検出器・遮蔽体の再配置をサポートします。
 */
class DetectorConstruction : public G4VUserDetectorConstruction
{
public:
  DetectorConstruction();
  ~DetectorConstruction() override = default;

  /// ジオメトリ構築
  G4VPhysicalVolume* Construct() override;

  /**
   * @brief 検出器位置および遮蔽体角度による再配置
   *
   * @param detPos        検出器の配置位置 [m]
   * @param shieldAngleDeg 遮蔽体の回転角度 [deg]（Z 軸回りに回転）
   */
  void UpdateGeometry(const G4ThreeVector& detPos, double shieldAngleDeg);

private:
  // 検出器と遮蔽体の論理/物理ボリューム
  G4LogicalVolume* fLogicDet;
  G4PVPlacement*   fDetPV;
  G4LogicalVolume* fLogicShield;
  G4PVPlacement*   fShieldPV;
};
