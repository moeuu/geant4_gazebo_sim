// DetectorConstruction.hh — 非指向性検出器 + 1/8球殻鉛遮蔽体
#pragma once

#include <G4VUserDetectorConstruction.hh>
#include <G4ThreeVector.hh>
#include <G4RotationMatrix.hh>  // ★ 前方宣言ではなく正式ヘッダをインクルード

class G4LogicalVolume;
class G4PVPlacement;

class DetectorConstruction : public G4VUserDetectorConstruction {
public:
  DetectorConstruction();
  ~DetectorConstruction() override = default;

  G4VPhysicalVolume* Construct() override;

  // 検出器位置と遮蔽体の向きを更新（角度はdeg）
  // shieldAngleDeg: Z軸まわり回転。360°超過時はY反転を併用して1/8殻で全方位をカバー。
  void UpdateGeometry(const G4ThreeVector& detPos, double shieldAngleDeg);

private:
  // 検出器
  G4LogicalVolume* fLogicDet   = nullptr;
  G4PVPlacement*   fDetPV      = nullptr;

  // 遮蔽体
  G4LogicalVolume*  fLogicShield = nullptr;
  G4PVPlacement*    fShieldPV    = nullptr;
  G4RotationMatrix* fShieldRot   = nullptr;
};
