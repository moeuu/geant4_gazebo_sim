// DetectorConstruction.cc — 非指向性検出器 + 1/8球殻鉛遮蔽体

#include "DetectorConstruction.hh"

#include <G4Box.hh>
#include <G4Sphere.hh>
#include <G4LogicalVolume.hh>
#include <G4PVPlacement.hh>
#include <G4NistManager.hh>
#include <G4SystemOfUnits.hh>
#include <G4PhysicalConstants.hh>
#include <G4RunManager.hh>
#include <G4VisAttributes.hh>
#include <G4Colour.hh>

// スコアリング（セル通過フラックス）
#include <G4SDManager.hh>
#include <G4MultiFunctionalDetector.hh>
#include <G4PSCellFlux.hh>

#include <cmath>  // std::fmod

DetectorConstruction::DetectorConstruction() = default;

G4VPhysicalVolume* DetectorConstruction::Construct() {
  auto* nist = G4NistManager::Instance();

  // --- World: 20×10×20 m の空間（半サイズで指定）
  auto* worldMat    = nist->FindOrBuildMaterial("G4_AIR");
  auto* solidWorld  = new G4Box("World", 10.0 * m, 5.0 * m, 10.0 * m);
  auto* logicWorld  = new G4LogicalVolume(solidWorld, worldMat, "WorldLV");
  auto* physWorld   = new G4PVPlacement(nullptr, {}, logicWorld, "WorldPV",
                                        nullptr, false, 0, true);

  // --- Detector: 球形 NaI(Tl) 相当（NaI）
  const double detRadius = 0.05 * m; // 5 cm
  auto* detMat   = nist->FindOrBuildMaterial("G4_SODIUM_IODIDE");
  auto* solidDet = new G4Sphere("DetSphere",
                                0.0, detRadius,
                                0.0 * deg, 360.0 * deg,
                                0.0 * deg, 180.0 * deg);
  fLogicDet = new G4LogicalVolume(solidDet, detMat, "DetLV");
  fDetPV    = new G4PVPlacement(nullptr, {}, fLogicDet, "DetPV",
                                logicWorld, false, 0, true);

  // --- Shield: 鉛の 1/8 球殻（TVL ≈ 22 mm）
  const double shieldThickness = 0.022 * m; // 22 mm
  auto* shieldMat   = nist->FindOrBuildMaterial("G4_Pb");
  // ★ solidShield は1回だけ宣言
  auto* solidShield = new G4Sphere("ShieldShell",
                                   detRadius, detRadius + shieldThickness,
                                   0.0 * deg, 90.0 * deg,   // φ: 0–90°
                                   0.0 * deg, 90.0 * deg);  // θ: 0–90°
  fLogicShield = new G4LogicalVolume(solidShield, shieldMat, "ShieldLV");
  // 初期配置は原点・無回転。UpdateGeometry() で回転・並進を与える
  fShieldPV = new G4PVPlacement(nullptr, {}, fLogicShield, "ShieldPV",
                                logicWorld, false, 0, true);

  // --- 視覚属性（任意）
  logicWorld->SetVisAttributes(G4VisAttributes::GetInvisible());

  auto* detVis = new G4VisAttributes(G4Colour(0.1, 0.6, 1.0, 0.4)); // 半透明シアン
  detVis->SetForceSolid(true);
  fLogicDet->SetVisAttributes(detVis);

  auto* shieldVis = new G4VisAttributes(G4Colour(0.5, 0.5, 0.5, 0.6)); // 半透明グレー
  shieldVis->SetForceSolid(true);
  fLogicShield->SetVisAttributes(shieldVis);

  // --- スコアリング（トラック数を計数）
  auto* sdMan = G4SDManager::GetSDMpointer();
  auto* mfd   = new G4MultiFunctionalDetector("Det");
  sdMan->AddNewDetector(mfd);
  mfd->RegisterPrimitive(new G4PSCellFlux("Flux")); // コレクション名: Det/Flux
  fLogicDet->SetSensitiveDetector(mfd);

  // 回転行列のインスタンス（以後 UpdateGeometry 内で再利用）
  if (!fShieldRot) fShieldRot = new G4RotationMatrix();

  return physWorld;
}

void DetectorConstruction::UpdateGeometry(const G4ThreeVector& detPos,
                                          double shieldAngleDeg) {
  // 検出器の位置を更新
  if (fDetPV) {
    fDetPV->SetTranslation(detPos);
    fDetPV->SetRotation(nullptr);
  }

  // 遮蔽体の回転・位置を更新
  if (fShieldPV) {
    // 角度規約：
    //   Z 軸周りに shieldAngleDeg を回転。360°超過で Y 方向に 180° 反転
    double angle = shieldAngleDeg;
    bool flipY = false;
    if (angle >= 360.0) {
      angle = std::fmod(angle, 360.0);
      flipY = true;
    }

    // 回転行列を設定
    *fShieldRot = G4RotationMatrix(); // identity
    fShieldRot->rotateZ(angle * deg);
    if (flipY) {
      fShieldRot->rotateY(180.0 * deg);
    }

    fShieldPV->SetRotation(fShieldRot);
    fShieldPV->SetTranslation(detPos);
  }

  // 幾何更新を通知
  G4RunManager::GetRunManager()->GeometryHasBeenModified();
}
