#include "DetectorConstruction.hh"
#include <G4NistManager.hh>
#include <G4Box.hh>
#include <G4Sphere.hh>
#include <G4SubtractionSolid.hh>
#include <G4LogicalVolume.hh>
#include <G4PVPlacement.hh>
#include <G4SDManager.hh>
#include <G4MultiFunctionalDetector.hh>
#include <G4PSEnergyDeposit.hh>
#include <G4RotationMatrix.hh>
#include <G4RunManager.hh>
#include <G4ThreeVector.hh>
#include <G4SystemOfUnits.hh>
#include <G4PhysicalConstants.hh>

DetectorConstruction::DetectorConstruction()
: fLogicDet(nullptr),
  fDetPV(nullptr),
  fLogicShield(nullptr),
  fShieldPV(nullptr)
{
}

G4VPhysicalVolume* DetectorConstruction::Construct()
{
  auto nist = G4NistManager::Instance();

  // --- World（空気） ---
  auto worldMat   = nist->FindOrBuildMaterial("G4_AIR");
  auto solidWorld = new G4Box("World", 2.*m, 2.*m, 2.*m);
  auto logicWorld = new G4LogicalVolume(solidWorld, worldMat, "World");
  auto physWorld  = new G4PVPlacement(nullptr, G4ThreeVector(), logicWorld, "World", nullptr, false, 0, true);

  // --- 検出器：Si 薄板（XY に広く、Z に薄い）。原点に配置 ---
  auto siMat    = nist->FindOrBuildMaterial("G4_Si");
  auto solidDet = new G4Box("DetBox", 1.0*m, 1.0*m, 0.5*mm); // 厚さ 1 mm
  fLogicDet     = new G4LogicalVolume(solidDet, siMat, "DetLV");
  fDetPV        = new G4PVPlacement(nullptr, G4ThreeVector(), fLogicDet, "DetPV", logicWorld, false, 0, true);

  // --- MFD + Edep スコアを設定 ---
  auto sdman = G4SDManager::GetSDMpointer();
  auto mfd   = new G4MultiFunctionalDetector("Det");
  sdman->AddNewDetector(mfd);
  mfd->RegisterPrimitive(new G4PSEnergyDeposit("Edep"));
  fLogicDet->SetSensitiveDetector(mfd);

  // --- 遮蔽体：1/8 球殻（鉛）---
  // 外半径 0.7 m、内半径 0.6 m、phi:0-90 deg, theta:0-90 deg
  auto pbMat      = nist->FindOrBuildMaterial("G4_Pb");
  auto solidOuter = new G4Sphere("ShieldOuter", 0.0, 0.7*m,
                                 0.0*deg, 90.0*deg,
                                 0.0*deg, 90.0*deg);
  auto solidInner = new G4Sphere("ShieldInner", 0.0, 0.6*m,
                                 0.0*deg, 90.0*deg,
                                 0.0*deg, 90.0*deg);
  auto solidShell = new G4SubtractionSolid("ShieldShell", solidOuter, solidInner);
  fLogicShield    = new G4LogicalVolume(solidShell, pbMat, "ShieldLV");
  fShieldPV       = new G4PVPlacement(nullptr, G4ThreeVector(), fLogicShield, "ShieldPV", logicWorld, false, 0, true);

  return physWorld;
}

void DetectorConstruction::UpdateGeometry(const G4ThreeVector& detPos, double shieldAngleDeg)
{
  // 検出器の位置変更
  if (fDetPV) {
    fDetPV->SetTranslation(detPos);
    fDetPV->SetRotation(nullptr);
  }
  // 遮蔽体の回転（Z 軸回り）
  if (fShieldPV) {
    auto* rot = new G4RotationMatrix();
    rot->rotateZ(shieldAngleDeg * deg);
    fShieldPV->SetRotation(rot);
    fShieldPV->SetTranslation(G4ThreeVector());
  }
  // ジオメトリ変更を通知
  G4RunManager::GetRunManager()->GeometryHasBeenModified();
}
