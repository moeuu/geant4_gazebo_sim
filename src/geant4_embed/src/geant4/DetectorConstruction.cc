//
//  DetectorConstruction.cc — 8方向回転対応版
//
//  Geant4 用の検出器・遮蔽体ジオメトリ定義クラスです。遮蔽体は 1/8 球殻で、
//  Z 軸回転だけでなく、角度値が 360 度以上のときに Y 軸回り 180 度反転させる
//  仕組みにより全 8 方向の配置を実現します。

#include "DetectorConstruction.hh"

#include <G4Box.hh>
#include <G4Sphere.hh>
#include <G4SubtractionSolid.hh>
#include <G4PVPlacement.hh>
#include <G4LogicalVolume.hh>
#include <G4NistManager.hh>
#include <G4MultiFunctionalDetector.hh>
#include <G4PSEnergyDeposit.hh>
#include <G4SDManager.hh>
#include <G4RotationMatrix.hh>
#include <G4RunManager.hh>

// CLHEP 単位系。m, mm, deg などを CLHEP::m のように使う。
#include <CLHEP/Units/SystemOfUnits.h>

#include <cmath>

DetectorConstruction::DetectorConstruction()
  : fLogicDet(nullptr), fDetPV(nullptr), fLogicShield(nullptr), fShieldPV(nullptr)
{
}

G4VPhysicalVolume* DetectorConstruction::Construct()
{
    auto nist = G4NistManager::Instance();

    // --- World（空気） ---
    auto worldMat   = nist->FindOrBuildMaterial("G4_AIR");
    auto solidWorld = new G4Box("World", 2.0 * CLHEP::m, 2.0 * CLHEP::m, 2.0 * CLHEP::m);
    auto logicWorld = new G4LogicalVolume(solidWorld, worldMat, "World");
    auto physWorld  = new G4PVPlacement(nullptr, G4ThreeVector(), logicWorld, "World", nullptr, false, 0, true);

    // --- 検出器：Si 薄板 ---
    auto siMat    = nist->FindOrBuildMaterial("G4_Si");
    auto solidDet = new G4Box("DetBox", 1.0 * CLHEP::m, 1.0 * CLHEP::m, 0.5 * CLHEP::mm);
    fLogicDet     = new G4LogicalVolume(solidDet, siMat, "DetLV");
    fDetPV        = new G4PVPlacement(nullptr, G4ThreeVector(), fLogicDet, "DetPV", logicWorld, false, 0, true);

    // --- MFD + エネルギー付与スコア ---
    auto sdman = G4SDManager::GetSDMpointer();
    auto mfd   = new G4MultiFunctionalDetector("Det");
    sdman->AddNewDetector(mfd);
    mfd->RegisterPrimitive(new G4PSEnergyDeposit("Edep"));
    fLogicDet->SetSensitiveDetector(mfd);

    // --- 遮蔽体：1/8 球殻（鉛） ---
    auto pbMat      = nist->FindOrBuildMaterial("G4_Pb");
    auto solidOuter = new G4Sphere("ShieldOuter", 0.0, 0.7 * CLHEP::m,
                                   0.0 * CLHEP::deg, 90.0 * CLHEP::deg,
                                   0.0 * CLHEP::deg, 90.0 * CLHEP::deg);
    auto solidInner = new G4Sphere("ShieldInner", 0.0, 0.6 * CLHEP::m,
                                   0.0 * CLHEP::deg, 90.0 * CLHEP::deg,
                                   0.0 * CLHEP::deg, 90.0 * CLHEP::deg);
    auto solidShell = new G4SubtractionSolid("ShieldShell", solidOuter, solidInner);
    fLogicShield    = new G4LogicalVolume(solidShell, pbMat, "ShieldLV");
    fShieldPV       = new G4PVPlacement(nullptr, G4ThreeVector(), fLogicShield, "ShieldPV", logicWorld, false, 0, true);

    return physWorld;
}

void DetectorConstruction::UpdateGeometry(const G4ThreeVector& detPos, double shieldAngleDeg)
{
    // 検出器位置の更新
    if (fDetPV) {
        fDetPV->SetTranslation(detPos);
        fDetPV->SetRotation(nullptr);
    }

    // 遮蔽体の姿勢変更（Z 軸回転および Y 軸反転）
    if (fShieldPV) {
        double phi = shieldAngleDeg;
        bool flip = false;
        if (phi >= 360.0) {
            phi -= 360.0;
            flip = true;
        }
        auto* rot = new G4RotationMatrix();
        rot->rotateZ(phi * CLHEP::deg);
        if (flip) {
            rot->rotateY(180.0 * CLHEP::deg);
        }
        fShieldPV->SetRotation(rot);
        fShieldPV->SetTranslation(G4ThreeVector());
    }

    // ジオメトリ更新の通知
    G4RunManager::GetRunManager()->GeometryHasBeenModified();
}