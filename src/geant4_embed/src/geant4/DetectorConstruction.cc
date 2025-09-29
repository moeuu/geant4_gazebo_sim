//
//  DetectorConstruction.cc — 非指向性検出器用ジオメトリ定義
//
//  Gazebo 上のロボットに搭載する等方性（非指向性）検出器を
//  定義するための Geant4 ジオメトリクラスです。検出器は小さな球形
//  ボリュームとして実装され、全方向から入射する放射線のトラック数を
//  測定します。

#include "DetectorConstruction.hh"

#include <G4Box.hh>
#include <G4Sphere.hh>
#include <G4LogicalVolume.hh>
#include <G4PVPlacement.hh>
#include <G4NistManager.hh>
#include <G4MultiFunctionalDetector.hh>
#include <G4PSCellFlux.hh>
#include <G4SDManager.hh>
#include <G4RunManager.hh>
#include <CLHEP/Units/SystemOfUnits.h>

using CLHEP::m;
using CLHEP::deg;

DetectorConstruction::DetectorConstruction()
  : fLogicDet(nullptr), fDetPV(nullptr)
{
}

// Construct the world and the isotropic spherical detector
G4VPhysicalVolume* DetectorConstruction::Construct()
{
    auto nist = G4NistManager::Instance();

    // --- World volume (air) ---
    auto worldMat   = nist->FindOrBuildMaterial("G4_AIR");
    // Configure the world to encompass the world_demo.obj extent (20 × 10 × 20 m)
    auto solidWorld = new G4Box("World", 10.0 * m, 5.0 * m, 10.0 * m);
    auto logicWorld = new G4LogicalVolume(solidWorld, worldMat, "World");
    auto physWorld  = new G4PVPlacement(nullptr, G4ThreeVector(), logicWorld, "World", nullptr, false, 0, true);

    // --- Non-directional detector: spherical volume ---
    // Use a realistic scintillator material (NaI(Tl)) for Cs‑137 gamma detection.
    double detRadius     = 0.05 * m;               // 5 cm radius detector crystal
    double shieldThickness = 0.01 * m;             // 1 cm thick shielding
    auto detMat          = nist->FindOrBuildMaterial("G4_SODIUM_IODIDE");
    // Detector sphere covering full 4π steradians
    auto solidDet        = new G4Sphere("DetSphere", 0.0, detRadius,
                                        0.0 * deg, 360.0 * deg,
                                        0.0 * deg, 180.0 * deg);
    fLogicDet            = new G4LogicalVolume(solidDet, detMat, "DetLV");
    fDetPV               = new G4PVPlacement(nullptr, G4ThreeVector(), fLogicDet, "DetPV", logicWorld, false, 0, true);

    // --- Shield: 1/8 spherical shell attached to the detector ---
    auto shieldMat       = nist->FindOrBuildMaterial("G4_Pb");
    auto solidShield     = new G4Sphere("ShieldShell", detRadius, detRadius + shieldThickness,
                                        0.0 * deg, 90.0 * deg,     // phi range: 0°–90° (1/4 of 360°)
                                        0.0 * deg, 90.0 * deg);    // theta range: 0°–90° (upper hemisphere)
    fLogicShield         = new G4LogicalVolume(solidShield, shieldMat, "ShieldLV");
    fShieldPV            = new G4PVPlacement(nullptr, G4ThreeVector(), fLogicShield, "ShieldPV", logicWorld, false, 0, true);

    // --- MultiFunctionalDetector and flux scoring primitive ---
    auto sdman = G4SDManager::GetSDMpointer();
    auto mfd   = new G4MultiFunctionalDetector("Det");
    sdman->AddNewDetector(mfd);
    // G4PSCellFlux counts the number of tracks passing through the detector volume
    mfd->RegisterPrimitive(new G4PSCellFlux("Flux"));
    fLogicDet->SetSensitiveDetector(mfd);

    return physWorld;
}

// Update the detector position and rotate the shield wedge
void DetectorConstruction::UpdateGeometry(const G4ThreeVector& detPos, double shieldAngleDeg)
{
    // Update detector position; keep orientation fixed since the detector is isotropic
    if (fDetPV) {
        fDetPV->SetTranslation(detPos);
        fDetPV->SetRotation(nullptr);
    }

    // Rotate and reposition the 1/8 spherical shield to cover different directions
    if (fShieldPV) {
        double phi = shieldAngleDeg;
        bool flip = false;
        if (phi >= 360.0) {
            phi -= 360.0;
            flip = true;
        }
        auto* rot = new G4RotationMatrix();
        rot->rotateZ(phi * deg);
        if (flip) {
            rot->rotateY(180.0 * deg);
        }
        fShieldPV->SetRotation(rot);
        // Shield is always centered on the detector, so translation is zero
        fShieldPV->SetTranslation(G4ThreeVector());
    }

    // Notify Geant4 about geometry changes
    G4RunManager::GetRunManager()->GeometryHasBeenModified();
}