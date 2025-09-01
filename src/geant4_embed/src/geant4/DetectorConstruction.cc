#include "DetectorConstruction.hh"
#include <G4NistManager.hh>
#include <G4Box.hh>
#include <G4LogicalVolume.hh>
#include <G4PVPlacement.hh>
#include <G4SystemOfUnits.hh>
#include <G4SDManager.hh>
#include <G4MultiFunctionalDetector.hh>
#include <G4PSEnergyDeposit.hh>

G4VPhysicalVolume* DetectorConstruction::Construct()
{
  auto nist = G4NistManager::Instance();

  // --- World（空気） ---
  auto worldMat   = nist->FindOrBuildMaterial("G4_AIR");
  auto solidWorld = new G4Box("World", 2.*m, 2.*m, 2.*m);
  auto logicWorld = new G4LogicalVolume(solidWorld, worldMat, "World");
  auto physWorld  = new G4PVPlacement(nullptr, {}, logicWorld, "World", nullptr, false, 0, true);

  // --- 検出器：Si薄板（XYに広く、Zに薄い）。原点に配置 ---
  auto siMat   = nist->FindOrBuildMaterial("G4_Si");
  auto solidDet = new G4Box("DetBox", 1.0*m, 1.0*m, 0.5*mm); // 厚さ1mm
  auto logicDet = new G4LogicalVolume(solidDet, siMat, "DetLV");
  new G4PVPlacement(nullptr, {}, logicDet, "DetPV", logicWorld, false, 0, true);

  // --- MFD + Edep スコアを設定 ---
  auto sdman = G4SDManager::GetSDMpointer();
  auto mfd   = new G4MultiFunctionalDetector("Det");
  sdman->AddNewDetector(mfd);
  mfd->RegisterPrimitive(new G4PSEnergyDeposit("Edep"));
  logicDet->SetSensitiveDetector(mfd);

  return physWorld;
}
