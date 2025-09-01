#pragma once
#include <G4VUserDetectorConstruction.hh>
class G4VPhysicalVolume;

class DetectorConstruction : public G4VUserDetectorConstruction
{
public:
  DetectorConstruction() = default;
  ~DetectorConstruction() override = default;
  G4VPhysicalVolume* Construct() override;
};
