#include "Measurement.hpp"
#include "DetectorConstruction.hh"
#include "ActionInitialization.hh"
#include "sim_shared.hpp"

#include <G4RunManagerFactory.hh>
#include <G4PhysListFactory.hh>
#include <G4UImanager.hh>
#include <G4SystemOfUnits.hh>
#include <G4DecayPhysics.hh>
#include <G4RadioactiveDecayPhysics.hh>

#include <numeric>
#include <vector>

/**
 * @brief Perform a measurement by running a specified number of Geant4 events.
 *
 * A new run manager is created for each measurement.  The detector is
 * constructed, the physics list (FTFP_BERT plus radioactive decay) is
 * initialized, and the action initialization sets up the primary generator
 * (Cs‑137 decay) and event action.  The detector geometry is then updated
 * according to the supplied position and shield angle, and the specified
 * number of events is executed.  The energy deposited in the detector is
 * accumulated and returned as the mean and variance.
 */
MeasurementResult performMeasurement(const G4ThreeVector& detectorPos,
                                     double shieldAngleDeg,
                                     long numEvents)
{
  // Create a serial run manager
  auto* runManager = G4RunManagerFactory::CreateRunManager(G4RunManagerType::Serial);

  // Detector geometry
  auto* det = new DetectorConstruction();
  runManager->SetUserInitialization(det);

  // Physics list: FTFP_BERT with decay and radioactive decay
  G4PhysListFactory factory;
  auto phys = factory.GetReferencePhysList("FTFP_BERT");
  phys->RegisterPhysics(new G4DecayPhysics());
  phys->RegisterPhysics(new G4RadioactiveDecayPhysics());
  runManager->SetUserInitialization(phys);

  // Action initialization
  runManager->SetUserInitialization(new ActionInitialization());

  // Initialize and suppress verbose output
  runManager->Initialize();
  auto* UI = G4UImanager::GetUIpointer();
  UI->ApplyCommand("/run/verbose 0");
  UI->ApplyCommand("/event/verbose 0");
  UI->ApplyCommand("/tracking/verbose 0");
  UI->ApplyCommand("/process/verbose 0");
  UI->ApplyCommand("/process/had/rdm/thresholdForVeryLongDecayTime 1.0e+60 year");

  // Update detector position and shield angle
  det->UpdateGeometry(detectorPos, shieldAngleDeg);

  // Collect energy deposits
  std::vector<double> edeps;
  edeps.reserve(static_cast<std::size_t>(numEvents));

  for (long i = 0; i < numEvents; ++i) {
    runManager->BeamOn(1);
    double edep = g_last_edep_MeV.load(std::memory_order_relaxed);
    edeps.push_back(edep);
  }

  // Compute mean and variance
  double sum = std::accumulate(edeps.begin(), edeps.end(), 0.0);
  double mean = numEvents > 0 ? (sum / static_cast<double>(numEvents)) : 0.0;
  double var  = 0.0;
  if (numEvents > 0) {
    for (double v : edeps) {
      double diff = v - mean;
      var += diff * diff;
    }
    var /= static_cast<double>(numEvents);
  }

  delete runManager;
  return { numEvents, mean, var };
}