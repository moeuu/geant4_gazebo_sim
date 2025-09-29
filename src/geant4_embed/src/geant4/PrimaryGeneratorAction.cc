#include "PrimaryGeneratorAction.hh"
#include "sim_shared.hpp"

#include <G4ParticleGun.hh>
#include <G4ParticleTable.hh>
#include <G4IonTable.hh>
#include <G4SystemOfUnits.hh>
#include <G4ThreeVector.hh>
#include <G4PhysicalConstants.hh>
#include <Randomize.hh>

/**
 * @brief Modified PrimaryGeneratorAction that emits a Cs‑137 ion which decays via the
 *        Geant4 RadioactiveDecay process.  The source position is read from the
 *        global simulation parameters (g_source_x/y/z) so that the source is
 *        always placed outside the detector.  Each primary event starts with a
 *        single Cs‑137 nucleus at rest; Geant4 then handles the β⁻ decay to
 *        Ba‑137m and the subsequent emission of a 662 keV γ ray.  The initial
 *        direction is sampled uniformly over 4π so that the decay products
 *        propagate isotropically.
 */
PrimaryGeneratorAction::PrimaryGeneratorAction()
{
  // Create a single–particle gun
  fGun = new G4ParticleGun(1);

  // Default to a neutral Cs‑137 ion at rest.  We set the definition here so
  // that Geant4 knows the primary particle is radioactive; however, the
  // definition and kinematics are re‑assigned in GeneratePrimaries() each
  // event to accommodate dynamic parameters.
  auto ionTable = G4IonTable::GetIonTable();
  auto cs137 = ionTable->GetIon(55, 137, 0.0); // Z=55 (Cs), A=137, E=0 (ground state)
  if (cs137) {
    fGun->SetParticleDefinition(cs137);
    fGun->SetParticleCharge(0.);
  }
  // Cs‑137 is extremely long‑lived, so the kinetic energy is set to zero and
  // decay occurs via the RadioactiveDecay process.  Direction and position
  // will be set in GeneratePrimaries().
  fGun->SetParticleEnergy(0.0 * keV);
  fGun->SetParticleMomentumDirection(G4ThreeVector(1., 0., 0.));
  fGun->SetParticlePosition(G4ThreeVector(0., 0., 0.));
}

PrimaryGeneratorAction::~PrimaryGeneratorAction()
{
  delete fGun;
}

void PrimaryGeneratorAction::GeneratePrimaries(G4Event* event)
{
  // Obtain the ion definition for Cs‑137.  Geant4 will handle the β decay
  // through the RadioactiveDecayPhysics registered in the physics list.
  auto ionTable = G4IonTable::GetIonTable();
  auto cs137 = ionTable->GetIon(55, 137, 0.0);
  if (cs137) {
    fGun->SetParticleDefinition(cs137);
    fGun->SetParticleCharge(0.0);
    // Update global PDG code for logging.  The PDG code for nuclei in
    // Geant4 follows the convention 1000000000 + 10000*Z + 10*A.  Storing
    // this value allows the event logger to report the correct species.
    g_particle_pdg.store(cs137->GetPDGEncoding(), std::memory_order_relaxed);
  } else {
    // Should never happen, but fall back to geantino if ion is missing.
    auto* table = G4ParticleTable::GetParticleTable();
    fGun->SetParticleDefinition(table->FindParticle("geantino"));
    g_particle_pdg.store(-1, std::memory_order_relaxed);
  }

  // Cs‑137 is stationary relative to the lab frame; kinetic energy is zero.
  fGun->SetParticleEnergy(0.0 * keV);
  g_energy_MeV.store(0.0, std::memory_order_relaxed);

  // Sample an isotropic direction over the full 4π steradians.  A uniform
  // distribution in cosθ and φ yields an unbiased direction vector.
  double cosTheta = 2.0 * G4UniformRand() - 1.0;
  double sinTheta = std::sqrt(std::max(0.0, 1.0 - cosTheta * cosTheta));
  double phi = CLHEP::twopi * G4UniformRand();
  G4ThreeVector dir(sinTheta * std::cos(phi), sinTheta * std::sin(phi), cosTheta);
  fGun->SetParticleMomentumDirection(dir.unit());
  // Store yaw for completeness, though isotropic sampling makes it irrelevant
  g_yaw_rad.store(std::atan2(dir.y(), dir.x()), std::memory_order_relaxed);

  // Use global atomic variables to set the source position (in metres).  In this
  // simulation, the radiation source is placed outside the detector and the
  // detector geometry is moved independently via DetectorConstruction.
  double sx = g_source_x.load(std::memory_order_relaxed);
  double sy = g_source_y.load(std::memory_order_relaxed);
  double sz = g_source_z.load(std::memory_order_relaxed);
  fGun->SetParticlePosition(G4ThreeVector(sx * m, sy * m, sz * m));

  // Generate the primary vertex.  The subsequent decay will be handled by the
  // physics processes registered in the run manager.
  fGun->GeneratePrimaryVertex(event);
}