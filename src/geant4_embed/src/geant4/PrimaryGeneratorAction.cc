// PrimaryGeneratorAction for multiple Cs‑137 sources
//
// This implementation extends the original PrimaryGeneratorAction to support
// multiple Cs‑137 radioactive sources.  For each Geant4 event we emit one
// primary ion at the position of each source specified in g_sources.  If
// g_sources is empty, the legacy single‑source variables g_source_x/y/z are
// used.  The ion definition is a neutral Cs‑137 nucleus at rest; Geant4's
// RadioactiveDecayPhysics will handle the β⁻ decay to Ba‑137m and the
// subsequent γ emission.  Directions are sampled isotropically over the full
// 4π steradians.

#include "PrimaryGeneratorAction.hh"
#include "sim_shared_multi.hpp"

#include <G4ParticleGun.hh>
#include <G4ParticleTable.hh>
#include <G4IonTable.hh>
#include <G4SystemOfUnits.hh>
#include <G4ThreeVector.hh>
#include <G4PhysicalConstants.hh>
#include <Randomize.hh>
#include <algorithm>

PrimaryGeneratorAction::PrimaryGeneratorAction()
{
  // Allocate a particle gun capable of producing one primary per call
  fGun = new G4ParticleGun(1);

  // Default to a neutral Cs‑137 ion at rest.  These fields will be
  // overwritten at runtime but provide sensible defaults.  Note that
  // Cs‑137 is extremely long‑lived; kinetic energy is set to zero.
  auto ionTable = G4IonTable::GetIonTable();
  auto cs137    = ionTable->GetIon(55, 137, 0.0);
  if (cs137) {
    fGun->SetParticleDefinition(cs137);
    fGun->SetParticleCharge(0.0);
  } else {
    // Fall back to geantino if Cs‑137 definition is missing
    auto* table = G4ParticleTable::GetParticleTable();
    fGun->SetParticleDefinition(table->FindParticle("geantino"));
    fGun->SetParticleCharge(0.0);
  }
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
  // Acquire the Cs‑137 ion definition each event; this ensures any dynamic
  // modifications to the ion table are respected.  Cs‑137 has Z=55,
  // A=137, and is produced in its ground state.  Should the definition be
  // unavailable, a geantino is used instead (though this should never
  // happen with a properly configured physics list).
  auto ionTable = G4IonTable::GetIonTable();
  auto cs137    = ionTable->GetIon(55, 137, 0.0);
  G4ParticleDefinition* def;
  if (cs137) {
    def = cs137;
    g_particle_pdg.store(cs137->GetPDGEncoding(), std::memory_order_relaxed);
  } else {
    def = G4ParticleTable::GetParticleTable()->FindParticle("geantino");
    g_particle_pdg.store(-1, std::memory_order_relaxed);
  }
  fGun->SetParticleDefinition(def);
  fGun->SetParticleCharge(0.0);

  // Cs‑137 is essentially stationary; assign zero kinetic energy.  The energy
  // deposition will occur via decay processes implemented by Geant4.
  fGun->SetParticleEnergy(0.0 * keV);
  g_energy_MeV.store(0.0, std::memory_order_relaxed);

  // Generate an isotropic direction for this event.  Use a uniform random
  // distribution in cosθ and φ to avoid clustering at the poles.  This
  // direction will be reused for all sources in this event to keep sampling
  // consistent; if you prefer independent directions per source, move this
  // block inside the loop below.
  const double cosTheta = 2.0 * G4UniformRand() - 1.0;
  const double sinTheta = std::sqrt(std::max(0.0, 1.0 - cosTheta * cosTheta));
  const double phi      = CLHEP::twopi * G4UniformRand();
  G4ThreeVector direction(sinTheta * std::cos(phi), sinTheta * std::sin(phi), cosTheta);
  fGun->SetParticleMomentumDirection(direction.unit());
  g_yaw_rad.store(std::atan2(direction.y(), direction.x()), std::memory_order_relaxed);

  // Copy the source positions into a local variable to avoid data races
  // if another thread modifies g_sources while this function is executing.
  const auto local_sources = g_sources;

  // If multiple sources are defined, emit one primary per source.  If no
  // sources are defined, fall back to the single legacy source.  Note that
  // intensity scaling is not currently used to adjust the number of
  // primaries; this could be implemented by weighting or duplication.
  if (!local_sources.empty()) {
    for (std::size_t i = 0; i < local_sources.size(); ++i) {
      const auto& pos  = local_sources[i];
      // Set position in metres; Geant4 expects SI units
      fGun->SetParticlePosition(G4ThreeVector(pos[0] * m, pos[1] * m, pos[2] * m));
      // Generate the vertex.  This call implicitly uses the most recently
      // set particle definition, energy, direction and charge.
      fGun->GeneratePrimaryVertex(event);
    }
  } else {
    // Single legacy source
    const double sx = g_source_x.load(std::memory_order_relaxed);
    const double sy = g_source_y.load(std::memory_order_relaxed);
    const double sz = g_source_z.load(std::memory_order_relaxed);
    fGun->SetParticlePosition(G4ThreeVector(sx * m, sy * m, sz * m));
    fGun->GeneratePrimaryVertex(event);
  }
}