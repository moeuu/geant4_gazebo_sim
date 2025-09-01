#include "PrimaryGeneratorAction.hh"
#include "sim_shared.hpp"

#include <G4ParticleGun.hh>
#include <G4ParticleTable.hh>
#include <G4SystemOfUnits.hh>
#include <G4ThreeVector.hh>
#include <CLHEP/Units/SystemOfUnits.h>  // deg 等を使う場合はこちらでも可
#include <cmath>
#include <climits>

PrimaryGeneratorAction::PrimaryGeneratorAction()
{
  fGun = new G4ParticleGun(1);

  // 既定は geantino
  auto particle = G4ParticleTable::GetParticleTable()->FindParticle("geantino");
  fGun->SetParticleDefinition(particle);

  // 初期値（GeneratePrimaries で毎回更新）
  fGun->SetParticleEnergy(1.0 * MeV);
  fGun->SetParticleMomentumDirection(G4ThreeVector(1., 0., 0.)); // +X 方向（yaw で上書き）
  fGun->SetParticlePosition(G4ThreeVector(0., 0., 0.));
}

PrimaryGeneratorAction::~PrimaryGeneratorAction()
{
  delete fGun;
}

void PrimaryGeneratorAction::GeneratePrimaries(G4Event* event)
{
  auto* table = G4ParticleTable::GetParticleTable();

  // 粒子種（PDG）を反映
  static int last_pdg = INT_MAX;
  int pdg = g_particle_pdg.load(std::memory_order_relaxed);
  if (pdg != last_pdg) {
    if (pdg == -1) {
      fGun->SetParticleDefinition(table->FindParticle("geantino"));
    } else {
      auto* p = table->FindParticle(pdg);
      if (p) fGun->SetParticleDefinition(p);
      else   fGun->SetParticleDefinition(table->FindParticle("geantino"));
    }
    last_pdg = pdg;
  }

  // エネルギー [MeV]
  double eMeV = g_energy_MeV.load(std::memory_order_relaxed);
  if (!std::isfinite(eMeV) || eMeV < 0.0) eMeV = 0.0;
  fGun->SetParticleEnergy(eMeV * MeV);

  // yaw [rad] → 方向ベクトル (cos, sin, 0)
  double yaw = g_yaw_rad.load(std::memory_order_relaxed);
  if (!std::isfinite(yaw)) yaw = 0.0;
  G4ThreeVector dir(std::cos(yaw), std::sin(yaw), 0.0);
  if (dir.mag2() < 1e-12) dir = G4ThreeVector(1., 0., 0.);
  fGun->SetParticleMomentumDirection(dir.unit());

  // 線源位置 [m] を反映
  double sx = g_source_x.load(std::memory_order_relaxed);
  double sy = g_source_y.load(std::memory_order_relaxed);
  double sz = g_source_z.load(std::memory_order_relaxed);
  fGun->SetParticlePosition(G4ThreeVector(sx * m, sy * m, sz * m));

  // 発射
  fGun->GeneratePrimaryVertex(event);
}
