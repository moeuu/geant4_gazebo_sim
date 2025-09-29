// This header extends the original sim_shared.hpp to support multiple
// radioactive sources.  It defines a vector of source positions
// and optional per‑source intensities while preserving the existing
// single‑source variables for backwards compatibility.  Each source
// position is a triple of (x,y,z) coordinates in metres.  See
// sim_shared_multi.cpp for definitions.

#pragma once

#include <atomic>
#include <array>
#include <vector>

// -----------------------------------------------------------------------------
// Shared simulation variables
//
// The variables defined here are accessed from both the Geant4 core and the
// ROS 2 interface code.  They are intentionally kept simple and thread‑safe so
// that updates from the odom_subscriber can safely modify the parameters
// consumed by Geant4 at runtime.  For multi‑source support we add
// g_sources and g_source_intensities; each entry in g_sources corresponds to a
// spatial coordinate (x,y,z) for a Cs‑137 source.  If g_sources is empty then
// the legacy single‑source variables g_source_x/y/z and g_source_intensity are
// used instead.

// Pending events counter – number of unprocessed Geant4 events
extern std::atomic<long> g_pending_events;

// Number of completed events (used for measurement statistics)
extern std::atomic<long> g_b;

// Last input energy in MeV (mapped from robot speed)
extern std::atomic<double> g_energy_MeV;

// Last robot speed in m/s
extern std::atomic<double> g_last_speed_mps;

// Last yaw angle in radians (used for directional dependence when sampling a beam)
extern std::atomic<double> g_yaw_rad;

// PDG code of the selected particle; -1 implies geantino
extern std::atomic<int> g_particle_pdg;

// Energy deposition of the most recent event (for logging)
extern std::atomic<double> g_last_edep_MeV;

// -----------------------------------------------------------------------------
// Legacy single‑source parameters
//
// For backward compatibility the simulation still accepts a single source via
// g_source_x/y/z and g_source_intensity.  If multiple sources are specified
// these variables are ignored.
extern std::atomic<double> g_source_x;
extern std::atomic<double> g_source_y;
extern std::atomic<double> g_source_z;
extern std::atomic<double> g_source_intensity;

// -----------------------------------------------------------------------------
// Multi‑source parameters
//
// Positions of all sources.  Each entry is an array<3> containing [x, y, z]
// coordinates in metres.  When non‑empty, these positions override
// g_source_x/y/z.
extern std::vector<std::array<double, 3>> g_sources;

// Optional intensities per source.  The i‑th element scales the number of
// primaries generated for the i‑th source.  If empty or shorter than
// g_sources the default g_source_intensity is used for all.
extern std::vector<double> g_source_intensities;

// Noise settings
extern std::atomic<bool>   g_noise_enabled;
extern std::atomic<double> g_noise_strength;

// Detector position
extern std::atomic<double> g_detector_x;
extern std::atomic<double> g_detector_y;
extern std::atomic<double> g_detector_z;

// Shield rotation angle in degrees
extern std::atomic<double> g_shield_angle_deg;