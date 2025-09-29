#include <rclcpp/rclcpp.hpp>
#include "odom_subscriber.hpp"
#include "sim_shared.hpp"

// Geant4
#include <G4RunManagerFactory.hh>
#include <G4PhysListFactory.hh>
#include <G4UImanager.hh>
#include "geant4/DetectorConstruction.hh"
#include "geant4/ActionInitialization.hh"

// Include radioactive decay physics modules
#include <G4DecayPhysics.hh>
#include <G4RadioactiveDecayPhysics.hh>

#include <thread>
#include <chrono>
#include <atomic>
#include <iostream>
#include <algorithm>

int main(int argc, char** argv)
{
  // --- ROS 2 initialization ---
  rclcpp::init(argc, argv);
  auto ros_node = std::make_shared<OdomSubscriber>();

  // Spin ROS on a separate thread
  std::atomic<bool> running{true};
  std::thread ros_thread([&](){
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(ros_node);
    using namespace std::chrono_literals;
    while (running.load()) {
      exec.spin_some();
      std::this_thread::sleep_for(5ms);
    }
  });

  // --- Geant4 initialization (Serial + quiet) ---
  auto* runManager = G4RunManagerFactory::CreateRunManager(G4RunManagerType::Serial);

  // 1) Detector construction
  runManager->SetUserInitialization(new DetectorConstruction());

  // 2) Physics list: start from FTFP_BERT and add radioactive decay physics
  G4PhysListFactory factory;
  auto phys = factory.GetReferencePhysList("FTFP_BERT");
  // Register decay and radioactive decay modules so that long‑lived nuclei such as
  // Cs‑137 will decay during the simulation.
  phys->RegisterPhysics(new G4DecayPhysics());
  phys->RegisterPhysics(new G4RadioactiveDecayPhysics());
  runManager->SetUserInitialization(phys);

  // 3) Action initialization (primary generator and event action)
  runManager->SetUserInitialization(new ActionInitialization());

  // Initialize run manager and suppress verbose output
  runManager->Initialize();
  auto* UI = G4UImanager::GetUIpointer();
  UI->ApplyCommand("/run/verbose 0");
  UI->ApplyCommand("/event/verbose 0");
  UI->ApplyCommand("/tracking/verbose 0");
  UI->ApplyCommand("/process/verbose 0");
  // Enable radioactive decay for very long half‑lives: reduce threshold so that
  // Cs‑137 (half‑life ~30 years) will decay in the time frame of the event.
  UI->ApplyCommand("/process/had/rdm/thresholdForVeryLongDecayTime 1.0e+60 year");

  std::cout << "[INFO] Geant4 initialized (Serial). /odom -> BeamOn(1) per event. Edep publishes on /g4/edep" << std::endl;

  // Main loop: process /odom requests sequentially, one event at a time
  using namespace std::chrono_literals;
  const int kBatch = 1;
  while (rclcpp::ok()) {
    int pending = g_pending_events.exchange(0, std::memory_order_acq_rel);
    if (pending > 0) {
      while (pending-- > 0) {
        runManager->BeamOn(kBatch);
        // Optional: print event count and deposition
        // long b = g_b.load(std::memory_order_relaxed);
        // std::cout << "[G4] BeamOn(" << kBatch << ") done. b=" << b << std::endl;
      }
    } else {
      std::this_thread::sleep_for(5ms);
    }
  }

  running.store(false);
  if (ros_thread.joinable()) ros_thread.join();

  delete runManager;
  rclcpp::shutdown();
  return 0;
}