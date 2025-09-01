#include <rclcpp/rclcpp.hpp>
#include "odom_subscriber.hpp"
#include "sim_shared.hpp"

// Geant4
#include <G4RunManagerFactory.hh>
#include <G4PhysListFactory.hh>
#include <G4UImanager.hh>
#include "geant4/DetectorConstruction.hh"
#include "geant4/ActionInitialization.hh"

#include <thread>
#include <chrono>
#include <atomic>
#include <iostream>
#include <algorithm>

int main(int argc, char** argv)
{
  // --- ROS 2 初期化 ---
  rclcpp::init(argc, argv);
  auto ros_node = std::make_shared<OdomSubscriber>();

  // 別スレッドでROSスピン
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

  // --- Geant4 初期化（Serial＋静音）---
  auto* runManager = G4RunManagerFactory::CreateRunManager(G4RunManagerType::Serial);

  // 1) Detector
  runManager->SetUserInitialization(new DetectorConstruction());

  // 2) PhysicsList
  G4PhysListFactory factory;
  auto phys = factory.GetReferencePhysList("FTFP_BERT");
  runManager->SetUserInitialization(phys);

  // 3) ActionInitialization
  runManager->SetUserInitialization(new ActionInitialization());

  // 初期化 & 冗長出力オフ
  runManager->Initialize();
  auto* UI = G4UImanager::GetUIpointer();
  UI->ApplyCommand("/run/verbose 0");
  UI->ApplyCommand("/event/verbose 0");
  UI->ApplyCommand("/tracking/verbose 0");
  UI->ApplyCommand("/process/verbose 0");

  std::cout << "[INFO] Geant4 initialized (Serial). /odom -> BeamOn(1) per event. Edep publishes on /g4/edep"
            << std::endl;

  // メインループ：/odom 要求を逐次で消化（厳密に1イベントずつ）
  using namespace std::chrono_literals;
  const int kBatch = 1;
  while (rclcpp::ok()) {
    int pending = g_pending_events.exchange(0, std::memory_order_acq_rel);
    if (pending > 0) {
      while (pending-- > 0) {
        runManager->BeamOn(kBatch);
        // 表示は控えめに（必要ならコメント解除）
        // auto b = g_b.load(std::memory_order_relaxed);
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
