#pragma once
#include <G4VUserPrimaryGeneratorAction.hh>

class G4ParticleGun;
class G4Event;

/**
 * @brief 初期粒子の生成を行うアクション
 *
 * beam パラメータや外部パラメータから粒子種・エネルギー・位置・方向を設定します。
 */
class PrimaryGeneratorAction : public G4VUserPrimaryGeneratorAction
{
public:
  PrimaryGeneratorAction();
  ~PrimaryGeneratorAction() override;

  /// 粒子生成を実行
  void GeneratePrimaries(G4Event* event) override;

private:
  G4ParticleGun* fGun;
};
