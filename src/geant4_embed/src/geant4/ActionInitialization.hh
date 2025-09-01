#pragma once
#include <G4VUserActionInitialization.hh>

/**
 * @brief アクション初期化クラス
 *
 * 初期生成アクション・イベントアクションをセットする。
 */
class ActionInitialization : public G4VUserActionInitialization
{
public:
  ActionInitialization() = default;
  virtual ~ActionInitialization() = default;

  /// ワーカースレッド用アクション登録
  virtual void Build() const override;

  /// マスタースレッド用アクション登録
  virtual void BuildForMaster() const override {}
};
