#pragma once
#include <G4UserEventAction.hh>

class G4Event;

/**
 * @brief イベント終了時にエネルギー付与量を集計し共有変数に格納するアクション
 *
 * ノイズ有無・強度に従って測定値に Poisson ノイズを付加します。
 */
class EventAction : public G4UserEventAction
{
public:
  EventAction() = default;
  ~EventAction() override = default;

  /// イベント終了時の処理
  void EndOfEventAction(const G4Event* event) override;
};
