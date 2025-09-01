#pragma once
#include <G4VUserActionInitialization.hh>

class ActionInitialization : public G4VUserActionInitialization
{
public:
  ActionInitialization() = default;
  virtual ~ActionInitialization() = default;

  virtual void Build() const override;
  virtual void BuildForMaster() const override {}
};
