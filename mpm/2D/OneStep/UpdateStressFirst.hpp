#pragma once

#include "OneStep.hpp"
class MPMbox;

struct UpdateStressFirst : public OneStep {
  std::string getRegistrationName();
  int advanceOneStep(MPMbox& MPM);
};
